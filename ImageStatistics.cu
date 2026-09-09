/*
 * Copyright (C) 2025-2026: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include <memory>
#include <thread>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <ImageStatistics.h>
#include <Display.h>
using namespace asdp;
using namespace asdp::render;
using namespace asdp::render::imageStatistics;

/// Maximum block size for the compute shader work group.
/// This is the portion of the image that each work group of invocations will process.
static const size_t BLOCK_SIZE = 32;

/// @brief Load and compile the MeanStd compute shader, returning the linked program.
/// @param status [out] Set to an error string on failure, empty on success.
/// @return The compiled/linked GL program, or 0 on failure.
static GLuint BuildMeanStdComputeProgram(std::string& status)
{
  static const char* kComputeSource =
    R"(#version 430

// Must match BLOCK_SIZE in ImageStatistics.cu / MeanStdImpl.
layout(local_size_x = 32, local_size_y = 32) in;

// Bound via glTextureView() to the same texel storage as the source GL_R16 texture,
// reinterpreted as r16ui so imageLoad() returns the raw unsigned 16-bit value.
layout(r16ui, binding = 0) uniform readonly uimage2D uImage;

// Each workgroup writes its own partial sum/sumOfSquares to a unique slot, indexed by
// workgroup ID. No atomics are needed because every workgroup owns a distinct pair of
// slots. The CPU (or a second reduction pass) sums these partials afterward.
// Using two 32-bit components (.xy) per accumulator to avoid needing 64-bit integer
// support in the shader at all: we accumulate in double-precision floating point
// instead, which comfortably holds exact integer sums for the pixel-count/value
// ranges involved (16-bit pixel values, workgroups up to 1024 pixels each).
layout(std430, binding = 1) buffer PartialSums {
  dvec2 partials[]; // partials[i].x = sum, partials[i].y = sumOfSquares, for workgroup i
};

shared double sharedSum[32 * 32];
shared double sharedSumOfSquares[32 * 32];

void main() {
  ivec2 coord = ivec2(gl_GlobalInvocationID.xy);
  uint tid = gl_LocalInvocationIndex;

  uint pixelValue = imageLoad(uImage, coord).r;
  double pixelValueSquared = double(pixelValue) * double(pixelValue);

  sharedSum[tid] = double(pixelValue);
  sharedSumOfSquares[tid] = pixelValueSquared;

  barrier();

  // Reduce within the workgroup (same tree reduction as ComputeMeanStdKernel in ImageStatistics.cu).
  for (uint stride = (32u * 32u) / 2u; stride > 0u; stride >>= 1u) {
    if (tid < stride) {
      sharedSum[tid] += sharedSum[tid + stride];
      sharedSumOfSquares[tid] += sharedSumOfSquares[tid + stride];
    }
    barrier();
  }

  if (tid == 0u) {
    uint groupIndex = gl_WorkGroupID.y * gl_NumWorkGroups.x + gl_WorkGroupID.x;
    partials[groupIndex] = dvec2(sharedSum[0], sharedSumOfSquares[0]);
  }
})";

  GLuint shader = glCreateShader(GL_COMPUTE_SHADER);
  glShaderSource(shader, 1, &kComputeSource, nullptr);
  glCompileShader(shader);

  GLint compiled = GL_FALSE;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
  if (compiled != GL_TRUE) {
    GLint logLen = 0;
    glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &logLen);
    std::string log(logLen, '\0');
    glGetShaderInfoLog(shader, logLen, nullptr, log.data());
    glDeleteShader(shader);
    status = "MeanStd compute shader failed to compile: " + log;
    return 0;
  }

  GLuint program = glCreateProgram();
  glAttachShader(program, shader);
  glLinkProgram(program);
  glDeleteShader(shader);

  GLint linked = GL_FALSE;
  glGetProgramiv(program, GL_LINK_STATUS, &linked);
  if (linked != GL_TRUE) {
    GLint logLen = 0;
    glGetProgramiv(program, GL_INFO_LOG_LENGTH, &logLen);
    std::string log(logLen, '\0');
    glGetProgramInfoLog(program, logLen, nullptr, log.data());
    glDeleteProgram(program);
    status = "MeanStd compute program failed to link: " + log;
    return 0;
  }

  return program;
}

/// Provides implementation details for the MeanStd class
class MeanStd::MeanStdImpl {
public:
  friend class MeanStd;
  MeanStdImpl() = delete;
  MeanStdImpl(MeanStd *parent, std::shared_ptr<CameraRenderInfo> camera)
    : m_parent(parent)
    , m_camera(camera)
  {
    // Make sure the image is an even multiple of the block size in each dimension.
    if (camera->m_resolutionPixels[0] % BLOCK_SIZE != 0 || camera->m_resolutionPixels[1] % BLOCK_SIZE != 0) {
      m_constructorStatus = "Image dimensions must be an even multiple of the block size";
      return;
    }
    m_width = camera->m_resolutionPixels[0];
    m_height = camera->m_resolutionPixels[1];
    m_numWorkGroupsX = m_width / static_cast<GLuint>(BLOCK_SIZE);
    m_numWorkGroupsY = m_height / static_cast<GLuint>(BLOCK_SIZE);
    m_numWorkGroups = static_cast<size_t>(m_numWorkGroupsX) * static_cast<size_t>(m_numWorkGroupsY);

    // Build (once) the compute program used for all instances/frames.
    m_program = BuildMeanStdComputeProgram(m_constructorStatus);
    if (m_program == 0) {
      return;
    }

    // Allocate the SSBO that holds one (sum, sumOfSquares) pair per workgroup.
    glGenBuffers(1, &m_ssbo);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_ssbo);
    glBufferData(GL_SHADER_STORAGE_BUFFER, m_numWorkGroups * 2 * sizeof(double), nullptr, GL_DYNAMIC_COPY);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

    // Reusable CPU-side staging buffer for reading back the partial sums.
    m_partialSumsCPU.resize(m_numWorkGroups * 2);

    GLenum err = glGetError();
    if (err != GL_NO_ERROR) {
      m_constructorStatus = "Failed to allocate MeanStd SSBO: GL error " + std::to_string(err);
      return;
    }
  }

  ~MeanStdImpl()
  {
    // Free the cached texture view for whichever texture we last bound, if any.
    if (m_viewTexture != 0) {
      glDeleteTextures(1, &m_viewTexture);
    }
    if (m_ssbo != 0) {
      glDeleteBuffers(1, &m_ssbo);
    }
    if (m_program != 0) {
      glDeleteProgram(m_program);
    }
  }

  /// @brief Ensure we have a r16ui texture view aliasing the given texture's storage.
  /// @details glTextureView() requires the source texture to have been created with
  /// glTexStorage2D() (immutable storage). If the camera's images are created with
  /// glTexImage2D() instead, switch that call site to glTexStorage2D()+glTexSubImage2D()
  /// so that texture views are legal. The view is cached and only rebuilt if the
  /// source texture handle changes between calls.
  std::string EnsureTextureView(GLuint sourceTexture)
  {
    if (m_viewTexture != 0 && m_viewSourceTexture == sourceTexture) {
      return "";
    }
    if (m_viewTexture != 0) {
      glDeleteTextures(1, &m_viewTexture);
      m_viewTexture = 0;
    }

    glGenTextures(1, &m_viewTexture);
    // Alias the storage of sourceTexture (internal format GL_R16) as GL_R16UI so that
    // imageLoad() in the shader returns the raw 16-bit integer bit pattern.
    glTextureView(m_viewTexture, GL_TEXTURE_2D, sourceTexture, GL_R16UI, 0, 1, 0, 1);

    GLenum err = glGetError();
    if (err != GL_NO_ERROR) {
      glDeleteTextures(1, &m_viewTexture);
      m_viewTexture = 0;
      return "glTextureView() failed with GL error " + std::to_string(err)
        + " (source texture must use immutable storage created with glTexStorage2D())";
    }

    m_viewSourceTexture = sourceTexture;
    return "";
  }

  std::string Compute(double& mean, double& stddev) const
  {
    if (m_constructorStatus != "") {
      return "Constructor failed: " + m_constructorStatus;
    }

#if !defined(NDEBUG)
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) {
      return "OpenGL error at start of Compute(): " + std::to_string(err);
    }
#endif

    // Lock the most-recent image from the camera.
    std::list< std::shared_ptr<ImageData> > images = m_camera->m_imageQueue->LockNewestImages(1);
    if (images.size() == 0) {
      return "No images available";
    }
    std::shared_ptr<ImageData> image = images.front();

    // Make (or reuse) the r16ui view aliasing the source texture's storage.
    std::string viewStatus = const_cast<MeanStdImpl*>(this)->EnsureTextureView(image->texture);
    if (viewStatus != "") {
      m_camera->m_imageQueue->UnlockImage(image);
      return viewStatus;
    }

    // No need to zero the SSBO first: every workgroup unconditionally writes its own slot,
    // so there is no partial/stale-data concern.
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, m_ssbo);

    // Bind the texture view as an image and dispatch the compute shader.
    glBindImageTexture(0, m_viewTexture, 0, GL_FALSE, 0, GL_READ_ONLY, GL_R16UI);
    glUseProgram(m_program);
    glDispatchCompute(m_numWorkGroupsX, m_numWorkGroupsY, 1);

    // We're done reading from the texture and the SSBO writes are enqueued; release the image lock now.
    m_camera->m_imageQueue->UnlockImage(image);

    // Ensure shader writes to the SSBO are visible before we read them back, and create a
    // fence so the CPU-side wait below only blocks on this dispatch (not the whole context).
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
    GLsync fence = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
    if (fence == nullptr) {
      GLenum fenceErr = glGetError();
      return "glFenceSync() failed: GL error " + std::to_string(fenceErr);
    }

    // Wait (with a generous timeout) for the dispatch to complete.
    GLenum waitResult = glClientWaitSync(fence, GL_SYNC_FLUSH_COMMANDS_BIT, 1000000000 /* 1 second */);
    glDeleteSync(fence);
    if (waitResult != GL_ALREADY_SIGNALED && waitResult != GL_CONDITION_SATISFIED) {
      return "glClientWaitSync() failed or timed out waiting for MeanStd dispatch: code " + std::to_string(waitResult);
    }

    // Read back all of the per-workgroup partial sums and finish the reduction on the CPU.
    // This is cheap: e.g. a 1280x1024 image with 32x32 workgroups is only 40x32 = 1280 entries.
    glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, m_partialSumsCPU.size() * sizeof(double), m_partialSumsCPU.data());
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

    double sum = 0.0;
    double sumOfSquares = 0.0;
    for (size_t i = 0; i < m_numWorkGroups; i++) {
      sum += m_partialSumsCPU[i * 2 + 0];
      sumOfSquares += m_partialSumsCPU[i * 2 + 1];
    }

    // Compute the mean and standard deviation knowing the number of pixels.
    double numPixels = static_cast<double>(m_width) * static_cast<double>(m_height);
    mean = sum / numPixels;
    double variance = sumOfSquares / numPixels - mean * mean;
    stddev = sqrt(variance);

#if !defined(NDEBUG)
    err = glGetError();
    if (err != GL_NO_ERROR) {
      return "OpenGL error at end of Compute(): " + std::to_string(err);
    }
#endif

    return "";
  }

  MeanStd* m_parent = nullptr;
  std::string m_constructorStatus;
  std::shared_ptr<CameraRenderInfo> m_camera; ///< Camera to use.

  uint16_t m_width = 0; ///< Width of the image, stored from the camera info.
  uint16_t m_height = 0; ///< Height of the image, stored from the camera info.
  GLuint m_numWorkGroupsX = 0;    ///< Number of workgroups dispatched in X.
  GLuint m_numWorkGroupsY = 0;    ///< Number of workgroups dispatched in Y.
  size_t m_numWorkGroups = 0;     ///< Total number of workgroups (= m_numWorkGroupsX * m_numWorkGroupsY).

  GLuint m_program = 0;              ///< Compiled/linked compute shader program (built once per instance).
  GLuint m_ssbo = 0;                 ///< Shader storage buffer holding one (sum, sumOfSquares) pair per workgroup.
  mutable GLuint m_viewTexture = 0;         ///< Cached r16ui view aliasing the most recent source texture.
  mutable GLuint m_viewSourceTexture = 0;   ///< Which source texture m_viewTexture currently aliases.
  mutable std::vector<double> m_partialSumsCPU; ///< Reusable staging buffer for reading back per-workgroup partials.
};

MeanStd::MeanStd(std::shared_ptr<CameraRenderInfo> camera)
{
  // Create the implementation.
  m_impl = std::make_unique<MeanStdImpl>(this, camera);
  m_constructorStatus = m_impl->m_constructorStatus;
}

std::string MeanStd::Compute(double& mean, double& stddev) const
{
  if (!m_constructorStatus.empty()) {
    return "Constructor failed: " + m_constructorStatus;
  }

  return m_impl->Compute(mean, stddev);
}


MeanStdGroup::MeanStdGroup(std::vector< std::shared_ptr<CameraRenderInfo> > cameras,
    std::shared_ptr<Display> display,
    double updateInterval)
  : m_cameras(cameras)
  , m_display(display)
  , m_updateInterval(updateInterval)
{
  // Start the thread that will update the statistics.
  m_stopThread = false;
  m_updateThread = std::thread(&MeanStdGroup::UpdateThread, this);
}

MeanStdGroup::~MeanStdGroup()
{
  // Signal the thread to stop and wait for it to finish.
  m_stopThread = true;
  if (m_updateThread.joinable()) {
    m_updateThread.join();
  }
}

std::string MeanStdGroup::GetMeanStd(double& mean, double& stddev) const
{
  if (m_status != "") {
    return "Class failed: " + m_status;
  }

  // Lock the mutex to access the vectors.
  std::lock_guard<std::mutex> lock(m_mutex);

  // If we have no entries yet, or we're stopping, return 0.0 for mean and stddev.
  if (m_means.size() == 0 || m_stopThread) {
    mean = stddev = 0.0;
    return "";
  }

  // Compute the mean of the means and the max of the standard deviations in the vectors.
  double sum = 0.0;
  double maxStddev = 0.0;
  for (size_t i = 0; i < m_means.size(); i++) {
    sum += m_means[i];
    if (m_stds[i] > maxStddev) {
      maxStddev = m_stds[i];
    }
  }
  mean = sum / m_means.size();

  // Compute the standard deviation of the means and add it to the maximum of the standard
  // deviations to compute the aggregate standard deviation.
  double sumOfSquares = 0.0;
  for (size_t i = 0; i < m_means.size(); i++) {
    sumOfSquares += (m_means[i] - mean) * (m_means[i] - mean);
  }
  stddev = sqrt(sumOfSquares / m_means.size()) + maxStddev;

  return "";
}

void MeanStdGroup::UpdateThread()
{
  // Start with the first camera.
  size_t nextCamera = 0;

  // Get the start time and compute the next time to update.
  std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
  long long durationMicroseconds = m_updateInterval * 1e6;
  std::chrono::steady_clock::time_point nextUpdate = now + std::chrono::microseconds(durationMicroseconds);

  // Loop until we are told to stop.
  while (!m_stopThread) {

    // Sleep until the next update time and then increase the update time by the duration.
    std::this_thread::sleep_until(nextUpdate);
    nextUpdate += std::chrono::microseconds(durationMicroseconds);

    // Borrow the context needed for our operations
    if (!m_display->BorrowContext()) {
      std::lock_guard<std::mutex> lock(m_mutex);
      m_status = "MeanStdGroup::UpdateThread(): BorrowContext() failed";
      break;
    }

    // Find out which is the next camera to update. If we have fewer entries than cameras, add a new one.
    // Otherwise, loop through the cameras.
    nextCamera = (nextCamera + 1) % m_cameras.size();
    if (m_means.size() < m_cameras.size()) {
      std::lock_guard<std::mutex> lock(m_mutex);
      m_means.push_back(0.0);
      m_stds.push_back(0.0);
      nextCamera = m_means.size() - 1;
      // Make the new entry to compute the mean and standard deviation.
      m_meanStds.push_back(std::make_shared<MeanStd>(m_cameras[nextCamera]));
    }

    // Compute the mean and standard deviation for the camera
    double mean, stddev;
    std::string res = m_meanStds[nextCamera]->Compute(mean, stddev);
    if (res != "") {
      std::lock_guard<std::mutex> lock(m_mutex);
      m_status = "MeanStdGroup::UpdateThread(): MeanStd::Compute() failed: " + res;
      break;
    }

    // Done with the context
    if (!m_display->ReturnContext()) {
      std::lock_guard<std::mutex> lock(m_mutex);
      m_status = "MeanStdGroup::UpdateThread(): ReturnContext() failed";
      break;
    }

    // Convert the mean and standard deviation to common units by adjusting by the
    // camera offset and gain. We multiply both by the gain and add the offset to the mean.
    float offset, gain;
    m_cameras[nextCamera]->GetColorOffsetGain(offset, gain);
    mean = (mean + offset) * gain;
    stddev = stddev * gain;

    // Overwrite the mean and standard deviation in the vectors.
    {
      std::lock_guard<std::mutex> lock(m_mutex);
      m_means[nextCamera] = mean;
      m_stds[nextCamera] = stddev;
    }
  }
}

//================================================================================================
// Testing and its helper functions and classes.

float MeanStd::SpeedTestSingleCalculation(uint16_t width, uint16_t height)
{
  // Create a window and OpenGL context.
  if (!glfwInit()) {
    return -1;
  }
  glfwWindowHint(GLFW_VISIBLE, false);
  std::shared_ptr<GLFWwindow> window(glfwCreateWindow(640, 480, "MeanStd Test", NULL, NULL), glfwDestroyWindow);
  if (!window) {
    return -1;
  }
  glfwMakeContextCurrent(window.get());

  // Initialize GLEW in our context. It is okay to initialize it more than once.
  glewExperimental = true;
  if (glewInit() != GLEW_OK) {
    return -1;
  }
  // Clear any GL error that Glew caused.  Apparently on Non-Windows
  // platforms, this can cause a spurious error 1280.
  glGetError();

  // Construct the object.
  DistortionNone* dNone = new DistortionNone();
  std::shared_ptr<Distortion> distortion(dNone);
  VignetteNone* vNone = new VignetteNone();
  std::shared_ptr<Vignette> vignette(vNone);
  std::shared_ptr<ImageQueue> queue(new ImageQueue);

  std::shared_ptr<CameraRenderInfo> camera(new CameraRenderInfo(
    1, { 0, 0, 0 }, { 0, 0, 0 }, { width, height }, { 90.0, 90.0 }, distortion, vignette, queue, -1.0f));
  MeanStd meanStd(camera);
  if (meanStd.m_constructorStatus != "") {
    return -1;
  }

  // Add an image to the queue.
  // Use a grey-filled image.
  // Construct an OpenGL texture and copy the image into it.
  std::vector<uint16_t> blankImage(width * height, 32768);
  GLuint texture;
  glGenTextures(1, &texture);
  glBindTexture(GL_TEXTURE_2D, texture);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexStorage2D(GL_TEXTURE_2D, 1, GL_R16, width, height);
  glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_RED, GL_UNSIGNED_SHORT, blankImage.data());
  glBindTexture(GL_TEXTURE_2D, 0);
  std::shared_ptr<ImageData> image(new ImageData);
  image->texture = texture;
  queue->InsertImage(image);

  // Run timing on a number of iterations and report the average.
  const size_t iterations = 1000;
  std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
  for (size_t i = 0; i < iterations; i++) {
    std::string res;
    double mean, stddev;
    res = meanStd.Compute(mean, stddev);
    if (res != "") {
      return -1;
    }
  }
  std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = end - start;
  return elapsed.count() / iterations;
}

std::string MeanStd::Test()
{
  // Create a window and OpenGL context.
  if (!glfwInit()) {
    return "Could not initialize GLFW";
  }
  glfwWindowHint(GLFW_VISIBLE, false);
  std::shared_ptr<GLFWwindow> window(glfwCreateWindow(640, 480, "DepthEstimator Test", NULL, NULL), glfwDestroyWindow);
  if (!window) {
    return "Could not create GLFW window";
  }
  glfwMakeContextCurrent(window.get());

  // Initialize GLEW in our context. It is okay to initialize it more than once.
  glewExperimental = true;
  if (glewInit() != GLEW_OK) {
    return "Could not initialize GLEW";
  }
  // Clear any GL error that Glew caused.  Apparently on Non-Windows
  // platforms, this can cause a spurious error 1280.
  glGetError();

  // Test the constructor and Compute() function.
  {
    // Test the constructor.
    uint16_t width = 1280;
    uint16_t height = 1024;
    DistortionNone* dNone = new DistortionNone();
    std::shared_ptr<Distortion> distortion(dNone);
    VignetteNone* vNone = new VignetteNone();
    std::shared_ptr<Vignette> vignette(vNone);
    std::shared_ptr<ImageQueue> queue(new ImageQueue);

    std::shared_ptr<CameraRenderInfo> camera(new CameraRenderInfo(
      1, { 0, 0, 0 }, { 0, 0, 0 }, { width, height }, { 90.0, 90.0 }, distortion, vignette, queue, -1.0f));
    MeanStd meanStd(camera);
    if (meanStd.m_constructorStatus != "") {
      return "MeanStd constructor failed: " + meanStd.m_constructorStatus;
    }

    // Add an image to the queue.
    // Use a grey-filled image.
    // Construct an OpenGL texture and copy the image into it.
    std::vector<uint16_t> blankImage(width * height, 32768);
    GLuint texture;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexStorage2D(GL_TEXTURE_2D, 1, GL_R16, width, height);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_RED, GL_UNSIGNED_SHORT, blankImage.data());
    glBindTexture(GL_TEXTURE_2D, 0);
    std::shared_ptr<ImageData> image(new ImageData);
    image->texture = texture;
    queue->InsertImage(image);

    // Test the Compute() function.
    double mean, stddev;
    std::string res = meanStd.Compute(mean, stddev);
    if (res != "") {
      return "MeanStd::Compute() failed for constant image: " + res;
    }
    if (mean != 32768.0) {
      return "MeanStd::Compute() failed for constant image: mean is not 32768.0";
    }
    if (stddev != 0.0) {
      return "MeanStd::Compute() failed for constant image: stddev is not 0.0";
    }

    // Make an image that is half black and half white.
    std::vector<uint16_t> halfBlackHalfWhite(width * height, 0);
    for (size_t i = 0; i < width * height / 2; i++) {
      halfBlackHalfWhite[i] = 65535;
    }
    queue->GetOldestImage();
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_RED, GL_UNSIGNED_SHORT, halfBlackHalfWhite.data());
    glBindTexture(GL_TEXTURE_2D, 0);
    queue->InsertImage(image);
    res = meanStd.Compute(mean, stddev);
    if (res != "") {
      return "MeanStd::Compute() failed for split image: " + res;
    }
    if (mean != 32767.5) {
      return "MeanStd::Compute() failed for split image: mean is not 32767.5 but " + std::to_string(mean);
    }
    if (stddev != 32767.5) {
      return "MeanStd::Compute() failed for split image: stddev is not 32767.5 but " + std::to_string(stddev);
    }
  }

  // Try a constructor with an image whose size is not an even multiple of the block size.  It should fail.
  {
    uint16_t width = 1281;
    uint16_t height = 1024;
    DistortionNone* dNone = new DistortionNone();
    std::shared_ptr<Distortion> distortion(dNone);
    VignetteNone* vNone = new VignetteNone();
    std::shared_ptr<Vignette> vignette(vNone);
    std::shared_ptr<ImageQueue> queue(new ImageQueue);

    std::shared_ptr<CameraRenderInfo> camera(new CameraRenderInfo(
      1, { 0, 0, 0 }, { 0, 0, 0 }, { width, height }, { 90.0, 90.0 }, distortion, vignette, queue, -1.0f));
    MeanStd meanStd(camera);
    if (meanStd.m_constructorStatus != "Image dimensions must be an even multiple of the block size") {
      return "MeanStd constructor failed to detect non-even multiple of block size";
    }

  }

  return "";
}

std::string MeanStdGroup::Test()
{
  // Create a window and OpenGL context.
  if (!glfwInit()) {
    return "Could not initialize GLFW";
  }
  glfwWindowHint(GLFW_VISIBLE, false);
  std::shared_ptr<GLFWwindow> window(glfwCreateWindow(640, 480, "MeanStdGroup Test", NULL, NULL), glfwDestroyWindow);
  if (!window) {
    return "Could not create GLFW window";
  }
  glfwMakeContextCurrent(window.get());

  // Initialize GLEW in our context. It is okay to initialize it more than once.
  glewExperimental = true;
  if (glewInit() != GLEW_OK) {
    return "Could not initialize GLEW";
  }
  // Clear any GL error that Glew caused.  Apparently on Non-Windows
  // platforms, this can cause a spurious error 1280.
  glGetError();

  // Make the display object that we'll use and borrow its context.
  std::shared_ptr<Display> display(new DisplayTexture());
  if (!display->BorrowContext()) {
    return "Display::BorrowContext() failed";
  }

  // Make four cameras with different offsets and gains and with different distributions of pixel values.
  // The first camera has a constant image of 10000 with an offset of 0 and gain of 1.
  // The second has a constant image of 20000 with an offset of 10000 and gain of 1 (making its values 30000).
  // The third has a constant image of 2000 with an offset of 3000 and gain of 2 (making its values 10000).
  // The fourth has a half and half image of 40000 and 20000 with an offset of 0 and gain of 1, making its mean
  // values 30000 and its variance 10000.
  // The total mean should be 20000 and the total standard deviation should be 10000 + 10000 = 20000.
  {
    // Test the constructor.
    uint16_t width = 1280;
    uint16_t height = 1024;
    DistortionNone* dNone = new DistortionNone();
    std::shared_ptr<Distortion> distortion(dNone);
    VignetteNone* vNone = new VignetteNone();
    std::shared_ptr<Vignette> vignette(vNone);

    // Make first camera.
    std::shared_ptr<ImageData> image1(new ImageData);
    std::shared_ptr<ImageQueue> queue1(new ImageQueue);
    std::shared_ptr<CameraRenderInfo> camera1(new CameraRenderInfo(
      1, { 0, 0, 0 }, { 0, 0, 0 }, { width, height }, { 90.0, 90.0 }, distortion, vignette, queue1, -1.0f));

    // Add an image to the queue.
    // Construct an OpenGL texture and copy the image into it.
    std::vector<uint16_t> image10K(width * height, 10000);
    GLuint texture1;
    glGenTextures(1, &texture1);
    glBindTexture(GL_TEXTURE_2D, texture1);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexStorage2D(GL_TEXTURE_2D, 1, GL_R16, width, height);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_RED, GL_UNSIGNED_SHORT, image10K.data());
    glBindTexture(GL_TEXTURE_2D, 0);
    image1->texture = texture1;
    queue1->InsertImage(image1);

    // Make the second camera.
    std::shared_ptr<ImageData> image2(new ImageData);
    std::shared_ptr<ImageQueue> queue2(new ImageQueue);
    std::shared_ptr<CameraRenderInfo> camera2(new CameraRenderInfo(
      1, { 0, 0, 0 }, { 0, 0, 0 }, { width, height }, { 90.0, 90.0 }, distortion, vignette, queue2, -1.0f));
    camera2->SetColorOffsetGain(10000.0, 1.0);

    // Add an image to the queue.
    // Construct an OpenGL texture and copy the image into it.
    std::vector<uint16_t> image20K(width * height, 20000);
    GLuint texture2;
    glGenTextures(1, &texture2);
    glBindTexture(GL_TEXTURE_2D, texture2);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexStorage2D(GL_TEXTURE_2D, 1, GL_R16, width, height);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_RED, GL_UNSIGNED_SHORT, image20K.data());
    glBindTexture(GL_TEXTURE_2D, 0);
    image2->texture = texture2;
    queue2->InsertImage(image2);

    // Make the third camera.
    std::shared_ptr<ImageData> image3(new ImageData);
    std::shared_ptr<ImageQueue> queue3(new ImageQueue);
    std::shared_ptr<CameraRenderInfo> camera3(new CameraRenderInfo(
      1, { 0, 0, 0 }, { 0, 0, 0 }, { width, height }, { 90.0, 90.0 }, distortion, vignette, queue3, -1.0f));
    camera3->SetColorOffsetGain(3000.0, 2.0);

    // Add an image to the queue.
    // Construct an OpenGL texture and copy the image into it.
    std::vector<uint16_t> image2K(width * height, 2000);
    GLuint texture3;
    glGenTextures(1, &texture3);
    glBindTexture(GL_TEXTURE_2D, texture3);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexStorage2D(GL_TEXTURE_2D, 1, GL_R16, width, height);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_RED, GL_UNSIGNED_SHORT, image2K.data());
    glBindTexture(GL_TEXTURE_2D, 0);
    image3->texture = texture3;
    queue3->InsertImage(image3);

    // Make the fourth camera.
    std::shared_ptr<ImageData> image4(new ImageData);
    std::shared_ptr<ImageQueue> queue4(new ImageQueue);
    std::shared_ptr<CameraRenderInfo> camera4(new CameraRenderInfo(
      1, { 0, 0, 0 }, { 0, 0, 0 }, { width, height }, { 90.0, 90.0 }, distortion, vignette, queue4, -1.0f));

    // Add an image to the queue.
    // Construct an OpenGL texture and copy the image into it.
    size_t imgSize = static_cast<size_t>(width) * height;
    std::vector<uint16_t> image40K20K(imgSize, 20000);
    for (size_t i = 0; i < imgSize / 2; i++) {
      image40K20K[i] = 40000;
    }
    GLuint texture4;
    glGenTextures(1, &texture4);
    glBindTexture(GL_TEXTURE_2D, texture4);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexStorage2D(GL_TEXTURE_2D, 1, GL_R16, width, height);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_RED, GL_UNSIGNED_SHORT, image40K20K.data());
    glBindTexture(GL_TEXTURE_2D, 0);
    image4->texture = texture4;
    queue4->InsertImage(image4);

    // Done with the display context.
    if (!display->ReturnContext()) {
      return "Display::ReturnContext() failed";
    }

    // Make a vector of cameras and construct the MeanStdGroup with a 0.1-second iteration time.
    std::vector< std::shared_ptr<CameraRenderInfo> > cameras = {
      camera1, camera2, camera3, camera4 };
    MeanStdGroup meanStdGroup(cameras, display, 0.1);

    // When we first start, the mean and standard deviation should be 0 and there should be no
    // entries in the vectors.
    double mean, stddev;
    std::string res = meanStdGroup.GetMeanStd(mean, stddev);
    if (res != "") {
      return "MeanStdGroup::GetMeanStd() failed at start: " + res;
    }
    if (mean != 0.0) {
      return "MeanStdGroup::GetMeanStd() failed at start: mean is not 0.0";
    }
    if (stddev != 0.0) {
      return "MeanStdGroup::GetMeanStd() failed at start: stddev is not 0.0";
    }

    // Wait until 0.05 seconds after there is one entry in the vectors so that the calculation
    // has time to complete. The mean should be 10000 and the standard deviation should be 0.
    while (meanStdGroup.m_means.size() < 1) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    res = meanStdGroup.GetMeanStd(mean, stddev);
    if (res != "") {
      return "MeanStdGroup::GetMeanStd() failed for first camera: " + res;
    }
    if (mean != 10000.0) {
      return "MeanStdGroup::GetMeanStd() failed for first camera: mean is not 10000.0";
    }
    if (stddev != 0.0) {
      return "MeanStdGroup::GetMeanStd() failed for first camera: stddev is not 0.0";
    }

    // Wait until 0.05 seconds after there are two entries in the vectors so that the calculation
    // has time to complete. The mean should be 20000 and the standard deviation should be 10000.
    while (meanStdGroup.m_means.size() < 2) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    res = meanStdGroup.GetMeanStd(mean, stddev);
    if (res != "") {
      return "MeanStdGroup::GetMeanStd() failed for second camera: " + res;
    }
    if (mean != 20000.0) {
      return "MeanStdGroup::GetMeanStd() failed for second camera: mean is not 20000.0";
    }
    if (stddev != 10000.0) {
      return "MeanStdGroup::GetMeanStd() failed for second camera: stddev is not 10000.0";
    }

    // Wait until 0.05 seconds after there are four cameras so that the calculation
    // has time to complete. The mean should be 20000 and the standard deviation should be 30000.
    while (meanStdGroup.m_means.size() < 4) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    res = meanStdGroup.GetMeanStd(mean, stddev);
    if (res != "") {
      return "MeanStdGroup::GetMeanStd() failed for all cameras: " + res;
    }
    if (mean != 20000.0) {
      return "MeanStdGroup::GetMeanStd() failed for all cameras: mean is not 20000.0 but " + std::to_string(mean);
    }
    if (stddev != 20000.0) {
      return "MeanStdGroup::GetMeanStd() failed for all cameras: stddev is not 20000.0 but " + std::to_string(stddev);
    }
  }

  return "";
}
