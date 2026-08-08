/*
 * Copyright (C) 2025: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include <iostream>
#include <chrono>
#include <memory>
#include <thread>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <ImageStatistics.h>
#include <Display.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>
using namespace asdp;
using namespace asdp::render;
using namespace asdp::render::imageStatistics;

/// Maximum block size for the CUDA kernel.
/// This is the portion of the image that each block of threads will process.
static const size_t BLOCK_SIZE = 32;

/// @brief CUDA kernel to compute the mean and standard deviation of an image.
/// @details This kernel sums across the entire block and then reduces the sums to a single value each.
/// It then does an atomic add to accumulate them into the final sum and sum of squares.
/// NOTE: The blockDim.x and blockDim.y must evenly divide the width and height of the image.
/// @param surface The surface object for the image.
/// @param outSum The sum of the pixel values.
/// @param outSumOfSquares The sum of the squares of the pixel values.
__global__ void ComputeMeanStdKernel(cudaSurfaceObject_t surface, unsigned long long* outSum, unsigned long long* outSumOfSquares)
{
  /// Block of memory to store the within-block results.
  __shared__ unsigned long long sharedSum[BLOCK_SIZE * BLOCK_SIZE];
  __shared__ unsigned long long sharedSquareSum[BLOCK_SIZE * BLOCK_SIZE];

  // Global coordinates in the surface.
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  int idy = blockIdx.y * blockDim.y + threadIdx.y;
  int tid = threadIdx.y * blockDim.x + threadIdx.x;

  // Read the 16-but pixel value. We must multiply by pixel size in X because it is indexed in bytes
  uint16_t pixelValue;
  surf2Dread(&pixelValue, surface, idx * sizeof(pixelValue), idy);
  // Compute the square of the pixel value.
  unsigned long long pixelValueSquared = ((unsigned long long)(pixelValue)) * pixelValue;
  sharedSum[tid] = pixelValue;
  sharedSquareSum[tid] = pixelValueSquared;

  __syncthreads();

  // Reduce within block
  for (int stride = blockDim.x * blockDim.y / 2; stride > 0; stride >>= 1) {
    if (tid < stride) {
      sharedSum[tid] += sharedSum[tid + stride];
      sharedSquareSum[tid] += sharedSquareSum[tid + stride];
    }
    __syncthreads();
  }

  // One thread per block writes the result to global memory
  if (tid == 0) {
    atomicAdd(outSum, sharedSum[0]);
    atomicAdd(outSumOfSquares, sharedSquareSum[0]);
  }
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

    // Make an auto-deleted CUDA stream.
    cudaStream_t* streamPtr = new cudaStream_t;
    cudaError_t res = cudaStreamCreate(streamPtr);
    if (res != cudaSuccess) {
      m_constructorStatus = "cudaStreamCreate() failed: " + std::string(cudaGetErrorString(res));
      delete streamPtr;
      return;
    }
    std::shared_ptr<cudaStream_t> stream(streamPtr, [](cudaStream_t* ptr) { cudaStreamDestroy(*ptr); delete ptr; });
    m_stream = stream;

    // Allocate the output variables on the device side.
    res = cudaMalloc(&m_sum, sizeof(unsigned long long));
    if (res != cudaSuccess) {
      m_constructorStatus = "cudaMalloc() failed: " + std::string(cudaGetErrorString(res));
      return;
    }
    res = cudaMalloc(&m_sumOfSquares, sizeof(unsigned long long));
    if (res != cudaSuccess) {
      m_constructorStatus = "cudaMalloc() failed: " + std::string(cudaGetErrorString(res));
      return;
    }
  }

  ~MeanStdImpl()
  {
    // Free our resources
    if (m_sumOfSquares) {
      cudaFree(m_sumOfSquares);
    }
    if (m_sum) {
      cudaFree(m_sum);
    }
  }

  std::string Compute(double& mean, double& stddev) const
  {
    cudaError_t res;

    if (m_constructorStatus != "") {
      return "Constructor failed: " + m_constructorStatus;
    }

#if !defined(NDEBUG)
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) {
      return "OpenGL error at start of Compute(): " + std::to_string(err);
    }
#endif

    // Lock a texture for CUDA to use and then map it to CUDA.
    std::list< std::shared_ptr<ImageData> > images = m_camera->m_imageQueue->LockNewestImages(1);
    if (images.size() == 0) {
      return "No images available";
    }
    std::shared_ptr<ImageData> image = images.front();

    cudaGraphicsResource* cgr;
    res = cudaGraphicsGLRegisterImage(&cgr, image->texture, GL_TEXTURE_2D, cudaGraphicsRegisterFlagsReadOnly);
    if (res != cudaSuccess) {
      m_camera->m_imageQueue->UnlockImage(image);
      return "cudaGraphicsGLRegisterImage() failed: " + std::string(cudaGetErrorString(res));
    }
    res = cudaGraphicsMapResources(1, &cgr, *m_stream);
    if (res != cudaSuccess) {
      m_camera->m_imageQueue->UnlockImage(image);
      return "cudaGraphicsMapResources() failed: " + std::string(cudaGetErrorString(res));
    }
    cudaArray* array;
    res = cudaGraphicsSubResourceGetMappedArray(&array, cgr, 0, 0);
    if (res != cudaSuccess) {
      m_camera->m_imageQueue->UnlockImage(image);
      return "cudaGraphicsSubResourceGetMappedArray() failed: " + std::string(cudaGetErrorString(res));
    }
    cudaSurfaceObject_t surfObj;
    cudaResourceDesc resDesc;
    memset(&resDesc, 0, sizeof(resDesc));
    resDesc.resType = cudaResourceTypeArray;
    resDesc.res.array.array = array;
    res = cudaCreateSurfaceObject(&surfObj, &resDesc);
    if (res != cudaSuccess) {
      m_camera->m_imageQueue->UnlockImage(image);
      return "cudaCreateSurfaceObject() failed: " + std::string(cudaGetErrorString(res));
    }

    // Zero the sum and sum of squares.
    res = cudaMemsetAsync(m_sum, 0, sizeof(unsigned long long), *m_stream);
    if (res != cudaSuccess) {
      m_camera->m_imageQueue->UnlockImage(image);
      return "cudaMemset() failed: " + std::string(cudaGetErrorString(res));
    }
    res = cudaMemsetAsync(m_sumOfSquares, 0, sizeof(unsigned long long), *m_stream);
    if (res != cudaSuccess) {
      m_camera->m_imageQueue->UnlockImage(image);
      return "cudaMemset() failed: " + std::string(cudaGetErrorString(res));
    }

    // Run the kernel on the stream.
    dim3 blockSize(BLOCK_SIZE, BLOCK_SIZE);
    dim3 gridSize(m_width / BLOCK_SIZE, m_height / BLOCK_SIZE);
    ComputeMeanStdKernel << <gridSize, blockSize, 0, *m_stream >> > (surfObj, m_sum, m_sumOfSquares);

    // Unlock the image.
    m_camera->m_imageQueue->UnlockImage(image);

    // Read back the results.
    unsigned long long sum, sumOfSquares;
    res = cudaMemcpyAsync(&sum, m_sum, sizeof(unsigned long long), cudaMemcpyDeviceToHost, *m_stream);
    if (res != cudaSuccess) {
      return "cudaMemcpy() failed: " + std::string(cudaGetErrorString(res));
    }
    res = cudaMemcpyAsync(&sumOfSquares, m_sumOfSquares, sizeof(unsigned long long), cudaMemcpyDeviceToHost, *m_stream);
    if (res != cudaSuccess) {
      return "cudaMemcpy() failed: " + std::string(cudaGetErrorString(res));
    }

    // Compute the mean and standard deviation knowing the number of pixels.
    double numPixels = m_width * m_height;
    mean = sum / numPixels;
    double variance = sumOfSquares / numPixels - mean * mean;
    stddev = sqrt(variance);

    // Done with surface object and other CUDA objects.
    res = cudaDestroySurfaceObject(surfObj);
    if (res != cudaSuccess) {
      return "cudaDestroySurfaceObject() failed: " + std::string(cudaGetErrorString(res));
    }
    res = cudaGraphicsUnmapResources(1, &cgr, *m_stream);
    if (res != cudaSuccess) {
      return "cudaGraphicsUnmapResources() failed: " + std::string(cudaGetErrorString(res));
    }
    res = cudaGraphicsUnregisterResource(cgr);
    if (res != cudaSuccess) {
      return "cudaGraphicsUnregisterResource() failed: " + std::string(cudaGetErrorString(res));
    }

    return "";
  }

  MeanStd* m_parent = nullptr;
  std::string m_constructorStatus;
  std::shared_ptr<CameraRenderInfo> m_camera; ///< Camera to use.

  uint16_t m_width = 0; ///< Width of the image, stored from the camera info.
  uint16_t m_height = 0; ///< Height of the image, stored from the camera info.

  std::shared_ptr<cudaStream_t> m_stream; ///< CUDA stream to use.

  unsigned long long* m_sum = nullptr; ///< Device poitner to sum of pixel values.
  unsigned long long* m_sumOfSquares = nullptr; ///< Device pointer to sum of squares of pixel values.
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

  // If we have no entries yet, return 0.0 for mean and stddev.
  if (m_means.size() == 0) {
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

    // Compute the mean and standard deviation for the camera, borrowing the context needed
    if (!m_display->BorrowContext()) {
      std::lock_guard<std::mutex> lock(m_mutex);
      m_status = "MeanStdGroup::UpdateThread(): BorrowContext() failed";
      break;
    }
    double mean, stddev;
    std::string res = m_meanStds[nextCamera]->Compute(mean, stddev);
    if (res != "") {
      std::lock_guard<std::mutex> lock(m_mutex);
      m_status = "MeanStdGroup::UpdateThread(): MeanStd::Compute() failed: " + res;
      break;
    }
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
  glTexImage2D(GL_TEXTURE_2D, 0, GL_R16, width, height, 0, GL_RED, GL_UNSIGNED_SHORT, blankImage.data());
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
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R16, width, height, 0, GL_RED, GL_UNSIGNED_SHORT, blankImage.data());
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
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R16, width, height, 0, GL_RED, GL_UNSIGNED_SHORT, halfBlackHalfWhite.data());
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
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R16, width, height, 0, GL_RED, GL_UNSIGNED_SHORT, image10K.data());
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
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R16, width, height, 0, GL_RED, GL_UNSIGNED_SHORT, image20K.data());
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
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R16, width, height, 0, GL_RED, GL_UNSIGNED_SHORT, image2K.data());
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
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R16, width, height, 0, GL_RED, GL_UNSIGNED_SHORT, image40K20K.data());
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
