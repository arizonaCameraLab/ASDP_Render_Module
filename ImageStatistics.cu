/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include <iostream>
#include <chrono>
#include <memory>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <ImageStatistics.h>
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
__global__ void ComputeMeanStdKernel(cudaSurfaceObject_t surface, uint64_t* outSum, uint64_t* outSumOfSquares)
{
  /// Block of memory to store the within-block results.
  __shared__ uint64_t sharedSum[BLOCK_SIZE * BLOCK_SIZE];
  __shared__ uint64_t sharedSquareSum[BLOCK_SIZE * BLOCK_SIZE];

  // Global coordinates in the surface.
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  int idy = blockIdx.y * blockDim.y + threadIdx.y;
  int tid = threadIdx.y * blockDim.x + threadIdx.x;

  // Read the 16-but pixel value. We must multiply by pixel size in X because it is indexed in bytes
  uint16_t pixelValue;
  surf2Dread(&pixelValue, surface, idx * sizeof(pixelValue), idy);
  // Compute the square of the pixel value.
  uint64_t pixelValueSquared = uint64_t(pixelValue) * pixelValue;
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
  MeanStdImpl(MeanStd *parent, std::shared_ptr<asdp::render::CameraRenderInfo> camera)
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
    res = cudaMalloc(&m_sum, sizeof(uint64_t));
    if (res != cudaSuccess) {
      m_constructorStatus = "cudaMalloc() failed: " + std::string(cudaGetErrorString(res));
      return;
    }
    res = cudaMalloc(&m_sumOfSquares, sizeof(uint64_t));
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

    GLenum err = glGetError();
    if (err != GL_NO_ERROR) {
      return "OpenGL error at start of Compute(): " + std::to_string(err);
    }

    // Lock a texture for CUDA to use and then map it to CUDA.
    std::list< std::shared_ptr<ImageData> > images = m_camera->m_imageQueue->LockNewestImages(1);
    if (images.size() == 0) {
      return "No images available";
    }
    std::shared_ptr<ImageData> image = images.front();

    cudaGraphicsResource* cgr;
    res = cudaGraphicsGLRegisterImage(&cgr, image->texture, GL_TEXTURE_2D, cudaGraphicsRegisterFlagsReadOnly);
    if (res != cudaSuccess) {
      return "cudaGraphicsGLRegisterImage() failed: " + std::string(cudaGetErrorString(res));
    }
    res = cudaGraphicsMapResources(1, &cgr, *m_stream);
    if (res != cudaSuccess) {
      return "cudaGraphicsMapResources() failed: " + std::string(cudaGetErrorString(res));
    }
    cudaArray* array;
    res = cudaGraphicsSubResourceGetMappedArray(&array, cgr, 0, 0);
    if (res != cudaSuccess) {
      return "cudaGraphicsSubResourceGetMappedArray() failed: " + std::string(cudaGetErrorString(res));
    }
    cudaSurfaceObject_t surfObj;
    cudaResourceDesc resDesc;
    memset(&resDesc, 0, sizeof(resDesc));
    resDesc.resType = cudaResourceTypeArray;
    resDesc.res.array.array = array;
    res = cudaCreateSurfaceObject(&surfObj, &resDesc);
    if (res != cudaSuccess) {
      return "cudaCreateSurfaceObject() failed: " + std::string(cudaGetErrorString(res));
    }

    // Zero the sum and sum of squares.
    res = cudaMemset(m_sum, 0, sizeof(uint64_t));
    if (res != cudaSuccess) {
      return "cudaMemset() failed: " + std::string(cudaGetErrorString(res));
    }
    res = cudaMemset(m_sumOfSquares, 0, sizeof(uint64_t));
    if (res != cudaSuccess) {
      return "cudaMemset() failed: " + std::string(cudaGetErrorString(res));
    }

    // Run the kernel on the stream.
    dim3 blockSize(BLOCK_SIZE, BLOCK_SIZE);
    dim3 gridSize(m_width / BLOCK_SIZE, m_height / BLOCK_SIZE);
    ComputeMeanStdKernel << <gridSize, blockSize, 0, *m_stream >> > (surfObj, m_sum, m_sumOfSquares);
    cudaDeviceSynchronize();

    // Unlock the image.
    m_camera->m_imageQueue->UnlockImage(image);

    // Read back the results.
    uint64_t sum, sumOfSquares;
    res = cudaMemcpy(&sum, m_sum, sizeof(uint64_t), cudaMemcpyDeviceToHost);
    if (res != cudaSuccess) {
      return "cudaMemcpy() failed: " + std::string(cudaGetErrorString(res));
    }
    res = cudaMemcpy(&sumOfSquares, m_sumOfSquares, sizeof(uint64_t), cudaMemcpyDeviceToHost);
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
  std::shared_ptr<asdp::render::CameraRenderInfo> m_camera; ///< Camera to use.

  uint16_t m_width = 0; ///< Width of the image, stored from the camera info.
  uint16_t m_height = 0; ///< Height of the image, stored from the camera info.

  std::shared_ptr<cudaStream_t> m_stream; ///< CUDA stream to use.

  uint64_t* m_sum = nullptr; ///< Device poitner to sum of pixel values.
  uint64_t* m_sumOfSquares = nullptr; ///< Device pointer to sum of squares of pixel values.
};

MeanStd::MeanStd(std::shared_ptr<asdp::render::CameraRenderInfo> camera)
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

//================================================================================================
// Testing and its helper functions and classes.

float MeanStd::SpeedTestSingleCalculation(uint16_t width, uint16_t height)
{
  // Create a window and OpenGL context.
  if (!glfwInit()) {
    return -1;
  }
  glfwWindowHint(GLFW_VISIBLE, false);
  std::shared_ptr<GLFWwindow> window(glfwCreateWindow(640, 480, "DepthEstimator Test", NULL, NULL), glfwDestroyWindow);
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

  asdp::render::CameraRenderInfo* camera = new asdp::render::CameraRenderInfo(
    1, { 0, 0, 0 }, { 0, 0, 0 }, { width, height }, { 90.0, 90.0 }, distortion, vignette, queue);
  std::shared_ptr<asdp::render::CameraRenderInfo> cameraPtr(camera);
  MeanStd meanStd(cameraPtr);
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
  std::shared_ptr<asdp::render::ImageData> image(new ImageData);
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

    asdp::render::CameraRenderInfo* camera = new asdp::render::CameraRenderInfo(
      1, { 0, 0, 0 }, { 0, 0, 0 }, { width, height }, { 90.0, 90.0 }, distortion, vignette, queue);
    std::shared_ptr<asdp::render::CameraRenderInfo> cameraPtr(camera);
    MeanStd meanStd(cameraPtr);
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
    std::shared_ptr<asdp::render::ImageData> image(new ImageData);
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

    asdp::render::CameraRenderInfo* camera = new asdp::render::CameraRenderInfo(
      1, { 0, 0, 0 }, { 0, 0, 0 }, { width, height }, { 90.0, 90.0 }, distortion, vignette, queue);
    std::shared_ptr<asdp::render::CameraRenderInfo> cameraPtr(camera);
    MeanStd meanStd(cameraPtr);
    if (meanStd.m_constructorStatus != "Image dimensions must be an even multiple of the block size") {
      return "MeanStd constructor failed to detect non-even multiple of block size";
    }

  }

  return "";
}
