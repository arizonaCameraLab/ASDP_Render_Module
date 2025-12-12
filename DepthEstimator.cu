/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include <iostream>
#include <chrono>
#include <memory>
#include <map>
#include <cstddef>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <ToneMap.h>
#include <DepthEstimator.h>
#include <Composite.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>
using namespace asdp;
using namespace asdp::render;

//==================================================================================================
// Helper classes for dealing with vectors and Quaternions because we can't use GLM in CUDA kernels.
// They are available on both host and device.

struct Vec3 {
  float vals[3];
  __host__ __device__ Vec3() : vals{0, 0, 0} {}
  __host__ __device__ Vec3(float x, float y, float z) : vals{x, y, z} {}
  __host__ __device__ Vec3(glm::vec3 v) : vals{ v.x, v.y, v.z } {}
  __host__ __device__ Vec3(glm::dvec3 v) : vals{ (float)v.x, (float)v.y, (float)v.z } {}

  // Index operator.
  __host__ __device__ float& operator[](size_t index) {
    return vals[index];
  }

  // Arithmetic operations.
  __host__ __device__ Vec3 operator-(const Vec3& other) const {
    return Vec3(vals[0] - other.vals[0], vals[1] - other.vals[1], vals[2] - other.vals[2]);
  }
  __host__ __device__ Vec3 operator+(const Vec3& other) const {
    return Vec3(vals[0] + other.vals[0], vals[1] + other.vals[1], vals[2] + other.vals[2]);
  }
  __host__ __device__ Vec3 operator*(float scalar) const {
    return Vec3(vals[0] * scalar, vals[1] * scalar, vals[2] * scalar);
  }
  __host__ __device__ bool operator==(const Vec3 other) const {
    return (vals[0] == other.vals[0]) && (vals[1] == other.vals[1]) && (vals[2] == other.vals[2]);
  }
  __host__ __device__ bool operator!=(const Vec3 other) const {
    return !((*this) == other);
  }

  // Normalization.
  __host__ __device__ float Length() const {
    return sqrt(vals[0] * vals[0] + vals[1] * vals[1] + vals[2] * vals[2]);
  }
  __host__ __device__ Vec3 Normalize() const {
    float length = Length();
    if (length > 0.0f) {
      return Vec3(vals[0] / length, vals[1] / length, vals[2] / length);
    } else {
      return Vec3(0, 0, 0);
    }
  }

  // Dot product.
  __host__ __device__ float Dot(const Vec3& other) const {
    return vals[0] * other.vals[0] + vals[1] * other.vals[1] + vals[2] * other.vals[2];
  }
};

struct Quat {
  float vals[4];
  __host__ __device__ Quat() : vals{0, 0, 0, 1} {}
  __host__ __device__ Quat(float x, float y, float z, float w) : vals{x, y, z, w} {}
  __host__ __device__ Quat(glm::quat q) : vals{ (float)q.x, (float)q.y, (float)q.z, (float)q.w } {}
  __host__ __device__ Quat(glm::dquat q) : vals{ (float)q.x, (float)q.y, (float)q.z, (float)q.w } {}

  // Index operator.
  __host__ __device__ float& operator[](size_t index) {
    return vals[index];
  }

  // Quaternion multiplication of a Vec3 to rotate it.
  __host__ __device__ Vec3 operator*(const Vec3& v) const {
    // Rotate vector v by this quaternion.
    Vec3 qVec(vals[0], vals[1], vals[2]);
    Vec3 t = qVec * (2.0f * qVec.Dot(v));
    Vec3 u = v * (vals[3] * vals[3] - qVec.Dot(qVec));
    Vec3 vCrossQ = Vec3(
      vals[1] * v.vals[2] - vals[2] * v.vals[1],
      vals[2] * v.vals[0] - vals[0] * v.vals[2],
      vals[0] * v.vals[1] - vals[1] * v.vals[0]
    );
    Vec3 w = vCrossQ * (2.0f * vals[3]);
    return t + u + w;
  }
};

//==================================================================================================

/// Maximum block size for the CUDA kernel, which matches the maximum number of samples in X or Y.
/// This is the size of the image that each block of threads will process.  This includes all of
/// the passes for the block, so it is the number of pixels in X and Y divided by the number of
/// regions in X and Y.
static const size_t MAX_BLOCK_SIZE = 100;

/// @brief CUDA kernel to compute the differences between two surfaces (OpenGL textures).
/// @details The kernel computes the mean squared difference between the corresponding pixels in two
/// RGBA or BGRA textures.  The kernel is designed to be run in blocks of threads, with each block
/// processing a region of the image.  Because the number of pixels in a block is larger than the
/// maximum number of threads in a block, each thread may handle pixels from multiple rows.  To minimize
/// calculations in the kernel, the number of iterations is a parameter to the kernel, as is the number
/// of rows in an iteration and the number of rows in each block.  The number of columns in the block is
/// always the same as the X block dimension.  The kernel uses shared memory to store the results for
/// each thread in the block and then computes the sum and count of the valid values in each row in a
/// subset of the threads.  It the sums the sums and counts and computes the final average using a
/// single thread.
/// @param surface1, surface2 The surfaces to compare. They are RGBA or BGRA textures.
/// @param out The per-region array of output values to write.
/// @param iterations The number of iterations to run.
/// @param rowsPerIteration The number of rows to process in each iteration.
/// @param rowsPerBlock The number of rows to process in each block.
__global__ void CompareSurfacesKernel(cudaSurfaceObject_t surface1, cudaSurfaceObject_t surface2, float* out,
  unsigned iterations, unsigned rowsPerIteration, unsigned rowsPerBlock)
{
  /// Block of memory to store the within-block results.
  __shared__ float sharedMem[MAX_BLOCK_SIZE][MAX_BLOCK_SIZE];
  __shared__ float rowSums[MAX_BLOCK_SIZE];
  __shared__ int rowCounts[MAX_BLOCK_SIZE];

  // Global coordinates in the images.
  // There are as many Y threads as there are rows in an iteration, with potentially many iterations
  // per block.  The number of rows per block is how much should be skipped in Y rather than the
  // block to cause each block of threads to handle that portion of the image.  We offset each thread
  // within the first iteration here to compute a base, which is then bumped per iteration.  There is
  // enough room in shared memory for the entire block (all iterations).
  unsigned x = blockIdx.x * blockDim.x + threadIdx.x;
  unsigned yBase = blockIdx.y * rowsPerBlock + threadIdx.y;

  unsigned xLocal = threadIdx.x;
  for (unsigned iter = 0; iter < iterations; iter++) {
    unsigned yLocal = threadIdx.y + iter * rowsPerIteration;
    unsigned y = yBase + iter * rowsPerIteration;

    // The block size matches the number of threads in X, and the block size evenly divides the image,
    // so we don't need to check bounds on that axis.
    if (yLocal < rowsPerBlock) {
      // Read the data from both surfaces. The x coordinate is in bytes, so we need to multiply by the
      // size of the data type.
      // The image size is a multiple of the block size, so we don't need to check for out-of-bounds.
      uchar4 val1, val2;
      surf2Dread(&val1, surface1, x * sizeof(val1), y);
      surf2Dread(&val2, surface2, x * sizeof(val2), y);

      // If the first and third colors are not the same in either of the values, then the region is outside
      // of the projected area (the pixel is blue), so we record -1 as the value.  Otherwise, we record the
      // squared difference between the first color in each value.  We have enough entries for all threads
      // in the block, we use the thread index to determine where to write.
      if ((val1.x != val1.z) || (val2.x != val2.z)) {
        sharedMem[yLocal][xLocal] = -1.0f;
      } else {
        float diff = val1.x - val2.x;
        sharedMem[yLocal][xLocal] = diff * diff;
      }
    }
  }

  // Wait until all threads in the block have completed and then have the first thread in each
  // row compute the sum and count of the valid values in the row.
  __syncthreads();
  for (unsigned iter = 0; iter < iterations; iter++) {
    unsigned yLocal = threadIdx.y + iter * rowsPerIteration;
    if (yLocal < rowsPerBlock) {
      // @todo We can get better utilization if we have threads in different columns handle
      // the different rows rather than looping over iterations.
      if (threadIdx.x == 0) {
        rowSums[yLocal] = 0.0f;
        rowCounts[yLocal] = 0;
        for (size_t i = 0; i < blockDim.x; i++) {
          float val = sharedMem[yLocal][i];
          if (val >= 0.0f) {
            rowSums[yLocal] += val;
            rowCounts[yLocal]++;
          }
        }
      }
    }
  }

  // Wait until all threads in the block have completed and then have one thread sum the
  // sum and counts and compute the final average.
  __syncthreads();
  if (threadIdx.x == 0 && threadIdx.y == 0) {
    float sum = 0.0f;
    int count = 0;
    // Sum over the entire block's worth of rows, not just the first iteration's
    for (size_t i = 0; i < rowsPerBlock; i++) {
      sum += rowSums[i];
      count += rowCounts[i];
    }

    // Avoid division by zero, return 0 in the case of no valid values.
    if (count == 0) { count = 1; }
    out[blockIdx.x + blockIdx.y * gridDim.x] = sum/count;
  }
}

/// @brief Encapsulates the multiple depths for each camera pair.
/// @details Generate two sets of CompositeCameras covering all depths for each pair of cameras,
/// one set for the left camera and one for the right camera. 
class CameraPairInfo {
public:
  CameraPairInfo() = delete;
  CameraPairInfo(ToneMap const &toneMap, std::shared_ptr<CameraRenderInfo> camera1, std::shared_ptr<CameraRenderInfo> camera2,
    std::shared_ptr<asdp::render::RangeEstimator> rangeEstimator,
    glm::dvec3 position, glm::dquat orientation,
    std::array<float, 2> fovsDeg, std::array<unsigned, 2> pixelCounts,
    std::shared_ptr<PoseAdjuster> poseAdjuster, Time cameraFrameInterval,
    std::vector<float> depths, float defaultDepth)
    : m_cameras({camera1, camera2})
    , m_position(position)
    , m_orientation(orientation)
    , m_fovsDeg(fovsDeg)
    , m_pixelCounts(pixelCounts)
    , m_poseAdjuster(poseAdjuster)
  {
    // Construct the tone map to use.
    m_toneMapTexture = toneMap.GenerateTexture();

    // Create the composite cameras for each depth, adjusting the camera render info to suit.
    for (float depth : depths) {
      PerDepth depthInfo;

      depthInfo.m_depth = depth;

      // We make a copy of each camera and then adjust the copy to the specific depth it is to use.
      std::shared_ptr<CameraRenderInfo> depth1(new CameraRenderInfo(*camera1));
      depth1->ComputePlanarCameraMeshInfo(100, 100, depth);
      std::vector< std::shared_ptr<CameraRenderInfo> > composites1;
      composites1.push_back(depth1);
      depthInfo.m_composites[0] = std::make_shared<CompositeCameras>(composites1, m_toneMapTexture,
        m_poseAdjuster, cameraFrameInterval, 0, Time(), nullptr, rangeEstimator);

      std::shared_ptr<CameraRenderInfo> depth2(new CameraRenderInfo(*camera2));
      depth2->ComputePlanarCameraMeshInfo(100, 100, depth);
      std::vector< std::shared_ptr<CameraRenderInfo> > composites2;
      composites2.push_back(depth2);
      depthInfo.m_composites[1] = std::make_shared<CompositeCameras>(composites2, m_toneMapTexture,
        m_poseAdjuster, cameraFrameInterval, 0, Time(), nullptr, rangeEstimator);

      // Generate a pair of frame buffers for the two cameras, with an associated color and depth
      // buffer for each.
      glGenFramebuffers(2, depthInfo.m_frameBuffers.data());

      glGenTextures(2, depthInfo.m_colorBuffers.data());
      for (size_t b = 0; b < depthInfo.m_colorBuffers.size(); b++) {
        glBindTexture(GL_TEXTURE_2D, depthInfo.m_colorBuffers[b]);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, pixelCounts[0], pixelCounts[1], 0,
          GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
        glBindTexture(GL_TEXTURE_2D, 0);
      }

      glGenTextures(2, depthInfo.m_depthBuffers.data());
      for (size_t b = 0; b < depthInfo.m_depthBuffers.size(); b++) {
        glBindTexture(GL_TEXTURE_2D, depthInfo.m_depthBuffers[b]);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32, pixelCounts[0], pixelCounts[1], 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
        glBindTexture(GL_TEXTURE_2D, 0);
      }

      // Map the CUDA graphics resources for the color buffers.
      for (size_t b = 0; b < depthInfo.m_colorBuffers.size(); b++) {
        cudaError_t res = cudaGraphicsGLRegisterImage(&depthInfo.m_cudaColorBuffers[b], depthInfo.m_colorBuffers[b],
          GL_TEXTURE_2D, cudaGraphicsRegisterFlagsSurfaceLoadStore);
        if (res != cudaSuccess) {
          m_constructorStatus = "Failed to register image: " + std::string(cudaGetErrorString(res));
          return;
        }
      }

      // Create the CUDA streams.
      cudaStream_t* streamPtr = new cudaStream_t;
      cudaError_t res = cudaStreamCreate(streamPtr);
      if (res != cudaSuccess) {
        m_constructorStatus = "Failed to create stream: " + std::string(cudaGetErrorString(res));
        return;
      }
      depthInfo.m_stream = streamPtr;

      // Zero the GPU and memory for the region buffer; it will be allocated the first time it is needed.
      depthInfo.m_GPURegionBuffer = nullptr;

      m_perDepths.push_back(depthInfo);
    }
    // Fill in the default depth for all regions.
    m_depths.resize(pixelCounts[0] * pixelCounts[1], defaultDepth);
  }

  ~CameraPairInfo() {
    // Delete the tone map texture.
    glDeleteTextures(1, &m_toneMapTexture);

    // Delete the frame bufffers, color buffers, and depth buffers.
    // Unmap the CUDA graphics resources for the color buffers.
    // Delete the CUDA streams.
    // Free the GPU memory for the depth buffers.
    for (PerDepth &di : m_perDepths) {
      glDeleteFramebuffers(2, di.m_frameBuffers.data());
      for (size_t b = 0; b < di.m_cudaColorBuffers.size(); b++) {
        cudaGraphicsUnregisterResource(di.m_cudaColorBuffers[b]);
      }
      glDeleteTextures(2, di.m_colorBuffers.data());
      glDeleteTextures(2, di.m_depthBuffers.data());
      cudaFree(di.m_GPURegionBuffer);
      cudaStreamDestroy(*(di.m_stream));
    }
  }

  std::array<std::shared_ptr<CameraRenderInfo>,2 > m_cameras;
  std::shared_ptr<PoseAdjuster> m_poseAdjuster;
  glm::dvec3 m_position;
  glm::dquat m_orientation;
  std::array<float, 2> m_fovsDeg;
  std::array<unsigned, 2> m_pixelCounts;
  GLuint m_toneMapTexture;

  class PerDepth {
  public:
    float m_depth = 0.0f;
    std::vector<float> m_CPURegionBuffer;
    float* m_GPURegionBuffer;
    cudaStream_t* m_stream = nullptr;
    /// @todo Consider pulling these out into yet another structure, making an array of 2 of them.
    std::array< std::shared_ptr<CompositeCameras>, 2> m_composites = {};
    std::array<GLuint, 2> m_frameBuffers = {};
    std::array<GLuint, 2> m_colorBuffers = {};
    std::array<GLuint, 2> m_depthBuffers = {};
    std::array<cudaGraphicsResource*, 2> m_cudaColorBuffers = {};
  };

  std::vector<PerDepth> m_perDepths;

  /// Computed estimated depth at every region location.
  std::vector<float> m_depths;

  std::string m_constructorStatus;
};

//==================================================================================================
// Object that forms a packed structure that contains the relevant entries from all available
// CameraPairInfo objects in a DepthEstimatorImpl for use in CUDA kernels.
struct CameraPairsKernelData {
  CameraPairsKernelData() = delete;

  /// @brief Constructor for the CPU-side code allocates memory and fills in the data.
  CameraPairsKernelData(std::vector< std::shared_ptr<CameraPairInfo> > const& cameraPairs)
    : m_cameraPairs(cameraPairs)
  {
    totalSizeFloats = 1;

    // The fixed-size camera pair info for each pair
    totalSizeFloats += cameraPairs.size() * CameraPairBaseInfoSize;

    // The depth information for each pair
    for (auto const& pair : cameraPairs) {
      totalSizeFloats += pair->m_depths.size();
    }

    // Allocate the pinned CPU data and the GPU data
    if (cudaMallocHost(&data, totalSizeFloats * sizeof(float)) != cudaSuccess) {
      throw std::runtime_error("Failed to allocate pinned CameraPairsKernelData");
    }
    if (cudaMalloc(&kData, totalSizeFloats * sizeof(float)) != cudaSuccess) {
      throw std::runtime_error("Failed to allocate GPU CameraPairsKernelData");
    }
  }

  void CopyDataToGPU(cudaStream_t stream = 0)
  {
    // Fill in the data on the pinned memory
    numCameraPairsCPU() = static_cast<uint32_t>(m_cameraPairs.size());

    // Fixed camera size data
    for (size_t i = 0; i < m_cameraPairs.size(); i++) {
      auto const& pair = m_cameraPairs[i];
      // Base info
      pairPositionsCPU(i) = Vec3(static_cast<float>(pair->m_position.x),
        static_cast<float>(pair->m_position.y),
        static_cast<float>(pair->m_position.z));
      pairOrientationsCPU(i) = Quat(pair->m_orientation);
      pairFOVsCPU(i)[0] = pair->m_fovsDeg[0];
      pairFOVsCPU(i)[1] = pair->m_fovsDeg[1];
      pairPixelCountsCPU(i)[0] = static_cast<uint32_t>(pair->m_pixelCounts[0]);
      pairPixelCountsCPU(i)[1] = static_cast<uint32_t>(pair->m_pixelCounts[1]);
    }

    // Camera pair depth information
    for (size_t i = 0; i < m_cameraPairs.size(); i++) {
      auto const& pair = m_cameraPairs[i];
      float* depths = pairDepthsCPU(i);
      for (size_t j = 0; j < pair->m_depths.size(); j++) {
        depths[j] = pair->m_depths[j];
      }
    }

    // Initiate a copy of the data to the GPU using the provided stream.
    cudaError_t res = cudaMemcpyAsync(kData, data, totalSizeFloats * sizeof(float), cudaMemcpyHostToDevice, stream);
    if (res != cudaSuccess) {
      throw std::runtime_error("Failed to copy CameraPairsKernelData to GPU: " + std::string(cudaGetErrorString(res)));
    }
  }

  /// @brief Destructor frees the allocated memory.
  ~CameraPairsKernelData()
  {
    cudaFree(data);
    cudaFree(kData);
    data = nullptr;
  }

  /// This is a horrible hack to pack all of the data into a single vector of floats and then
  /// provide accessors to the individual entries, some of which are not floats.
  float* data = nullptr;

  /// This is an even more horrible hack that defines different accessors for the info on the CPU and
  /// kernel sides based on the correct pointer.  This is the pointer to be used on the GPU kernel.
  float* kData = nullptr;

  /// The total size in floats of the data.
  size_t totalSizeFloats = 0;

  /// The size of the base info for each camera pair.
  static const size_t CameraPairBaseInfoSize = 3 + 4 + 2 + 2; // position(3) + orientation(4) + fovs(2) + pixelCounts(2)

  /// The first thing packed is the number of camera pairs.
  __host__ __device__ static uint32_t& numCameraPairs(float *d) { return *reinterpret_cast<unsigned int*>(&d[0]); }
  __host__ uint32_t& numCameraPairsCPU() const { return numCameraPairs(data); }
  __device__ uint32_t& numCameraPairsGPU() const { return numCameraPairs(kData); }

  /// The second batch of things packed is the base info for each camera pair, with all data for each together.
  __host__ __device__ static Vec3& pairPositions(float* d, size_t i) {
    return *reinterpret_cast<Vec3*>(&d[1 + i * CameraPairBaseInfoSize]);
  };
  __host__ Vec3& pairPositionsCPU(size_t i) const { return pairPositions(data, i); };
  __device__ Vec3& pairPositionsGPU(size_t i) const { return pairPositions(kData, i); };

  __host__ __device__ static Quat& pairOrientations(float* d, size_t i) {
    return *reinterpret_cast<Quat*>(&d[1 + i * CameraPairBaseInfoSize + 3]);
  };
  __host__ Quat& pairOrientationsCPU(size_t i) const { return pairOrientations(data, i); };
  __device__ Quat& pairOrientationsGPU(size_t i) const { return pairOrientations(kData, i); };

  __host__ __device__ static float* pairFOVs(float* d, size_t i) {
    return &d[1 + i * CameraPairBaseInfoSize + 7];
  };
  __host__ float* pairFOVsCPU(size_t i) const { return pairFOVs(data, i); };
  __device__ float* pairFOVsGPU(size_t i) const { return pairFOVs(kData, i); };

  __host__ __device__ static uint32_t* pairPixelCounts(float* d, size_t i) {
    return reinterpret_cast<uint32_t*>(&d[1 + i * CameraPairBaseInfoSize + 9]);
  };
  __host__ uint32_t* pairPixelCountsCPU(size_t i) const { return pairPixelCounts(data, i); };
  __device__ uint32_t* pairPixelCountsGPU(size_t i) const { return pairPixelCounts(kData, i); };

  /// The next batch of things packed are the depth values per pair.
  __host__ __device__ static float* pairDepths(float* d, size_t i) {
    size_t depthIndex = 1 + numCameraPairs(d) * CameraPairBaseInfoSize;
    for (size_t p = 0; p < i; p++) {
      uint32_t* pixCounts = pairPixelCounts(d, p);
      depthIndex += pixCounts[0] * pixCounts[1];
    }
    return &d[depthIndex];
  };
  __host__ float* pairDepthsCPU(size_t i) const { return pairDepths(data, i); };
  __device__ float* pairDepthsGPU(size_t i) const { return pairDepths(kData, i); };

protected:
  /// Stores the camera pairs for when we need to refer back to them.
  std::vector< std::shared_ptr<CameraPairInfo> > m_cameraPairs;
};

//==================================================================================================
/// @todo Output data structure to compact and re-fill the depth estimates for a CameraRenderInfo.

struct CameraDepthInfoKernelData {
  CameraDepthInfoKernelData() = delete;

  /// @brief Constructor for the CPU-side code allocates memory and fills in the data.
  CameraDepthInfoKernelData(std::shared_ptr<CameraRenderInfo> cameraRenderInfo) : cri(cameraRenderInfo) {
    // The fixed-size number of elements in X and Y
    size_t totalSizeFloats = CameraBaseInfoSize;

    // The depth information for the camera
    totalSizeFloats += cri->m_mesh.vertexInfo.size();

    // Allocate the data
    if (cudaMallocManaged(&data, totalSizeFloats * sizeof(float)) != cudaSuccess) {
      throw std::runtime_error("Failed to allocate CameraRenderInfoKernelData");
    }

    // Fill in the data
    // Fixed camera size data
    cameraNx() = cri->m_mesh.nx;
    cameraNy() = cri->m_mesh.ny;

    // Camera depth information
    float* depths = cameraDepths();
    for (size_t j = 0; j < cri->m_mesh.vertexInfo.size(); j++) {
      depths[j] = cri->m_mesh.vertexInfo[j].depth;
    }
  }

  /// @brief Fill the depths back into the CameraRenderInfo objects from the data in this structure.
  void FillDepthsBackToCameraRenderInfos() {
    float* depths = cameraDepths();
    for (size_t j = 0; j < cri->m_mesh.vertexInfo.size(); j++) {
      cri->m_mesh.vertexInfo[j].depth = depths[j];
    }
  }

  /// @brief Destructor frees the allocated memory.
  ~CameraDepthInfoKernelData() {
    cudaFree(data);
    data = nullptr;
  }

  /// Stored camera render info used to put depths back.
  std::shared_ptr<CameraRenderInfo> cri;

  /// This is a horrible hack to pack all of the data into a single vector of floats and then
  /// provide accessors to the individual entries, some of which are not floats.
  float* data = nullptr;

  /// The size of the base info for each camera pair.
  const size_t CameraBaseInfoSize = 2; // nx, ny

  /// The first batch of things packed is the nx,ny info for each camera.
  __host__ __device__ uint32_t& cameraNx() const {
    return *reinterpret_cast<uint32_t*>(&data[0]);
  };
  __host__ __device__ uint32_t& cameraNy() const {
    return *reinterpret_cast<uint32_t*>(&data[1]);
  };

  /// The last batch of things packed are the depth values for the camera.
  __host__ __device__ float* cameraDepths() {
    size_t depthIndex = CameraBaseInfoSize;
    return &data[depthIndex];
  }
};

//==================================================================================================

/// Provides implementation details for the DepthEstimator class
class DepthEstimator::DepthEstimatorImpl {
public:
  friend class DepthEstimator;
  DepthEstimatorImpl() = delete;
  DepthEstimatorImpl(DepthEstimator *parent,
      std::vector< std::array<std::shared_ptr<CameraRenderInfo>, 2> > cameras,
      std::shared_ptr<asdp::render::RangeEstimator> rangeEstimator,
      std::shared_ptr<PoseAdjuster> poseAdjuster,
      Time cameraFrameInterval,
      unsigned nx, unsigned ny,
      std::vector<float> depths,
      float fitnessThreshold)
    : m_parent(parent)
    , m_nx(nx)
    , m_ny(ny)
    , m_fitnessThreshold(fitnessThreshold)
  {
    // Find the default depth.
    if (depths.size() == 0) {
      // Error -- no default depth
      m_defaultDepth = -1.0f;
    } else {
      // Use the furthest depth as the default.
      m_defaultDepth = depths.back();
    }

    // Generate CameraPairInfo (two sets of CompositeCameras covering all depths) for each pair of cameras,
    // one set for the left camera and one for the right camera.  The two sets for each depth will be rendered
    // separately into a pair of frame buffers with the same (average of the two cameras) view frustum
    // and then compared to estimate the depth.
    // NOTE: Tonemap must be monochrome because the test code calls all colored pixels background.
    // The default black-to-white one works.
    ToneMap toneMap;
    for (unsigned i = 0; i < cameras.size(); i++) {

      // For each pair, create a viewpoint that is halfway between
      // the two cameras with an orientation that is the average of the two.
      // We store the rotation as a quaternion and convert the Euler angles
      // to quaternions to average them.
      glm::dvec3 position;

      std::array<double, 3> const& p1 = cameras[i][0]->m_positionMeters;
      std::array<double, 3> const& p2 = cameras[i][1]->m_positionMeters;
      position = 0.5 * (glm::dvec3(p1[0], p1[1], p1[2]) + glm::dvec3(p2[0], p2[1], p2[2]));

      glm::quat orientation;
      glm::quat rotx = glm::angleAxis(glm::radians(cameras[i][0]->m_orientationDegrees[0]), glm::dvec3(1, 0, 0));
      glm::quat roty = glm::angleAxis(glm::radians(cameras[i][0]->m_orientationDegrees[1]), glm::dvec3(0, 1, 0));
      glm::quat rotz = glm::angleAxis(glm::radians(cameras[i][0]->m_orientationDegrees[2]), glm::dvec3(0, 0, 1));
      glm::quat rot1 = rotz * roty * rotx;

      rotx = glm::angleAxis(glm::radians(cameras[i][1]->m_orientationDegrees[0]), glm::dvec3(1, 0, 0));
      roty = glm::angleAxis(glm::radians(cameras[i][1]->m_orientationDegrees[1]), glm::dvec3(0, 1, 0));
      rotz = glm::angleAxis(glm::radians(cameras[i][1]->m_orientationDegrees[2]), glm::dvec3(0, 0, 1));
      glm::quat rot2 = rotz * roty * rotx;

      orientation = glm::slerp(rot1, rot2, 0.5f);

      // Determine the FOVs of the frame buffer that will be used to render the manifolds.
      // It should cover the range of the manifolds, including their distortion.  Then determine the
      // pixel count, which should be an even multiple of the number of samples in each dimension
      // and its ratio should be similar to the aspect ration of the frame buffer and it should have
      // at least as many pixels as the camera images in each dimension.  Start by determining the
      // distorted location of a point at the upper-right corner of the camera image on a plane at
      // Z = -1 and computing its fields of view.
      std::array<float, 2> fovsDeg;
      double depthForFOV = 1.0;
      double maxHFOV = 0, maxVFOV = 0;
      double maxXRatio = 1.0, maxYRatio = 1.0;
      for (size_t c = 0; c < 2; c++) {
        double xHalfWidth = tan(glm::radians(cameras[i][c]->m_fovDegrees[0]) * 0.5) * depthForFOV;
        double yHalfWidth = tan(glm::radians(cameras[i][c]->m_fovDegrees[1]) * 0.5) * depthForFOV;

        std::array<double, 3> corner = { xHalfWidth, yHalfWidth, -depthForFOV };
        std::array<double, 3> distortedCorner = cameras[i][c]->m_distortion->MapPoint(corner);

        double hFOV = glm::degrees(2.0 * atan(fabs(distortedCorner[0] / distortedCorner[2])));
        double vFOV = glm::degrees(2.0 * atan(fabs(distortedCorner[1] / distortedCorner[2])));

        maxHFOV = std::max(maxHFOV, hFOV);
        maxVFOV = std::max(maxVFOV, vFOV);

        maxXRatio = std::max(maxXRatio, fabs(distortedCorner[0] / xHalfWidth));
        maxYRatio = std::max(maxYRatio, fabs(distortedCorner[1] / yHalfWidth));
      }
      fovsDeg[0] = maxHFOV;
      fovsDeg[1] = maxVFOV;

      // Use the ratio of the new and original fields of view to scale the pixel count, making sure that
      // the results are an even multiple of the number of samples in X and Y.
      std::array<unsigned, 2> pixelCounts;
      uint16_t maxX = std::max(cameras[i][0]->m_resolutionPixels[0], cameras[i][1]->m_resolutionPixels[0]);
      uint16_t maxY = std::max(cameras[i][0]->m_resolutionPixels[1], cameras[i][1]->m_resolutionPixels[1]);
      pixelCounts[0] = maxX * maxXRatio;
      if (pixelCounts[0] % m_nx != 0) { pixelCounts[0] += m_nx - (pixelCounts[0] % m_nx); }
      pixelCounts[1] = maxY * maxYRatio;
      if (pixelCounts[1] % m_ny != 0) { pixelCounts[1] += m_ny - (pixelCounts[1] % m_ny); }

      //std::cout << "XXX Position: " << position.x << " " << position.y << " " << position.z
      //  << ", Orientation: " << orientation.w << " " << orientation.x << " " << orientation.y << " " << orientation.z
      //  << ", Pixel counts: " << pixelCounts[0] << " " << pixelCounts[1]
      //  << ", Regions: " << m_nx << " " << m_ny
      //  << std::endl;

      // Make the camera pair info.
      std::shared_ptr<CameraPairInfo> cameraPairInfo = std::make_shared<CameraPairInfo>(
        toneMap, cameras[i][0], cameras[i][1], rangeEstimator,
        position, orientation, fovsDeg, pixelCounts,
        poseAdjuster, cameraFrameInterval, depths, m_defaultDepth);
      if (!cameraPairInfo->m_constructorStatus.empty()) {
        m_constructorStatus = cameraPairInfo->m_constructorStatus;
        return;
      }
      m_cameraPairs.push_back(cameraPairInfo);
    }
  }

  ~DepthEstimatorImpl() {
    // Done with all of the CUDA streams used per camera.
    for (auto& pair : m_cameraStreams) {
      cudaStreamDestroy(*pair.second);
      delete pair.second;
    }
  }

  std::string ComputeDepthEstimate(Time time)
  {
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) {
      return "OpenGL error at start of ComputeDepthEstimate(): " + std::to_string(err);
    }

    // OpenGL fence objects to let us ensure that we're done with OpenGL rendering before we
    // start to map the buffers to CUDA and do the depth estimation.  There is one entry
    // for each camera with one entry for each depth with an entry for each of the pair of cameras.
    std::vector < std::vector< std::array<GLsync, 2> > > fences;

    // For each camera pair and depth, render the two cameras into their frame buffers and keep track of the fences.
    for (size_t c = 0; c < m_cameraPairs.size(); c++) {
      CameraPairInfo& cpi = *m_cameraPairs[c];
      std::vector< std::array<GLsync, 2> > cFences;
      for (size_t d = 0; d < cpi.m_perDepths.size(); d++) {
        std::array<GLsync, 2> dFences;
        for (size_t b = 0; b < 2; b++) {
          // Fill in the render info.
          ViewRenderInfo vri;
          for (size_t i = 0; i < 3; i++) {
            vri.viewpoint[i] = cpi.m_position[i];
          }
          // The vri.orientation quaternion is in WXYZ order, but the glm quaternion is in XYZW order.
          vri.orientation[0] = cpi.m_orientation.w;
          vri.orientation[1] = cpi.m_orientation.x;
          vri.orientation[2] = cpi.m_orientation.y;
          vri.orientation[3] = cpi.m_orientation.z;
          vri.leftHalfFOV = -cpi.m_fovsDeg[0] / 2.0f;
          vri.rightHalfFOV = cpi.m_fovsDeg[0] / 2.0f;
          vri.bottomHalfFOV = -cpi.m_fovsDeg[1] / 2.0f;
          vri.topHalfFOV = cpi.m_fovsDeg[1] / 2.0f;
          vri.nearClip = cpi.m_perDepths[d].m_depth / 2;
          vri.farClip = cpi.m_perDepths[d].m_depth * 2;
          vri.frameBuffer = cpi.m_perDepths[d].m_frameBuffers[b];
          vri.colorBuffer = cpi.m_perDepths[d].m_colorBuffers[b];
          vri.depthBuffer = cpi.m_perDepths[d].m_depthBuffers[b];
          vri.x = 0;
          vri.y = 0;
          vri.width = cpi.m_pixelCounts[0];
          vri.height = cpi.m_pixelCounts[1];

          // Render the composite camera and construct a fence to indicate completion.
          err = glGetError();
          if (err != GL_NO_ERROR) {
            return "OpenGL error before Render() for pair " + std::to_string(c)
              + " depth " + std::to_string(d) + " camera " + std::to_string(b) + ": "
              + std::to_string(err);
          }
          cpi.m_perDepths[d].m_composites[b]->Render(time, {vri});
          err = glGetError();
          if (err != GL_NO_ERROR) {
            return "OpenGL error before fence for pair " + std::to_string(c)
              + " depth " + std::to_string(d) + " camera " + std::to_string(b) + ": "
              + std::to_string(err);
          }
          dFences[b] = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
          if (dFences[b] == nullptr) {
            err = glGetError();
            return "glFenceSync() failed for pair " + std::to_string(c)
              + " depth " + std::to_string(d) + " camera " + std::to_string(b)
              + ": error " + std::to_string(err);
          }
        }
        cFences.push_back(dFences);
      }
      fences.push_back(cFences);
    }

    // Loop back through the camera pairs and depths, waiting for both fences to complete
    // and then mapping the color buffers to CUDA and running the depth estimation.
    for (size_t c = 0; c < m_cameraPairs.size(); c++) {
      CameraPairInfo& cpi = *m_cameraPairs[c];

      // The number of regions is the number of regions in X times the number of regions in Y.
      size_t numRegions = m_nx * m_ny;

      // Vector of surface objects to destroy once we're done with them.
      std::vector<cudaSurfaceObject_t> surfObjs;

      for (size_t d = 0; d < cpi.m_perDepths.size(); d++) {
        CameraPairInfo::PerDepth &pd = cpi.m_perDepths[d];

        // Wait for both fences to complete.
        for (size_t b = 0; b < 2; b++) {
          // 1-second timeout.
          GLenum ret = glClientWaitSync(fences[c][d][b], 0, 1000000000);
          if (ret != GL_ALREADY_SIGNALED && ret != GL_CONDITION_SATISFIED) {
            return "glClientWaitSync() failed for pair " + std::to_string(c)
              + " depth " + std::to_string(d) + " camera " + std::to_string(b)
              + ": code " + std::to_string(ret);
          }
        }

        // Map the color buffers to CUDA, using the already-registered images.  Then get the
        // mapped array values.
        std::array< cudaArray*, 2> arrays;
        for (size_t b = 0; b < 2; b++) {
#if 0
          // Read back the texture to a CPU buffer.
          // Write a debugging PPM file named for the camera pair, depth, and camera.
          {
            // Check for OpenGL errors.
            GLenum err = glGetError();
            if (err != GL_NO_ERROR) {
              return "OpenGL error before reading back texture for pair " + std::to_string(c)
                + " depth " + std::to_string(d) + " camera " + std::to_string(b) + ": "
                + std::to_string(err);
            }
            std::vector<uchar4> pixels(cpi.m_pixelCounts[0] * cpi.m_pixelCounts[1]);
            glBindTexture(GL_TEXTURE_2D, pd.m_colorBuffers[b]);
            glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
            glBindTexture(GL_TEXTURE_2D, 0);
            std::ofstream ppmFile("depthEstimator" + std::to_string(c) + "_" + std::to_string(d) + "_" + std::to_string(b) + ".ppm");
            ppmFile << "P3\n" << cpi.m_pixelCounts[0] << " " << cpi.m_pixelCounts[1] << "\n255\n";
            for (size_t y = 0; y < cpi.m_pixelCounts[1]; y++) {
              // The texture has lower-left corner first, but the PPM file has upper-left first.
              size_t flipY = (cpi.m_pixelCounts[1] - 1) - y;
              for (size_t x = 0; x < cpi.m_pixelCounts[0]; x++) {
                uchar4 val = pixels[x + flipY * cpi.m_pixelCounts[0]];
                ppmFile << (int)val.x << " " << (int)val.y << " " << (int)val.z << " ";
              }
              ppmFile << "\n";
            }
          }
#endif

          cudaError_t res = cudaGraphicsMapResources(1, &pd.m_cudaColorBuffers[b], *(pd.m_stream));
          if (res != cudaSuccess) {
            return "cudaGraphicsMapResources() failed for pair " + std::to_string(c)
              + " depth " + std::to_string(d) + " camera " + std::to_string(b) + ": "
              + std::string(cudaGetErrorString(res));
          }
          res = cudaGraphicsSubResourceGetMappedArray(&arrays[b], pd.m_cudaColorBuffers[b], 0, 0);
          if (res != cudaSuccess) {
            return "cudaGraphicsSubResourceGetMappedArray() failed for pair " + std::to_string(c)
              + " depth " + std::to_string(d) + " camera " + std::to_string(b) + ": "
              + std::string(cudaGetErrorString(res));
          }
        }

        // Create surface objects for the color buffers and add them to the list to destroy.
        cudaSurfaceObject_t surfObj1, surfObj2;
        cudaResourceDesc resDesc;
        memset(&resDesc, 0, sizeof(resDesc));
        resDesc.resType = cudaResourceTypeArray;
        resDesc.res.array.array = arrays[0];
        cudaError_t res = cudaCreateSurfaceObject(&surfObj1, &resDesc);
        if (res != cudaSuccess) {
          return "cudaCreateSurfaceObject() failed for pair " + std::to_string(c)
            + " depth " + std::to_string(d) + " camera 0: " + std::string(cudaGetErrorString(res));
        }
        surfObjs.push_back(surfObj1);
        resDesc.res.array.array = arrays[1];
        res = cudaCreateSurfaceObject(&surfObj2, &resDesc);
        if (res != cudaSuccess) {
          return "cudaCreateSurfaceObject() failed for pair " + std::to_string(c)
            + " depth " + std::to_string(d) + " camera 1: " + std::string(cudaGetErrorString(res));
        }
        surfObjs.push_back(surfObj2);

        // Ensure that we have CPU and GPU buffers for the depth estimation.
        if (pd.m_CPURegionBuffer.size() != numRegions) {
          pd.m_CPURegionBuffer.resize(numRegions);
        }
        if (pd.m_GPURegionBuffer == nullptr) {
          res = cudaMalloc(&pd.m_GPURegionBuffer, numRegions * sizeof(float));
          if (res != cudaSuccess) {
            return "cudaMalloc() failed for pair " + std::to_string(c)
              + " depth " + std::to_string(d) + ": " + std::string(cudaGetErrorString(res));
          }
        }

        // Run the depth estimation kernels, which must handle portions of a region that are outside
        // of the projected area (they will be blue).  The grid size is the number of regions in X and Y
        // and the block size is the number of pixels per block in X and Y.  The image size is guaranteed
        // to be an even multiple of the block size.
        // Because the number of pixels in a block is larger than the maximum number of threads in a block,
        // each thread may handle pixels from multiple rows.  To minimize calculations in the kernel, the
        // number of iterations is a parameter to the kernel, as is the number of rows in an iteration and
        // the number of rows in each block.  The number of columns in the block is always the same as the X
        // block dimension.
        dim3 blockSize(cpi.m_pixelCounts[0] / m_nx, cpi.m_pixelCounts[1] / m_ny);
        dim3 gridSize(m_nx, m_ny);
        unsigned rowsPerIteration = blockSize.y;
        unsigned iterations = 1;
        unsigned rowsPerBlock = blockSize.y;
        if (blockSize.x * blockSize.y > 1024) {
          rowsPerIteration = 1024 / blockSize.x;
          iterations = blockSize.y / rowsPerIteration;
          if (blockSize.y > iterations * rowsPerIteration) {
            iterations++;
          }
        }
        blockSize.y = rowsPerIteration;
        CompareSurfacesKernel << <gridSize, blockSize, 0, *(pd.m_stream)>> > (
          surfObj1, surfObj2, pd.m_GPURegionBuffer,
          iterations, rowsPerIteration, rowsPerBlock);

        // Copy the results back to CPU memory.
        res = cudaMemcpyAsync(pd.m_CPURegionBuffer.data(), pd.m_GPURegionBuffer,
          numRegions * sizeof(float), cudaMemcpyDeviceToHost,
          *(pd.m_stream));
        if (res != cudaSuccess) {
          return "cudaMemcpy() failed for pair " + std::to_string(c)
            + " depth " + std::to_string(d) + ": " + std::string(cudaGetErrorString(res));
        }
      }

      // Wait for all the CUDA streams to complete.
      for (auto pd : cpi.m_perDepths) {
        cudaStreamSynchronize(*(pd.m_stream));
      }

      // Done with the surface objects.
      for (cudaSurfaceObject_t surfObj : surfObjs) {
        cudaDestroySurfaceObject(surfObj);
      }

      // Loop back through the depths, unmap the color buffers from CUDA.
      for (size_t d = 0; d < cpi.m_perDepths.size(); d++) {
        CameraPairInfo::PerDepth& pd = cpi.m_perDepths[d];
        for (size_t b = 0; b < 2; b++) {
          cudaError_t res = cudaGraphicsUnmapResources(1, &pd.m_cudaColorBuffers[b], *(pd.m_stream));
          if (res != cudaSuccess) {
            return "cudaGraphicsUnmapResources() failed for pair " + std::to_string(c)
              + " depth " + std::to_string(d) + " camera " + std::to_string(b) + ": "
              + std::string(cudaGetErrorString(res));
          }
        }
      }

      // Find the best depth value for each region.
      // Determine the best-matched and worse-matched scores at each location.
      std::vector<float> bestDepths(numRegions);
      std::vector<float> bestDepthValues(numRegions, 1e30);
      std::vector<float> worstDepthValues(numRegions, -1e30);
      for (CameraPairInfo::PerDepth& pd : cpi.m_perDepths) {
        float depth = pd.m_depth;
        for (size_t i = 0; i < pd.m_CPURegionBuffer.size(); i++) {
          float score = pd.m_CPURegionBuffer[i];
          if (score < bestDepthValues[i]) {
            bestDepths[i] = depth;
            bestDepthValues[i] = score;
          }
          if (score > worstDepthValues[i]) {
            worstDepthValues[i] = score;
          }
        }
      }

      // Determine the difference between the best-matched and worst-matched scores as a certainty/quality of fit measure.
      std::vector<float> qualityOfFit(numRegions, 0.0f);
      for (size_t i = 0; i < numRegions; i++) {
        // The largest possible average squared difference is 255^2 (65535), but the largest likely
        // value will be more like 200 or lower.  The minimum possible is 0, but with noise the minimum
        // likely would be more like 4.
        // The metric must be resilient to the best being 0 and to best and worst being the same.
        // It must also be be properly scaled compared to the distances between points (which are on
        // an integer lattice).
        qualityOfFit[i] = worstDepthValues[i] - bestDepthValues[i];
      }

      // We want to use the default if a region is not well fit.  Otherwise, we use the best depth.
      //std::cout << "XXX, Pair, X, Y, best, worst, selected" << std::endl;
      for (size_t i = 0; i < numRegions; i++) {
        if (qualityOfFit[i] < m_fitnessThreshold) {
          cpi.m_depths[i] = m_defaultDepth;
        } else {
          cpi.m_depths[i] = bestDepths[i];
        }
        //std::cout << "XXX, " << c << ", " << i % m_nx << ", " << i / m_nx << ", " << bestDepthValues[i] << ", " << worstDepthValues[i] << ", " << cpi.m_depths[i] << std::endl;
      }
    }

    return "";
  }

  DepthEstimator *m_parent;
  std::string m_constructorStatus;

  /// Camera pairs to use to estimate depth.
  std::vector< std::shared_ptr<CameraPairInfo> > m_cameraPairs;

  /// Number of points to create in the X direction.
  unsigned m_nx;

  /// Number of points to create in the Y direction.
  unsigned m_ny;

  /// Default depth to use if the depth cannot be estimated.
  float m_defaultDepth;

  /// Fitness threshold for determining if a region is well fit.
  float m_fitnessThreshold;

  /// Map from camera IDs to CUDA stream pointers.  Used to enable multiple camera depths to be computed in parallel.
  std::map<uint16_t, cudaStream_t*> m_cameraStreams;
};

DepthEstimator::DepthEstimator(std::vector< std::array<std::shared_ptr<CameraRenderInfo>, 2> > cameras,
  std::shared_ptr<asdp::render::RangeEstimator> rangeEstimator,
  std::shared_ptr<PoseAdjuster> poseAdjuster, Time cameraFrameInterval,
  unsigned nx, unsigned ny,
  std::vector<float> depths,
  float fitnessThreshold)
{
  // Check the parameters.
  if (cameras.size() == 0) {
    m_constructorStatus = "DepthEstimator::DepthEstimator(): cameras.size() must be greater than 0";
    return;
  }
  if (depths.size() == 0) {
    m_constructorStatus = "DepthEstimator::DepthEstimator(): depths.size() must be greater than 0";
    return;
  }
  if (nx == 0) {
    m_constructorStatus = "DepthEstimator::DepthEstimator(): nx must be greater than 0";
    return;
  }
  if (cameras[0][0]->m_resolutionPixels[0] / nx > MAX_BLOCK_SIZE) {
    m_constructorStatus = "DepthEstimator::DepthEstimator(): Block size must be less than or equal to "
      + std::to_string(MAX_BLOCK_SIZE) + " (add more regions)";
    return;
  }
  if (ny == 0) {
    m_constructorStatus = "DepthEstimator::DepthEstimator(): ny must be greater than 0";
    return;
  }
  if (cameras[0][0]->m_resolutionPixels[1] / ny > MAX_BLOCK_SIZE) {
    m_constructorStatus = "DepthEstimator::DepthEstimator(): Block size must be less than or equal to "
      + std::to_string(MAX_BLOCK_SIZE) + " (add more regions)";
    return;
  }

  // Create the implementation.
  m_impl = std::make_unique<DepthEstimatorImpl>(this, cameras, rangeEstimator,
    poseAdjuster, cameraFrameInterval, nx, ny, depths, fitnessThreshold);
  m_constructorStatus = m_impl->m_constructorStatus;
}

std::string DepthEstimator::ComputeDepthEstimate(Time time)
{
  if (!m_constructorStatus.empty()) {
    return "Constructor failed: " + m_constructorStatus;
  }

  return m_impl->ComputeDepthEstimate(time);
}

/// @brief Convert from degrees to radians.
static float radians(float deg)
{
  constexpr float kDegToRad = 3.14159265358979323846f / 180.0f;
  return deg * kDegToRad;
}

/// @brief Clamp value to lie between min and max range specified
static float clamp(float val, float minVal, float maxVal)
{
  return std::min(std::max(val, minVal), maxVal);
}

/// @brief CUDA-device-accessible function for computing the same value as intersectRayWithPlane() function.
static __host__ __device__ bool intersectRayWithPlane(const Vec3& rayStart, const Vec3& rayDir,
  const Vec3& planeStart, const Vec3& planeNormal, Vec3& intersectionPoint)
{
  float denom = rayDir.Dot(planeNormal);
  if (abs(denom) < 1e-8f) {
    // The ray is parallel to the plane
    return false;
  }

  float t = (planeStart - rayStart).Dot(planeNormal) / denom;
  if (t < 0) {
    // The intersection is behind the ray's start point
    return false;
  }

  intersectionPoint = rayStart + rayDir * t;
  return true;
}

/// @brief CUDA-device-accessible function for computing the same value as EstimateDepth() member function.
/// @param data The DepthEstimator's compacted data structure appropriate for the calling context (CPU or GPU).
static __device__ __host__ float estimateDepth(float *data, const Vec3& point, const Vec3& direction,
  float defaultDepth, unsigned int nx, unsigned int ny)
{
  // Compute values we'll need more than once.
  Vec3 rayDir = direction.Normalize();
  const Vec3& rayStart = point;

  // Find out which camera pair has the largest positive normalized dot product with the ray.
  // We rotate the +Y axis by the camera orientation to get the direction of the camera.
  float bestDot = -2;
  size_t bestPair = 0;
  Vec3 cameraDir = Vec3(0, 1, 0);
  for (size_t i = 0; i < CameraPairsKernelData::numCameraPairs(data); i++) {
    Vec3 cameraDirRot = CameraPairsKernelData::pairOrientations(data, i) * cameraDir;
    float dot = rayDir.Dot(cameraDirRot);
    if (dot > bestDot) {
      bestDot = dot;
      bestPair = i;
    }
  }

  // Make references to all of the data from m_impl.
  const Vec3& position = CameraPairsKernelData::pairPositions(data, bestPair);
  const Quat& orientation = CameraPairsKernelData::pairOrientations(data, bestPair);
  const float* fovsDeg = CameraPairsKernelData::pairFOVs(data, bestPair);
  const float* depths = CameraPairsKernelData::pairDepths(data, bestPair);

  // Find the coordinate of the ray's piercing point on the plane at the default depth.
  // The plane's orientation is the camera axis rotated by the camera orientation, and its
  // distance from the origin is the default depth.
  Vec3 pierce;
  Vec3 planeNormal = orientation * cameraDir;
  Vec3 originOnPlane = position + planeNormal * defaultDepth;
  if (!intersectRayWithPlane(rayStart, rayDir, originOnPlane, planeNormal, pierce)) {
    return defaultDepth;
  }

  // Determine the coordinates in the view frustum of the piercing point, clamping to the
  // range -1 to 1 in each dimension.  In helicopter space, the screen is in the XZ plane,
  // so screen Y will correspond to helicopter-space Z.
  float halfX = defaultDepth * tan(radians(fovsDeg[0] / 2));
  float halfY = defaultDepth * tan(radians(fovsDeg[1] / 2));
  Vec3 xDir = orientation * Vec3(1, 0, 0);
  Vec3 yDir = orientation * Vec3(0, 0, 1);
  float x = (pierce - originOnPlane).Dot(xDir) / halfX;
  float y = (pierce - originOnPlane).Dot(yDir) / halfY;
  x = clamp(x, -1.0f, 1.0f);
  y = clamp(y, -1.0f, 1.0f);

  // Convert the coordinates to the region index, which goes from 0 to 1 on each axis with the
  // -1 to 1 range covering half a pixel beyond the index point for each.  Clamp to the range 0 to 1 on each axis.
  // Interpolate the depth value based on the fractional piercing index.
  float xScaled = x * nx / (nx - 1.0f);
  float yScaled = y * ny / (ny - 1.0f);
  float xCoord = (xScaled + 1.0f) / 2.0f * (nx - 1.0f);
  xCoord = clamp(xCoord, 0.0f, nx - 1.0f);
  float yCoord = (yScaled + 1.0f) / 2.0 * (ny - 1.0f);
  yCoord = clamp(yCoord, 0.0, ny - 1.0f);

  // Look up the four values (floor and ceiling) around the point and use bilinear interpolation
  // to determine the depth at the point.
  size_t xFloor = size_t(floor(xCoord));
  size_t xCeil = size_t(ceil(xCoord));
  size_t yFloor = size_t(floor(yCoord));
  size_t yCeil = size_t(ceil(yCoord));
  float xFrac = xCoord - xFloor;
  float yFrac = yCoord - yFloor;
  float depthFF = depths[yFloor * nx + xFloor];
  float depthFC = depths[yFloor * nx + xCeil];
  float depthCF = depths[yCeil * nx + xFloor];
  float depthCC = depths[yCeil * nx + xCeil];
  float depth = (1 - xFrac) * (1 - yFrac) * depthFF + xFrac * (1 - yFrac) * depthFC +
    (1 - xFrac) * yFrac * depthCF + xFrac * yFrac * depthCC;

  // Estimate the contact point as that depth from the camera pair origin in the direction of the
  // piercing point, scaled by ratio of the found depth to the default depth to make it match the
  // one that would have been found for a plane at the found depth.
  Vec3 cameraToPierce = pierce - position;
  Vec3 contactPoint = position + cameraToPierce * (depth / defaultDepth);

  // Return the distance from the ray start to that point.
  return (contactPoint - rayStart).Length();
}

/// @brief Intersect a ray with a plane.
/// @param rayStart The start point of the ray.
/// @param rayDir The direction of the ray.
/// @param planeStart A point on the plane.
/// @param planeNormal The normal to the plane.
/// @param intersectionPoint The point of intersection.
/// @return True if the ray intersects the plane, false otherwise.
static bool intersectRayWithPlane(const glm::dvec3& rayStart, const glm::dvec3& rayDir,
  const glm::dvec3& planeStart, const glm::dvec3& planeNormal,
  glm::dvec3& intersectionPoint)
{
  float denom = glm::dot(rayDir, planeNormal);
  if (glm::epsilonEqual(denom, 0.0f, glm::epsilon<float>())) {
    // The ray is parallel to the plane
    return false;
  }

  double t = glm::dot(planeStart - rayStart, planeNormal) / denom;
  if (t < 0) {
    // The intersection is behind the ray's start point
    return false;
  }

  intersectionPoint = rayStart + t * rayDir;
  return true;
}

float DepthEstimator::EstimateDepth(const glm::vec3& point, const glm::vec3& direction) const
{
  // Make sure we have valid data.
  if (m_impl == nullptr || m_impl->m_cameraPairs.size() == 0) {
    return m_impl->m_defaultDepth;
  }

  // Compute values we'll need more than once.
  glm::dvec3 rayDir = glm::normalize(glm::dvec3(direction.x, direction.y, direction.z));
  glm::dvec3 rayStart = glm::dvec3(point.x, point.y, point.z);

  // Find out which camera pair has the largest positive normalized dot product with the ray.
  // We rotate the +Y axis by the camera orientation to get the direction of the camera.
  double bestDot = -2;
  size_t bestPair = 0;
  glm::dvec3 cameraDir = glm::dvec3(0, 1, 0);
  for (size_t i = 0; i < m_impl->m_cameraPairs.size(); i++) {
    CameraPairInfo const& cpi = *m_impl->m_cameraPairs[i];
    glm::dvec3 cameraDirRot = cpi.m_orientation * cameraDir;
    double dot = glm::dot(rayDir, cameraDirRot);
    if (dot > bestDot) {
      bestDot = dot;
      bestPair = i;
    }
  }

  // Make references to all of the data from m_impl.
  CameraPairInfo const& bestCameraPair = *m_impl->m_cameraPairs[bestPair];
  float defaultDepth = m_impl->m_defaultDepth;
  unsigned nx = m_impl->m_nx;
  unsigned ny = m_impl->m_ny;

  // Find the coordinate of the ray's piercing point on the plane at the default depth.
  // The plane's orientation is the camera axis rotated by the camera orientation, and its
  // distance from the origin is the default depth.
  glm::dvec3 pierce;
  glm::dvec3 planeNormal = bestCameraPair.m_orientation * cameraDir;
  glm::dvec3 originOnPlane = bestCameraPair.m_position + double(defaultDepth) * planeNormal;
  if (!intersectRayWithPlane(rayStart, rayDir, originOnPlane, planeNormal, pierce)) {
    return defaultDepth;
  }

  // Determine the coordinates in the view frustum of the piercing point, clamping to the
  // range -1 to 1 in each dimension.  In helicopter space, the screen is in the XZ plane,
  // so screen Y will correspond to helicopter-space Z.
  double halfX = defaultDepth * tan(glm::radians(bestCameraPair.m_fovsDeg[0] / 2));
  double halfY = defaultDepth * tan(glm::radians(bestCameraPair.m_fovsDeg[1] / 2));
  glm::dvec3 xDir = bestCameraPair.m_orientation * glm::dvec3(1, 0, 0);
  glm::dvec3 yDir = bestCameraPair.m_orientation * glm::dvec3(0, 0, 1);
  double x = glm::dot(pierce - originOnPlane, xDir) / halfX;
  double y = glm::dot(pierce - originOnPlane, yDir) / halfY;
  x = glm::clamp(x, -1.0, 1.0);
  y = glm::clamp(y, -1.0, 1.0);

  // Convert the coordinates to the region index, which goes from 0 to 1 on each axis with the
  // -1 to 1 range covering half a pixel beyond the index point for each.  Clamp to the range 0 to 1 on each axis.
  // Interpolate the depth value based on the fractional piercing index.
  double xScaled = x * nx / (nx - 1.0);
  double yScaled = y * ny / (ny - 1.0);
  double xCoord = (xScaled + 1.0) / 2.0 * (nx - 1.0);
  xCoord = glm::clamp(xCoord, 0.0, nx - 1.0);
  double yCoord = (yScaled + 1.0) / 2.0 * (ny - 1.0);
  yCoord = glm::clamp(yCoord, 0.0, ny - 1.0);

  // Look up the four values (floor and ceiling) around the point and use bilinear interpolation
  // to determine the depth at the point.
  size_t xFloor = size_t(floor(xCoord));
  size_t xCeil = size_t(ceil(xCoord));
  size_t yFloor = size_t(floor(yCoord));
  size_t yCeil = size_t(ceil(yCoord));
  double xFrac = xCoord - xFloor;
  double yFrac = yCoord - yFloor;
  double depthFF = bestCameraPair.m_depths[yFloor * nx + xFloor];
  double depthFC = bestCameraPair.m_depths[yFloor * nx + xCeil];
  double depthCF = bestCameraPair.m_depths[yCeil * nx + xFloor];
  double depthCC = bestCameraPair.m_depths[yCeil * nx + xCeil];
  double depth = (1 - xFrac) * (1 - yFrac) * depthFF + xFrac * (1 - yFrac) * depthFC +
    (1 - xFrac) * yFrac * depthCF + xFrac * yFrac * depthCC;

  // Estimate the contact point as that depth from the camera pair origin in the direction of the
  // piercing point, scaled by ratio of the found depth to the default depth to make it match the
  // one that would have been found for a plane at the found depth.
  glm::dvec3 cameraToPierce = pierce - bestCameraPair.m_position;
  glm::dvec3 contactPoint = bestCameraPair.m_position
    + (depth / defaultDepth) * cameraToPierce;

  // Return the distance from the ray start to that point.
  return glm::length(glm::vec3(contactPoint - rayStart));
}

void DepthEstimator::UpdateMeshesCPU(std::vector<std::shared_ptr<CameraRenderInfo>> cams)
{
  // Update the mesh depth values for each camera.
  for (std::shared_ptr<CameraRenderInfo> c : cams) {
    CameraRenderInfo& cam = *c;

    // Lock the mutex to protect the mesh data.
    std::lock_guard<std::mutex> lock(cam.m_meshMutex);

    // Record the camera position offset.
    glm::vec3 cameraPosition(cam.m_positionMeters[0], cam.m_positionMeters[1], cam.m_positionMeters[2]);

    // Look up the depth for each vertex in the mesh and update the depth in the mesh.
    // We add a small offset based on how far the mesh point is from the center of the camera
    // so that the visible triangle at a location is from the camera whose center of projection
    // is closest.
    const double offsetScale = 0.01;
    double xCenter = cam.m_mesh.nx / 2.0;
    double yCenter = cam.m_mesh.ny / 2.0;
    for (int y = 0; y < cam.m_mesh.ny; y++) {
      double yOff = y - yCenter;
      for (int x = 0; x < cam.m_mesh.nx; x++) {
        double xOff = x - xCenter;
        VertexInfo& v = cam.m_mesh.vertexInfo[y * cam.m_mesh.nx + x];
        double offsetFactor = 1.0 + offsetScale * sqrt(xOff * xOff + yOff * yOff);
        v.depth = EstimateDepth(cameraPosition, v.normalizedOffset) * offsetFactor;
      }
    }
  }
}

void DepthEstimator::UpdateMeshesGPU(std::vector<std::shared_ptr<CameraRenderInfo>> cams)
{
  // Ensure that we have an entry in m_cameraStreams for each camera.  If not, create one.
  for (auto c : cams) {
    uint16_t camID = c->m_ID;
    if (m_impl->m_cameraStreams.find(camID) == m_impl->m_cameraStreams.end()) {
      cudaStream_t* stream = new cudaStream_t;
      cudaError_t res = cudaStreamCreate(stream);
      if (res != cudaSuccess) {
        throw std::runtime_error("DepthEstimator::UpdateMeshesGPU(): cudaStreamCreate() failed for camera ID "
          + std::to_string(camID) + ": " + std::string(cudaGetErrorString(res)));
      }
      m_impl->m_cameraStreams[camID] = stream;
    }
  }

  // Make the CameraPairsKernelData structure for passing to the CUDA kernel.
  /// @todo This makes things incredibly slow. We need to find a way to avoid the managed malloc on each call.
  //CameraPairsKernelData kernelPairData(m_impl->m_cameraPairs);

  // Make a vector of shared_ptr to CameraDepthInfoKernelData to maintain for passing to the CUDA kernel.
  // These will be filled in as each camera is handled, overlapping computation and data transfer.
  std::vector< std::shared_ptr<CameraDepthInfoKernelData> > kernelCamerasData;

  // Update the mesh depth values for each camera.
  for (std::shared_ptr<CameraRenderInfo> c : cams) {
    CameraRenderInfo& cam = *c;

    // Lock the mutex to protect the mesh data.
    std::lock_guard<std::mutex> lock(cam.m_meshMutex);

    // Record the camera position offset.
    glm::vec3 cameraPosition(cam.m_positionMeters[0], cam.m_positionMeters[1], cam.m_positionMeters[2]);

    // Look up the depth for each vertex in the mesh and update the depth in the mesh.
    // We add a small offset based on how far the mesh point is from the center of the camera
    // so that the visible triangle at a location is from the camera whose center of projection
    // is closest.
    const double offsetScale = 0.01;
    double xCenter = cam.m_mesh.nx / 2.0;
    double yCenter = cam.m_mesh.ny / 2.0;
    for (int y = 0; y < cam.m_mesh.ny; y++) {
      double yOff = y - yCenter;
      for (int x = 0; x < cam.m_mesh.nx; x++) {
        double xOff = x - xCenter;
        VertexInfo& v = cam.m_mesh.vertexInfo[y * cam.m_mesh.nx + x];
        double offsetFactor = 1.0 + offsetScale * sqrt(xOff * xOff + yOff * yOff);
        v.depth = EstimateDepth(cameraPosition, v.normalizedOffset) * offsetFactor;
      }
    }
  }
}

//================================================================================================
// Testing and its helper functions and classes.

/// @brief Returns a test pattern value at the given X coordinate in the range 0..1.
/// @details This has variations at many frequency ranges so that it will handle a range
/// of depths.
/// @param x The X coordinate in arbitrary units.
/// @return The test pattern value at that X coordinate.
static double TestPattern(double x)
{
  const double periods[] = { 0.2, 0.7, 1.1, 2, 7, 11, 20, 70, 110 };
  const double phases[] = { 0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8 };
  const double magnitudes[] = { 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 1.0, 1.0, 1.0 };
  double result = 0, scale = 0;
  for (size_t i = 0; i < sizeof(periods) / sizeof(periods[0]); i++) {
    result += magnitudes[i] * (1.0 + sin(10*x / periods[i] + phases[i]));
    scale += 2 * magnitudes[i];
  }
  return result / scale;
}

static bool isBigEndian() {
  union {
    uint32_t i;
    char c[4];
  } testUnion = { 0x01020304 };

  return testUnion.c[0] == 1;
}

static void fixEndian(std::vector<uint16_t>& data) {
  if (!isBigEndian()) {
    for (uint16_t& value : data) {
      value = (value >> 8) | (value << 8);
    }
  }
}

void DepthEstimator::BuildGradientImages(DepthEstimator& de, uint16_t width, uint16_t height,
  uint16_t nx, uint16_t ny, float cameraFrameInterval, std::vector<float> testDepths)
{

  // Start with a grey-filled single-color image.  This will be the background.
  std::vector<uint16_t> blankImage(width * height, 32768);

  // Make a copy of the background image for the each camera and then fill its lower half with the test
  // pattern at different depths.

  for (size_t c = 0; c < 2; c++) {
    std::vector<uint16_t> im = blankImage;
    for (size_t y = 0; y < height / 2; y++) {
      // Flip the Y axis so the near ground is on the bottom.
      size_t yFlip = height - y - 1;

      // The depth changes for every height/ny pixels and it comes from the list of
      // depths.
      double depth = testDepths[(y / (height / ny)) % testDepths.size()];

      // Find the X piercing points for the depth at the left and right edges of the image
      // and then compute the scale and offset that will map the image pixels correctly.
      // Remember that the pixel centers are half a pixel in from the edges.
      double halfX = depth * tan(glm::radians(de.m_impl->m_cameraPairs[0]->m_cameras[c]->m_fovDegrees[0] / 2));
      double xLeft = -halfX * (double(width) / (width - 1));
      double xRight = halfX * (double(width) / (width - 1));
      double scale = (xRight - xLeft) / (width - 1);
      double offset = -xLeft + (0.5 * scale);

      // Add the camera's X center to the offset.
      offset += de.m_impl->m_cameraPairs[0]->m_cameras[c]->m_positionMeters[0];

      for (size_t x = 0; x < width; x++) {
        double value = 65535 * TestPattern(x * scale + offset);
        im[yFlip * width + x] = uint16_t(value);
      }
    }

#if 0
    // Write the image to a 16-bit PGM file for debugging, remembering to swap the endianness.
    std::ofstream file("DepthEstimatorTest" + std::to_string(c) + ".pgm", std::ios::binary);
    file << "P5\n" << width << " " << height << "\n65535\n";
    std::vector<uint16_t> fixedImage = im;
    fixEndian(fixedImage);
    file.write(reinterpret_cast<char*>(fixedImage.data()), fixedImage.size() * sizeof(uint16_t));
#endif

    // Construct an OpenGL texture and copy the image into it.
    GLuint texture;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R16, width, height, 0, GL_RED, GL_UNSIGNED_SHORT, im.data());
    glBindTexture(GL_TEXTURE_2D, 0);

    // Add three copies of the image to the image queue after constructing the ImageData object to hold it.
    std::shared_ptr<ImageData> id(new ImageData);
    id->texture = texture;
    id->exposure = cameraFrameInterval;
    for (size_t i = 0; i < 3; i++) {
      de.m_impl->m_cameraPairs[0]->m_cameras[c]->m_imageQueue->InsertImage(id);
    }
  }
}

float DepthEstimator::SpeedTestSingleEstimation(uint16_t width, uint16_t height, uint16_t nx, uint16_t ny)
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

  // Construct a DepthEstimator after making the objects required to construct it.
  std::vector< std::array<std::shared_ptr<CameraRenderInfo>, 2> > cameras;
  DistortionNone* dNone = new DistortionNone();
  std::shared_ptr<Distortion> distortion(dNone);
  VignetteNone* vNone = new VignetteNone();
  std::shared_ptr<Vignette> vignette(vNone);
  std::shared_ptr<ImageQueue> queue1(new ImageQueue);
  std::shared_ptr<ImageQueue> queue2(new ImageQueue);
  std::vector< std::shared_ptr<ImageQueue> > queues;
  queues.push_back(queue1);
  queues.push_back(queue2);
  std::shared_ptr<CameraRenderInfo> cam1(new CameraRenderInfo(1, { -1, 0, 0 }, { 0, 0, 0 }, { width, height }, { 90.0, 90.0 },
    distortion, vignette, queues[0], -1.0f));
  std::shared_ptr<CameraRenderInfo> cam2(new CameraRenderInfo(2, { 1, 0, 0 }, { 0, 0, 0 }, { width, height }, { 90.0, 90.0 },
    distortion, vignette, queues[1], -1.0f));
  cameras.push_back({ cam1, cam2 });
  std::shared_ptr<PoseAdjuster> poseAdjuster = std::make_shared<PoseAdjuster>();
  // Use the same value for the camera frame interval and the exposure time on the frames
  // so that we don't engage the time-varying brightness adjustment on the render system.
  float cameraFrameInterval = 1.0f;
  std::vector<float> testDepths = { 10, 20, 50, 100, 200, 500, 1000 };
  DepthEstimator de(cameras, nullptr, poseAdjuster, cameraFrameInterval, nx, ny, testDepths);
  if (de.m_constructorStatus != "") {
    return -1;
  }

  // Fill in test images for the cameras that will exercise the depth estimation.
  BuildGradientImages(de, width, height, nx, ny, cameraFrameInterval, testDepths);

  // Compute the depth estimate once to cause it to build all required structures.
  std::string res = de.ComputeDepthEstimate(0);
  if (res != "") {
    return -1;
  }

  // Run timing on a number of iterations and report the average.
  const size_t iterations = 100;
  std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
  for (size_t i = 0; i < iterations; i++) {
    res = de.ComputeDepthEstimate(0);
    if (res != "") {
      return -1;
    }
  }
  std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = end - start;
  return elapsed.count() / iterations;
}

__global__ void TestEstimateDepthKernel(float* depthInfo, Vec3 point, Vec3 direction, float defaultDepth,
  unsigned int nx, unsigned int ny, float *out)
{
  *out = estimateDepth(depthInfo, point, direction, defaultDepth, nx, ny);
}

std::string DepthEstimator::Test()
{
  // Test Vec3 and Quat classes
  {
    Vec3 v1(1.0, 2.0, 3.0);
    Vec3 v2(4.0, 5.0, 6.0);

    if (v1 != v1) {
      return "Vec3 != failed";
    }
    if (v2 == v1) {
      return "Vec3 == failed";
    }

    Vec3 v3 = v1 + v2;
    if (v3[0] != 5 || v3[1] != 7 || v3[2] != 9) {
      return "Vec3 addition failed";
    }

    Vec3 v4 = v2 - v1;
    if (v4[0] != 3 || v4[1] != 3 || v4[2] != 3) {
      return "Vec3 subtraction failed";
    }

    Vec3 v5 = v1 * 2;
    if (v5[0] != 2 || v5[1] != 4 || v5[2] != 6) {
      return "Vec3 scalar multiplication failed";
    }

    Vec3 v6 = v1.Normalize();
    double length = sqrt(v6[0] * v6[0] + v6[1] * v6[1] + v6[2] * v6[2]);
    if (fabs(length - 1.0) > 1e-6) {
      return "Vec3 normalization failed";
    }

    float dotProduct = v1.Dot(v2);
    if (fabs(dotProduct - 32) > 1e-6) {
      return "Vec3 dot product failed";
    }

    Quat q1(0.0, 0.0, 0.0, 1.0);
    Vec3 v7 = q1 * v1;
    if (fabs(v7[0] - v1[0]) > 1e-6 || fabs(v7[1] - v1[1]) > 1e-6 || fabs(v7[2] - v1[2]) > 1e-6) {
      return "Vec3 rotation failed";
    }

    // Try rotation by 90 degrees around Z axis
    float angle = glm::radians(90.0f);
    glm::quat qz = glm::angleAxis(angle, glm::vec3(0, 0, 1));
    Quat q2(qz);
    Vec3 v8 = q2 * v1;
    if (fabs(v8[0] + 2.0) > 1e-6 || fabs(v8[1] - 1.0) > 1e-6 || fabs(v8[2] - 3.0) > 1e-6) {
      return "Quat rotation failed";
    }
  }

  // Test the CameraPairsKernelData class
  {
    ToneMap toneMap;
    std::vector< std::shared_ptr<CameraPairInfo> > cameraPairs;
    std::shared_ptr<CameraRenderInfo> camera1, camera2;
    std::array<float, 2> fovs = { 40, 30 };
    std::array<unsigned, 2> resolution = { 80, 60 };
    std::shared_ptr<CameraPairInfo> pair1 = std::make_shared<CameraPairInfo>(toneMap, camera1, camera2,
      std::make_shared<RangeEstimatorFixed>(), glm::dvec3( 1, 2, 3 ), glm::angleAxis(0.0, glm::dvec3(0, 0, 1)),
      fovs, resolution,
      std::make_shared<PoseAdjuster>(2000, HELICOPTER, false), 1/60.0f, std::vector<float>(), 10.0);
    std::shared_ptr<CameraPairInfo> pair2 = std::make_shared<CameraPairInfo>(toneMap, camera1, camera2,
      std::make_shared<RangeEstimatorFixed>(), glm::dvec3(4, 5, 6), glm::angleAxis(0.0, glm::dvec3(0, 0, 1)),
      fovs, resolution,
      std::make_shared<PoseAdjuster>(2000, HELICOPTER, false), 1 / 60.0f, std::vector<float>(), 20.0);
    cameraPairs.push_back(pair1);
    cameraPairs.push_back(pair2);
    CameraPairsKernelData kd(cameraPairs);
    kd.CopyDataToGPU();

    if (kd.numCameraPairsCPU() != 2) {
      return "CameraPairsKernelData initialization failed";
    }

    for (size_t i = 0; i < cameraPairs.size(); i++) {
      if (kd.pairPositionsCPU(i)[0] != (i * 3 + 1) || kd.pairPositionsCPU(i)[1] != (i * 3 + 2) || kd.pairPositionsCPU(i)[2] != (i * 3 + 3)) {
        return "CameraPairsKernelData pairPositions " + std::to_string(i) + " failed";
      }
      if (kd.pairOrientationsCPU(i)[0] != 0 || kd.pairOrientationsCPU(i)[1] != 0 ||
        kd.pairOrientationsCPU(i)[2] != 0 || kd.pairOrientationsCPU(i)[3] != 1) {
        return "CameraPairsKernelData pairOrientations " + std::to_string(i) + " failed";
      }
      if (kd.pairFOVsCPU(i)[0] != fovs[0] || kd.pairFOVsCPU(i)[1] != fovs[1]) {
        return "CameraPairsKernelData pairFOVs " + std::to_string(i) + " failed";
      }
      if (kd.pairPixelCountsCPU(i)[0] != resolution[0] || kd.pairPixelCountsCPU(i)[1] != resolution[1]) {
        return "CameraPairsKernelData pairPixelCounts " + std::to_string(i) + " failed";
      }
    }

    // All depth entries in the first pair should be 10 and all in the second pair should be 20.
    size_t numDepths = resolution[0] * resolution[1];
    for (size_t d = 0; d < numDepths; d++) {
      float d0 = kd.pairDepthsCPU(0)[d];
      float d1 = kd.pairDepthsCPU(1)[d];
      if (d0 != 10 || d1 != 20) {
        return "CameraPairsKernelData pairDepths information incorrect: got "
          + std::to_string(d0) + ", " + std::to_string(d1) + " at element " + std::to_string(d);
      }
    }

    // Copy the GPU data back to the CPU buffer and verify that it matches.
    cudaMemcpy(kd.kData, kd.data, sizeof(float) * kd.totalSizeFloats, cudaMemcpyDeviceToHost);
    for (size_t d = 0; d < numDepths; d++) {
      float d0 = kd.pairDepthsCPU(0)[d];
      float d1 = kd.pairDepthsCPU(1)[d];
      if (d0 != 10 || d1 != 20) {
        return "CameraPairsKernelData pairDepths copied information incorrect: got "
          + std::to_string(d0) + ", " + std::to_string(d1) + " at element " + std::to_string(d);
      }
    }
  }

  // Test the CameraDepthInfoKernelData class
  {
    std::array<double, 3> positionMeters = { 1.0, 2.0, 3.0 };
    std::array<double, 3> orientationDegrees = { 10.0, 20.0, 30.0 };
    std::array<uint16_t, 2> resolution = { 1280, 1024 };
    std::array<double, 2> fovs = { 40, 30 };
    std::shared_ptr<Distortion> distortion = std::make_shared<DistortionNone>();
    std::shared_ptr<Vignette> vignette = std::make_shared<VignetteNone>();
    std::shared_ptr<asdp::render::ImageQueue> imageQueue;
    std::shared_ptr<CameraRenderInfo> camera = std::make_shared<CameraRenderInfo>(1, positionMeters, orientationDegrees,
      resolution, fovs, distortion, vignette, imageQueue, 0.0f);

    // Fill in the mesh information with 900 for all elements.
    camera->ComputePlanarCameraMeshInfo(30, 20, 900.0f);

    CameraDepthInfoKernelData kd(camera);

    // Verify that all count and depth values match in the copied structure.
    if (camera->m_mesh.nx != kd.cameraNx() || camera->m_mesh.ny != kd.cameraNy()) {
      return "CameraDepthInfoKernelData count mismatch for camera 0";
    }
    float* depths = kd.cameraDepths();
    size_t count = camera->m_mesh.ny * camera->m_mesh.nx;

    // Make sure the depth values match.
    for (size_t d = 0; d < count; d++) {
      if (camera->m_mesh.vertexInfo[d].depth != depths[d]) {
        return "CameraDepthInfoKernelData data mismatch for camera 0";
      }
    }

    // Modify the depth values then write them back.
    for (size_t d = 0; d < count; d++) {
      depths[d] = 1.0f;
    }
    kd.FillDepthsBackToCameraRenderInfos();

    // Make sure the depth values match.
    for (size_t d = 0; d < count; d++) {
      if (camera->m_mesh.vertexInfo[d].depth != depths[d]) {
        return "CameraDepthInfoKernelData data mismatch after modification";
      }
    }
  }

  // Test intersectRayWithPlane()
  {
    glm::dvec3 rayStart(0, 0, 0);
    glm::dvec3 rayDir(0, 0, 1);
    glm::dvec3 planeStart(0, 0, 1);
    glm::dvec3 planeNormal(0, 0, 1);
    glm::dvec3 intersectionPoint;

    if (!intersectRayWithPlane(rayStart, rayDir, planeStart, planeNormal, intersectionPoint)) {
      return "intersectRayWithPlane() failed for perpendicular ray and plane";
    }
    if (intersectionPoint != glm::dvec3(0, 0, 1)) {
      return "intersectRayWithPlane() location failed for perpendicular ray and plane";
    }

    rayDir = glm::dvec3(0, 0, -1);
    if (intersectRayWithPlane(rayStart, rayDir, planeStart, planeNormal, intersectionPoint)) {
      return "intersectRayWithPlane() should not have succeeded for anti-perpendicular ray and plane";
    }

    rayDir = glm::dvec3(1, 0, 0);
    if (intersectRayWithPlane(rayStart, rayDir, planeStart, planeNormal, intersectionPoint)) {
      return "intersectRayWithPlane() should not have succeeded for parallel ray and plane";
    }

    rayDir = glm::dvec3(1, 0, 1);
    if (!intersectRayWithPlane(rayStart, rayDir, planeStart, planeNormal, intersectionPoint)) {
      return "intersectRayWithPlane() failed for skew ray and plane";
    }
    if (!glm::epsilonEqual(glm::distance(intersectionPoint, glm::dvec3(1, 0, 1)), 0.0, glm::epsilon<double>())) {
      return "intersectRayWithPlane() failed for skew ray and plane";
    }

    Vec3 vRayStart(0, 0, 0);
    Vec3 vRayDir(0, 0, 1);
    Vec3 vPlaneStart(0, 0, 1);
    Vec3 vPlaneNormal(0, 0, 1);
    Vec3 vIntersectionPoint;

    if (!intersectRayWithPlane(vRayStart, vRayDir, vPlaneStart, vPlaneNormal, vIntersectionPoint)) {
      return "intersectRayWithPlane() Vec3 failed for perpendicular ray and plane";
    }
    if (vIntersectionPoint != Vec3(0, 0, 1)) {
      return "intersectRayWithPlane() Vec3 location failed for perpendicular ray and plane";
    }

    vRayDir = glm::dvec3(0, 0, -1);
    if (intersectRayWithPlane(vRayStart, vRayDir, vPlaneStart, vPlaneNormal, vIntersectionPoint)) {
      return "intersectRayWithPlane() Vec3should not have succeeded for anti-perpendicular ray and plane";
    }

    vRayDir = glm::dvec3(1, 0, 0);
    if (intersectRayWithPlane(vRayStart, vRayDir, vPlaneStart, vPlaneNormal, vIntersectionPoint)) {
      return "intersectRayWithPlane() Vec3 should not have succeeded for parallel ray and plane";
    }

    vRayDir = glm::dvec3(1, 0, 1);
    if (!intersectRayWithPlane(vRayStart, vRayDir, vPlaneStart, vPlaneNormal, vIntersectionPoint)) {
      return "intersectRayWithPlane() Vec3 failed for skew ray and plane";
    }
    if (abs((vIntersectionPoint - Vec3(1, 0, 1)).Length()) > 1e-8f) {
      return "intersectRayWithPlane() Vec3 failed for skew ray and plane";
    }
  }

  /// Test the DepthEstimator class.
  {
    // Create a window and OpenGL context.
    if (!glfwInit()) {
      return "Failed to initialize GLFW";
    }
    glfwWindowHint(GLFW_VISIBLE, false);
    std::shared_ptr<GLFWwindow> window(glfwCreateWindow(640, 480, "DepthEstimator Test", NULL, NULL), glfwDestroyWindow);
    if (!window) {
      return "Failed to create GLFW window";
    }
    glfwMakeContextCurrent(window.get());

    // Initialize GLEW in our context. It is okay to initialize it more than once.
    glewExperimental = true;
    if (glewInit() != GLEW_OK) {
      return "Failed to initialize GLEW";
    }
    // Clear any GL error that Glew caused.  Apparently on Non-Windows
    // platforms, this can cause a spurious error 1280.
    glGetError();

    // Put into a block so that we destroy things in here before we destroy the context.
    {
      uint16_t nx = 12;   ///< Number of points to create in the X direction.  Must be divisible by 4 for our tests below.
      uint16_t ny = 12;

      uint16_t width = 50*nx;  ///< Width of the frame buffer.
      uint16_t height = 50*ny; ///< Height of the frame buffer.  Must be divisible by ny for our tests below.

      // Construct a DepthEstimator after making the objects required to construct it.
      std::vector< std::array<std::shared_ptr<CameraRenderInfo>, 2> > cameras;
      DistortionNone* dNone = new DistortionNone();
      std::shared_ptr<Distortion> distortion(dNone);
      VignetteNone* vNone = new VignetteNone();
      std::shared_ptr<Vignette> vignette(vNone);
      std::shared_ptr<ImageQueue> queue1(new ImageQueue);
      std::shared_ptr<ImageQueue> queue2(new ImageQueue);
      std::vector< std::shared_ptr<ImageQueue> > queues;
      queues.push_back(queue1);
      queues.push_back(queue2);
      std::shared_ptr<CameraRenderInfo> cam1(new CameraRenderInfo(1, { -1, 0, 0 }, { 0, 0, 0 }, { width, height}, { 90.0, 90.0 },
        distortion, vignette, queues[0], -1.0f));
      std::shared_ptr<CameraRenderInfo> cam2(new CameraRenderInfo(2, { 1, 0, 0 }, { 0, 0, 0 }, { width, height}, { 90.0, 90.0 },
        distortion, vignette, queues[1], -1.0f));
      cameras.push_back({ cam1, cam2 });

      std::shared_ptr<PoseAdjuster> poseAdjuster = std::make_shared<PoseAdjuster>();
      // Use the same value for the camera frame interval and the exposure time on the frames
      // so that we don't engage the time-varying brightness adjustment on the render system.
      float cameraFrameInterval = 1.0f;
      std::vector<float> testDepths = { 10, 20, 50, 100, 200, 500, 1000 };
      DepthEstimator de(cameras, nullptr, poseAdjuster, cameraFrameInterval, nx, ny, testDepths);
      if (de.m_constructorStatus != "") {
        return "DepthEstimator constructor failed: " + de.m_constructorStatus;
      }

      //================================================================================================
      // Fill in the default depth for all regions and test EstimateDepth() on the result to make sure
      // our geometric transformations work properly.
      float defaultDepth = de.m_impl->m_defaultDepth;
      float depth = defaultDepth;
      de.m_impl->m_cameraPairs[0]->m_depths.clear();
      de.m_impl->m_cameraPairs[0]->m_depths.resize(nx * ny, depth);

      // Test the depth at the origin, shooting along the +Y axis towards the plane.
      float estimatedDepth = de.EstimateDepth(glm::vec3(0, 0, 0), glm::vec3(0, 1, 0));
      if (fabs(estimatedDepth - depth) > depth * 1e-6) {
        return "EstimateDepth() default-depth failed for origin";
      }

      // Test shooting at a slight angle to the +Y axis towards the plane.  The depth should scale
      // with the length of the long edge of the triangle.
      float dx = 0.1;
      estimatedDepth = de.EstimateDepth(glm::vec3(0, 0, 0), glm::vec3(dx, 1, 0));
      float expectedDepth = sqrt(1 + dx * dx) * depth;
      if (fabs(estimatedDepth - expectedDepth) > expectedDepth * 1e-6) {
        return "EstimateDepth() default-depth failed for slight angle";
      }

      // Test shooting at a slight angle in Z to the +Y axis towards the plane.  The depth should scale
      // with the length of the long edge of the triangle.
      float dy = 0.2;
      estimatedDepth = de.EstimateDepth(glm::vec3(0, 0, 0), glm::vec3(0, 1, dy));
      expectedDepth = sqrt(1 + dy * dy) * depth;
      if (fabs(estimatedDepth - expectedDepth) > expectedDepth * 1e-6) {
        return "EstimateDepth() default-depth failed for slight Y angle";
      }

      // Test shooting at an angle that is beyond the edge.  It should use the same depth as the
      // value at the edge.
      dx = 2.0;
      estimatedDepth = de.EstimateDepth(glm::vec3(0, 0, 0), glm::vec3(dx, 1, 0));
      expectedDepth = sqrt(1 + dx * dx) * depth;
      if (fabs(estimatedDepth - expectedDepth) > expectedDepth * 1e-6) {
        return "EstimateDepth() default-depth failed for edge";
      }

      //================================================================================================
      // Repeat the above tests using the static estimateDepth() function
      {
        CameraPairsKernelData kd(de.m_impl->m_cameraPairs);
        kd.CopyDataToGPU();

        // Test the depth at the origin, shooting along the +Y axis towards the plane.
        estimatedDepth = estimateDepth(kd.data, glm::vec3(0, 0, 0), glm::vec3(0, 1, 0), defaultDepth,
          de.m_impl->m_nx, de.m_impl->m_ny);
        if (fabs(estimatedDepth - depth) > depth * 1e-6) {
          return "estimateDepth() default-depth failed for origin";
        }

        // Test shooting at a slight angle to the +Y axis towards the plane.  The depth should scale
        // with the length of the long edge of the triangle.
        float dx = 0.1;
        estimatedDepth = estimateDepth(kd.data, glm::vec3(0, 0, 0), glm::vec3(dx, 1, 0), defaultDepth,
          de.m_impl->m_nx, de.m_impl->m_ny);
        expectedDepth = sqrt(1 + dx * dx) * depth;
        if (fabs(estimatedDepth - expectedDepth) > expectedDepth * 1e-6) {
          return "estimateDepth() default-depth failed for slight angle";
        }

        // Test shooting at a slight angle in Z to the +Y axis towards the plane.  The depth should scale
        // with the length of the long edge of the triangle.
        float dy = 0.2;
        estimatedDepth = estimateDepth(kd.data, glm::vec3(0, 0, 0), glm::vec3(0, 1, dy), defaultDepth,
          de.m_impl->m_nx, de.m_impl->m_ny);
        expectedDepth = sqrt(1 + dy * dy) * depth;
        if (fabs(estimatedDepth - expectedDepth) > expectedDepth * 1e-6) {
          return "estimateDepth() default-depth failed for slight Y angle";
        }

        // Test shooting at an angle that is beyond the edge.  It should use the same depth as the
        // value at the edge.
        dx = 2.0;
        estimatedDepth = estimateDepth(kd.data, glm::vec3(0, 0, 0), glm::vec3(dx, 1, 0), defaultDepth,
          de.m_impl->m_nx, de.m_impl->m_ny);
        expectedDepth = sqrt(1 + dx * dx) * depth;
        if (fabs(estimatedDepth - expectedDepth) > expectedDepth * 1e-6) {
          return "estimateDepth() default-depth failed for edge";
        }
      }

      //================================================================================================
      // Fill in half of the default depth for all regions and test EstimateDepth() again.
      depth /= 2;
      de.m_impl->m_cameraPairs[0]->m_depths.clear();
      de.m_impl->m_cameraPairs[0]->m_depths.resize(nx * ny, depth);

      // Test the depth at the origin, shooting along the - axis away from the plane.
      // This should return the default depth because there is not intersection.
      estimatedDepth = de.EstimateDepth(glm::vec3(0, 0, 0), glm::vec3(0, -1,0));
      if (fabs(estimatedDepth - defaultDepth) > defaultDepth * 1e-6) {
        return "EstimateDepth() half-default-depth failed away from origin";
      }

      // Test the depth at the origin, shooting along the +Y axis towards the plane.
      estimatedDepth = de.EstimateDepth(glm::vec3(0, 0, 0), glm::vec3(0, 1, 0));
      if (fabs(estimatedDepth - depth) > depth * 1e-6) {
        return "EstimateDepth() half-default-depth failed for origin";
      }

      // Test shooting at a slight angle in X to the +Y axis towards the plane.  The depth should scale
      // with the length of the long edge of the triangle.
      dx = 0.1;
      estimatedDepth = de.EstimateDepth(glm::vec3(0, 0, 0), glm::vec3(dx, 1, 0));
      expectedDepth = sqrt(1 + dx * dx) * depth;
      if (fabs(estimatedDepth - expectedDepth) > expectedDepth * 1e-6) {
        return "EstimateDepth() half-default-depth failed for slight X angle";
      }

      // Test shooting at an angle that is beyond the edge.  It should use the same depth as the
      // value at the edge
      dx = 2.0;
      estimatedDepth = de.EstimateDepth(glm::vec3(0, 0, 0), glm::vec3(dx, 1, 0));
      expectedDepth = sqrt(1 + dx * dx) * depth;
      if (fabs(estimatedDepth - expectedDepth) > expectedDepth * 1e-6) {
        return "EstimateDepth() half-default-depth failed for edge";
      }

      //================================================================================================
      // Repeat the above tests using the static estimateDepth() function
      {
        CameraPairsKernelData kd(de.m_impl->m_cameraPairs);
        kd.CopyDataToGPU();

        // Test the depth at the origin, shooting along the - axis away from the plane.
        // This should return the default depth because there is not intersection.
        estimatedDepth = estimateDepth(kd.data, glm::vec3(0, 0, 0), glm::vec3(0, -1, 0), defaultDepth,
          de.m_impl->m_nx, de.m_impl->m_ny);
        if (fabs(estimatedDepth - defaultDepth) > defaultDepth * 1e-6) {
          return "estimateDepth() half-default-depth failed away from origin";
        }

        // Test the depth at the origin, shooting along the +Y axis towards the plane.
        estimatedDepth = estimateDepth(kd.data,glm::vec3(0, 0, 0), glm::vec3(0, 1, 0), defaultDepth,
          de.m_impl->m_nx, de.m_impl->m_ny);
        if (fabs(estimatedDepth - depth) > depth * 1e-6) {
          return "estimateDepth() half-default-depth failed for origin";
        }

        // Test shooting at a slight angle in X to the +Y axis towards the plane.  The depth should scale
        // with the length of the long edge of the triangle.
        dx = 0.1;
        estimatedDepth = estimateDepth(kd.data, glm::vec3(0, 0, 0), glm::vec3(dx, 1, 0), defaultDepth,
          de.m_impl->m_nx, de.m_impl->m_ny);
        expectedDepth = sqrt(1 + dx * dx) * depth;
        if (fabs(estimatedDepth - expectedDepth) > expectedDepth * 1e-6) {
          return "estimateDepth() half-default-depth failed for slight X angle";
        }

        // Test shooting at an angle that is beyond the edge.  It should use the same depth as the
        // value at the edge
        dx = 2.0;
        estimatedDepth = estimateDepth(kd.data, glm::vec3(0, 0, 0), glm::vec3(dx, 1, 0), defaultDepth,
          de.m_impl->m_nx, de.m_impl->m_ny);
        expectedDepth = sqrt(1 + dx * dx) * depth;
        if (fabs(estimatedDepth - expectedDepth) > expectedDepth * 1e-6) {
          return "estimateDepth() half-default-depth failed for edge";
        }
      }

      //================================================================================================
      // Fill in different depths for different regions and test EstimateDepth() again.  Make the center
      // further away so that any collision method will interpolate between the two depths.
      // Make the Y difference go all the way to the bottom so we can ensure that the polarity is correct.
      double centerDepth = depth * 1.5;
      for (size_t x = nx/4; x < 3*nx/4; x++) {
        for (size_t y = 0; y < 3*ny/4; y++) {
          de.m_impl->m_cameraPairs[0]->m_depths[y * nx + x] = centerDepth;
        }
      }

      // Test the depth at the origin, shooting along the +Y axis towards the plane.
      estimatedDepth = de.EstimateDepth(glm::vec3(0, 0, 0), glm::vec3(0, 1, 0));
      if (fabs(estimatedDepth - centerDepth) > centerDepth * 1e-6) {
        return "EstimateDepth() varying depth failed for origin";
      }

      // Test shooting at a wide angle in X to the +Y axis towards the plane.  The depth should scale
      // with the length of the long edge of the triangle.
      dx = 2.0;
      estimatedDepth = de.EstimateDepth(glm::vec3(0, 0, 0), glm::vec3(dx, 1, 0));
      expectedDepth = sqrt(1 + dx * dx) * depth;
      if (fabs(estimatedDepth - expectedDepth) > expectedDepth * 1e-6) {
        return "EstimateDepth() varying depth failed for slight X angle";
      }

      // Test shooting between two points and make sure that the answer is between the two depths.
      // We aim for slightly over halfway to the edge, which should be between the center and
      // outer points.
      dx = tan(glm::radians(de.m_impl->m_cameraPairs[0]->m_fovsDeg[0]) / 2.0) / 2.0 + 0.01;
      double below = sqrt(1 + dx * dx) * depth;
      double above = sqrt(1 + dx * dx) * centerDepth;
      estimatedDepth = de.EstimateDepth(glm::vec3(0, 0, 0), glm::vec3(dx, 1, 0));
      if (estimatedDepth <= below || estimatedDepth >= above) {
        return "EstimateDepth() varying depth failed for interpolating between two points";
      }

      // Test shooting between two points in +Y screen (+Z world) and make sure that the answer
      // is between the two depths.
      // We aim for slightly over halfway to the edge, which should be between the center and
      // outer points.
      double dz = tan(glm::radians(de.m_impl->m_cameraPairs[0]->m_fovsDeg[0]) / 2.0) / 2.0 + 0.01;
      below = sqrt(1 + dz * dz) * depth;
      above = sqrt(1 + dz * dz) * centerDepth;
      estimatedDepth = de.EstimateDepth(glm::vec3(0, 0, 0), glm::vec3(0, 1, dz));
      if (estimatedDepth <= below || estimatedDepth >= above) {
        return "EstimateDepth() varying depth failed for interpolating between two points in +Y";
      }

      // Test in -Y screen (-Z world) and make sure that the answer is centerDepth.
      dz = -dz;
      expectedDepth = sqrt(1 + dz * dz) * centerDepth;
      estimatedDepth = de.EstimateDepth(glm::vec3(0, 0, 0), glm::vec3(0, 1, dz));
      if (fabs(estimatedDepth - expectedDepth) > expectedDepth * 1e-6) {
        return "EstimateDepth() varying depth failed for interpolating between two points in -Y";
      }

      //================================================================================================
      // Repeat the above tests using the static estimateDepth() function
      {
        CameraPairsKernelData kd(de.m_impl->m_cameraPairs);
        kd.CopyDataToGPU();

        // Test the depth at the origin, shooting along the +Y axis towards the plane.
        estimatedDepth = estimateDepth(kd.data, glm::vec3(0, 0, 0), glm::vec3(0, 1, 0), defaultDepth,
          de.m_impl->m_nx, de.m_impl->m_ny);
        if (fabs(estimatedDepth - centerDepth) > centerDepth * 1e-6) {
          return "estimateDepth() varying depth failed for origin";
        }

        // Test shooting at a wide angle in X to the +Y axis towards the plane.  The depth should scale
        // with the length of the long edge of the triangle.
        dx = 2.0;
        estimatedDepth = estimateDepth(kd.data, glm::vec3(0, 0, 0), glm::vec3(dx, 1, 0), defaultDepth,
          de.m_impl->m_nx, de.m_impl->m_ny);
        expectedDepth = sqrt(1 + dx * dx) * depth;
        if (fabs(estimatedDepth - expectedDepth) > expectedDepth * 1e-6) {
          return "estimateDepth() varying depth failed for slight X angle";
        }

        // Test shooting between two points and make sure that the answer is between the two depths.
        // We aim for slightly over halfway to the edge, which should be between the center and
        // outer points.
        dx = tan(glm::radians(de.m_impl->m_cameraPairs[0]->m_fovsDeg[0]) / 2.0) / 2.0 + 0.01;
        double below = sqrt(1 + dx * dx) * depth;
        double above = sqrt(1 + dx * dx) * centerDepth;
        estimatedDepth = estimateDepth(kd.data, glm::vec3(0, 0, 0), glm::vec3(dx, 1, 0), defaultDepth,
          de.m_impl->m_nx, de.m_impl->m_ny);
        if (estimatedDepth <= below || estimatedDepth >= above) {
          return "estimateDepth() varying depth failed for interpolating between two points";
        }

        // Test shooting between two points in +Y screen (+Z world) and make sure that the answer
        // is between the two depths.
        // We aim for slightly over halfway to the edge, which should be between the center and
        // outer points.
        double dz = tan(glm::radians(de.m_impl->m_cameraPairs[0]->m_fovsDeg[0]) / 2.0) / 2.0 + 0.01;
        below = sqrt(1 + dz * dz) * depth;
        above = sqrt(1 + dz * dz) * centerDepth;
        estimatedDepth = estimateDepth(kd.data, glm::vec3(0, 0, 0), glm::vec3(0, 1, dz), defaultDepth,
          de.m_impl->m_nx, de.m_impl->m_ny);
        if (estimatedDepth <= below || estimatedDepth >= above) {
          return "estimateDepth() varying depth failed for interpolating between two points in +Y";
        }

        // Test in -Y screen (-Z world) and make sure that the answer is centerDepth.
        dz = -dz;
        expectedDepth = sqrt(1 + dz * dz) * centerDepth;
        estimatedDepth = estimateDepth(kd.data, glm::vec3(0, 0, 0), glm::vec3(0, 1, dz), defaultDepth,
          de.m_impl->m_nx, de.m_impl->m_ny);
        if (fabs(estimatedDepth - expectedDepth) > expectedDepth * 1e-6) {
          return "estimateDepth() varying depth failed for interpolating between two points in -Y";
        }
      }

      //================================================================================================
      // Test the estimateDepth() function running on the GPU, called from a kernel run.
      {
        CameraPairsKernelData kd(de.m_impl->m_cameraPairs);
        kd.CopyDataToGPU();

        // Allocate GPU memory for the result.
        float* d_estimatedDepth;
        cudaMalloc(&d_estimatedDepth, sizeof(float));
        cudaMemset(d_estimatedDepth, 0, sizeof(float));

        // Launch a kernel to call estimateDepth().
        // Test the depth at the origin, shooting along the +Y axis towards the plane.
        dim3 blockSize(1, 1, 1);
        dim3 gridSize(1, 1, 1);
        TestEstimateDepthKernel << <gridSize, blockSize >> > (kd.kData, glm::vec3(0, 0, 0), glm::vec3(0, 1, 0),
          defaultDepth, de.m_impl->m_nx, de.m_impl->m_ny, d_estimatedDepth);

        // Copy the result back to the CPU.
        float estimatedDepth;
        cudaMemcpy(&estimatedDepth, d_estimatedDepth, sizeof(float), cudaMemcpyDeviceToHost);
        cudaFree(d_estimatedDepth);

        // Test the value.
        if (abs(estimatedDepth - defaultDepth) > 1e-6f) {
          return "estimateDepth() GPU test failed for origin";
        }
      }

      //================================================================================================
      // Produce a set of images with known depths for each camera and test ComputeDepthEstimate() on them.
      // We analytically render the images with different depths for different image regions in Y so that
      // each of the ny samples is completely at the same depth.  We use a sum of sinusoids at relatively
      // prime frequencies with different phases to make the image have contrast and a specific alignment.
      // We move this pattern to different Z depths for each region in the Y camera axis.
      BuildGradientImages(de, width, height, nx, ny, cameraFrameInterval, testDepths);

      // Compute the depth estimate.
      std::string res = de.ComputeDepthEstimate(0);
      if (res != "") {
        return "ComputeDepthEstimate() failed: " + res;
      }

      // Check the depth estimates directly.
      for (size_t y = 0; y < ny; y++) {
        double depth = testDepths[y % testDepths.size()];
        if (y >= ny / 2) {
          // In the top half of the image, there are no features, so it should be the default depth.
          depth = testDepths.back();
        }
        for (size_t x = 0; x < nx; x++) {
          size_t i = y * nx + x;
          if (fabs(de.m_impl->m_cameraPairs[0]->m_depths[i] - depth) > depth * 1e-6) {
            return "ComputeDepthEstimate() failed for region " + std::to_string(i)
              + ", found " + std::to_string(de.m_impl->m_cameraPairs[0]->m_depths[i]) + " expected " + std::to_string(depth);
          }
        }
      }

      // Check the depth estimates using probe rays, ensuring different results for different camera pairs.
      // We make two camera pairs with different default depths, one pair pointed 45 degrees to the right and
      // the other 45 degrees to the left.  We then shoot rays from the origin to the center of the image and
      // make sure that the depths are correct.
      // We don't actually do depth estimation, so we just re-use the same (ignored) image data.
      std::vector< std::array<std::shared_ptr<CameraRenderInfo>, 2> > probeCameras;
      std::shared_ptr<CameraRenderInfo> cam1Probe(new CameraRenderInfo(1, { -1, 1, 0 }, { 0, 0, -45 }, { width, height }, { 90.0, 90.0 },
        distortion, vignette, queues[0], -1.0f));
      std::shared_ptr<CameraRenderInfo> cam2Probe(new CameraRenderInfo(2, { 1, -1, 0 }, { 0, 0, -45 }, { width, height }, { 90.0, 90.0 },
        distortion, vignette, queues[1], -1.0f));
      probeCameras.push_back({ cam1Probe, cam2Probe });
      std::shared_ptr<CameraRenderInfo> cam3Probe(new CameraRenderInfo(1, { -1, -1, 0 }, { 0, 0, 45 }, { width, height }, { 90.0, 90.0 },
        distortion, vignette, queues[0], -1.0f));
      std::shared_ptr<CameraRenderInfo> cam4Probe(new CameraRenderInfo(2, { 1, 1, 0 }, { 0, 0, 45 }, { width, height }, { 90.0, 90.0 },
        distortion, vignette, queues[1], -1.0f));
      probeCameras.push_back({ cam3Probe, cam4Probe });
      DepthEstimator deProbe(probeCameras, nullptr, poseAdjuster, cameraFrameInterval, nx, ny, testDepths);
      if (deProbe.m_constructorStatus != "") {
        return "DepthEstimator constructor failed for probe: " + deProbe.m_constructorStatus;
      }

      // Fill in left camera pair with a depth of 100 and the right with a depth of 200.
      deProbe.m_impl->m_cameraPairs[0]->m_depths.clear();
      deProbe.m_impl->m_cameraPairs[0]->m_depths.resize(nx * ny, 100);
      deProbe.m_impl->m_cameraPairs[1]->m_depths.clear();
      deProbe.m_impl->m_cameraPairs[1]->m_depths.resize(nx * ny, 200);

      // Test the depth from the origin, shooting along the 45-degree directions towards each plane.
      // Also take a step up in Z to check for the correct offset.
      estimatedDepth = deProbe.EstimateDepth(glm::vec3(0, 0, 0), glm::vec3(1, 1, 0));
      if (fabs(estimatedDepth - 100) > 100 * 1e-6) {
        return "EstimateDepth() probe failed for origin to right plane: expected 100, got " + std::to_string(estimatedDepth);
      }
      estimatedDepth = deProbe.EstimateDepth(glm::vec3(0, 0, 0), glm::vec3(-1, 1, 0));
      if (fabs(estimatedDepth - 200) > 200 * 1e-6) {
        return "EstimateDepth() probe failed for origin to left plane: expected 200, got " + std::to_string(estimatedDepth);
      }

      //================================================================================================
      // Rotate the cameras by N degrees around the Y axis so that we test the rotation code.
      // Also translate the cameras in Z so that we can test the position offset.
      double degN = 10.0;
      double cosN = cos(glm::radians(degN));
      double sinN = sin(glm::radians(degN));
      std::shared_ptr<CameraRenderInfo> camRot1(new CameraRenderInfo(1, { -cosN, -sinN, 1 }, { 0, 0, degN }, { width, height }, { 90.0, 90.0 },
        distortion, vignette, queues[0], -1.0f));
      std::shared_ptr<CameraRenderInfo> camRot2(new CameraRenderInfo(2, { cosN, sinN, 1 }, { 0, 0, degN }, { width, height }, { 90.0, 90.0 },
        distortion, vignette, queues[1], -1.0f));
      cameras.clear();
      cameras.push_back({ camRot1, camRot2 });
      DepthEstimator deRotated(cameras, nullptr, poseAdjuster, cameraFrameInterval, nx, ny, testDepths);
      if (deRotated.m_constructorStatus != "") {
        return "DepthEstimator constructor failed for rotated: " + deRotated.m_constructorStatus;
      }

      // Compute the depth estimate.
      res = deRotated.ComputeDepthEstimate(0);
      if (res != "") {
        return "ComputeDepthEstimate() failed for rotated: " + res;
      }

      // Test the depth estimates directly.
      for (size_t y = 0; y < ny; y++) {
        double depth = testDepths[y % testDepths.size()];
        if (y >= ny / 2) {
          // In the top half of the image, there are no features, so it should be the default depth.
          depth = testDepths.back();
        }
        for (size_t x = 0; x < nx; x++) {
          size_t i = y * nx + x;
          if (fabs(deRotated.m_impl->m_cameraPairs[0]->m_depths[i] - depth) > depth * 1e-6) {
            return "ComputeDepthEstimate() failed for rotated region " + std::to_string(i)
              + ", found " + std::to_string(deRotated.m_impl->m_cameraPairs[0]->m_depths[i]) + " expected " + std::to_string(depth);
          }
        }
      }
    }
  }

  return "";
}
