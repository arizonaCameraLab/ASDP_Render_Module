/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include <memory>
#include <GL/glew.h>
#include <ToneMap.h>
#include <DepthEstimator.h>
#include <Composite.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>
using namespace asdp;
using namespace asdp::render;

/// Maximum block size for the CUDA kernel, which matches the maximum number of samples in X or Y.
static const size_t MAX_BLOCK_SIZE = 100;

/// @brief CUDA kernel to compute the differences between two surfaces (OpenGL textures).
/// @param surface1, surface2 The surfaces to compare. They are RGBA or BGRA textures.
/// @param out The per-region array of output values to write.
/// @param nx The total width of the image.
/// @param ny The total height of the image.
__global__ void CompareSurfacesKernel(cudaSurfaceObject_t surface1, cudaSurfaceObject_t surface2, float* out,
  uint16_t nx, uint16_t ny)
{
  /// Block of memory to store the within-block results.
  __shared__ float sharedMem[MAX_BLOCK_SIZE][MAX_BLOCK_SIZE];
  __shared__ float rowSums[MAX_BLOCK_SIZE];
  __shared__ int rowCounts[MAX_BLOCK_SIZE];

  uint16_t x = blockIdx.x * blockDim.x + threadIdx.x;
  uint16_t y = blockIdx.y * blockDim.y + threadIdx.y;
  // The block size evenly divides the image size, so we don't need to check for out-of-bounds.
  /* if (x < nx && y < ny) */
  {
    // Read the data from both surfaces. The x coordinate is in bytes, so we need to multiply by the
    // size of the data type.
    uchar4 val1, val2;
    surf2Dread(&val1, surface1, x * sizeof(val1), y);
    surf2Dread(&val2, surface2, x * sizeof(val2), y);

    // If the first and third colors are not the same in either of the values, then the region is outside
    // of the projected area (the pixel is blue), so we record -1 as the value.  Otherwise, we record the
    // squared difference between the first color in each value.
    if (val1.x != val1.z || val2.x != val2.z) {
      sharedMem[threadIdx.y][threadIdx.x] = -1.0f;
    } else {
      float diff = val1.x - val2.x;
      sharedMem[threadIdx.y][threadIdx.x] = diff * diff;
    }

    // Wait until all threads in the block have completed and then have the first thread in each
    // row compute the sum and count of the valid values in the row.
    __syncthreads();
    if (threadIdx.x == 0) {
      rowSums[threadIdx.y] = 0.0f;
      rowCounts[threadIdx.y] = 0;
      for (size_t i = 0; i < blockDim.y; i++) {
        float val = sharedMem[threadIdx.y][i];
        if (val >= 0.0f) {
          rowSums[threadIdx.y] += val;
          rowCounts[threadIdx.y]++;
        }
      }
    }

    // Wait until all threads in the block have completed and then have one thread sum the
    // sum and counts and compute the final average.
    __syncthreads();
    if (threadIdx.x == 0 && threadIdx.y == 0) {
      float sum = 0.0f;
      int count = 0;
      for (size_t i = 0; i < blockDim.y; i++) {
        sum += rowSums[i];
        count += rowCounts[i];
      }

      // Avoid division by zero, return 0 in the case of no valid values.
      if (count == 0) { count = 1; }
      out[blockIdx.x + blockIdx.y * gridDim.x] = sum / count;
    }
  }
}

/// @brief Encapsulates the multiple depths for each camera pair.
/// @details Generate two sets of CompositeCameras covering all depths for each pair of cameras,
/// one set for the left camera and one for the right camera. 
class CameraPairInfo {
public:
  CameraPairInfo() = delete;
  CameraPairInfo(ToneMap const &toneMap, CameraRenderInfo camera1, CameraRenderInfo camera2,
    glm::dvec3 position, glm::dquat orientation,
    std::array<float, 2> fovsDeg, std::array<unsigned, 2> pixelCounts,
    std::shared_ptr<PoseAdjuster> poseAdjuster, Time cameraFrameInterval,
    std::vector<float> depths)
    : m_camera1(camera1)
    , m_camera2(camera2)
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

      CameraRenderInfo depth1 = camera1;
      depth1.ComputePlanarCameraMeshInfo(100, 100, depth);
      std::vector<CameraRenderInfo> composites1;
      composites1.push_back(depth1);
      depthInfo.m_composites[0] = std::make_shared<CompositeCameras>(composites1, m_toneMapTexture,
        m_poseAdjuster, cameraFrameInterval);

      CameraRenderInfo depth2 = camera2;
      depth2.ComputePlanarCameraMeshInfo(100, 100, depth);
      std::vector<CameraRenderInfo> composites2;
      composites2.push_back(depth2);
      depthInfo.m_composites[1] = std::make_shared<CompositeCameras>(composites2, m_toneMapTexture,
        m_poseAdjuster, cameraFrameInterval);

      // Generate a pair of frame buffers for the two cameras, with an associated color and depth
      // buffer for each.
      glGenFramebuffers(2, depthInfo.m_frameBuffers.data());

      glGenTextures(2, depthInfo.m_colorBuffers.data());
      for (size_t b = 0; b < 2; b++) {
        glBindTexture(GL_TEXTURE_2D, depthInfo.m_colorBuffers[b]);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, pixelCounts[0], pixelCounts[1], 0,
          GL_RGBA, GL_FLOAT, nullptr);
      }

      glGenRenderbuffers(2, depthInfo.m_depthBuffers.data());
      for (size_t b = 0; b < 2; b++) {
        glBindTexture(GL_TEXTURE_2D, depthInfo.m_depthBuffers[b]);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32, pixelCounts[0], pixelCounts[1], 0,
          GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
      }

      // Map the CUDA graphics resources for the color buffers.
      for (size_t b = 0; b < 2; b++) {
        cudaError_t res = cudaGraphicsGLRegisterImage(&depthInfo.m_cudaColorBuffers[b], depthInfo.m_colorBuffers[b],
          GL_TEXTURE_2D, cudaGraphicsRegisterFlagsSurfaceLoadStore);
        if (res != cudaSuccess) {
          m_constructorStatus = "Failed to register image: " + std::string(cudaGetErrorString(res));
          return;
        }
      }

      // Create the CUDA streams for the two cameras.
      for (size_t b = 0; b < 2; b++) {
        cudaStream_t* streamPtr = new cudaStream_t;
        cudaError_t res = cudaStreamCreate(streamPtr);
        if (res != cudaSuccess) {
          m_constructorStatus = "Failed to create stream: " + std::string(cudaGetErrorString(res));
          return;
        }
        depthInfo.m_streams[b] = streamPtr;
      }

      m_perDepths.push_back(depthInfo);
    }
  }

  ~CameraPairInfo() {
    // Delete the tone map texture.
    glDeleteTextures(1, &m_toneMapTexture);

    // Delete the frame bufffers, color buffers, and depth buffers.
    // Unmap the CUDA graphics resources for the color buffers.
    // Delete the CUDA streams.
    for (PerDepth &di : m_perDepths) {
      glDeleteFramebuffers(2, di.m_frameBuffers.data());
      for (size_t b = 0; b < 2; b++) {
        cudaGraphicsUnregisterResource(di.m_cudaColorBuffers[b]);
        cudaStreamDestroy(*(di.m_streams[b]));
      }
      glDeleteTextures(2, di.m_colorBuffers.data());
      glDeleteTextures(2, di.m_depthBuffers.data());
    }
  }

  CameraRenderInfo m_camera1, m_camera2;
  std::shared_ptr<PoseAdjuster> m_poseAdjuster;
  glm::dvec3 m_position;
  glm::dquat m_orientation;
  std::array<float, 2> m_fovsDeg;
  std::array<unsigned, 2> m_pixelCounts;
  GLuint m_toneMapTexture;

  typedef struct {
    float m_depth = 0.0f;
    /// @todo Consider pulling these out into yet another structure, making an array of 2 of them.
    std::array< std::shared_ptr<CompositeCameras>, 2> m_composites = {};
    std::array<GLuint, 2> m_frameBuffers = {};
    std::array<GLuint, 2> m_colorBuffers = {};
    std::array<GLuint, 2> m_depthBuffers = {};
    std::array<cudaGraphicsResource*, 2> m_cudaColorBuffers = {};
    std::array<cudaStream_t*, 2> m_streams = {};
  } PerDepth;

  std::vector<PerDepth> m_perDepths;

  std::string m_constructorStatus;
};

/// Provides implementation details for the DepthEstimator class
class DepthEstimator::DepthEstimatorImpl {
public:
  DepthEstimatorImpl() = delete;
  DepthEstimatorImpl(DepthEstimator *parent,
      std::vector< std::array<CameraRenderInfo, 2> > cameras,
      std::shared_ptr<PoseAdjuster> poseAdjuster,
      Time cameraFrameInterval,
      unsigned nx, unsigned ny,
      float minZRotDeg, float maxZRotDeg,
      float minXRotDeg, float maxXRotDeg,
      std::vector<float> depths)
    : m_parent(parent)
    , m_nx(nx)
    , m_ny(ny)
    , m_minZRotDeg(minZRotDeg)
    , m_maxZRotDeg(maxZRotDeg)
    , m_minXRotDeg(minXRotDeg)
    , m_maxXRotDeg(maxXRotDeg)
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
    ToneMap toneMap;  /// @todo Optimize the tone map so that we get good contrast, but it must be monochrome
    for (unsigned i = 0; i < cameras.size(); i++) {
      // For each pair, create a viewpoint that is halfway between
      // the two cameras with an orientation that is the average of the two.
      // We store the rotation as a quaternion and convert the Euler angles
      // to quaternions to average them.
      glm::dvec3 position;

      std::array<double, 3> const& p1 = cameras[i][0].m_positionMeters;
      std::array<double, 3> const& p2 = cameras[i][1].m_positionMeters;
      position = 0.5 * (glm::dvec3(p1[0], p1[1], p1[2]) + glm::dvec3(p2[0], p2[1], p2[2]));

      glm::quat orientation;
      glm::quat rotx = glm::angleAxis(cameras[i][0].m_orientationDegrees[0], glm::dvec3(1, 0, 0));
      glm::quat roty = glm::angleAxis(cameras[i][0].m_orientationDegrees[1], glm::dvec3(0, 1, 0));
      glm::quat rotz = glm::angleAxis(cameras[i][0].m_orientationDegrees[2], glm::dvec3(0, 0, 1));
      glm::quat rot1 = rotz * roty * rotx;

      rotx = glm::angleAxis(cameras[i][1].m_orientationDegrees[0], glm::dvec3(1, 0, 0));
      roty = glm::angleAxis(cameras[i][1].m_orientationDegrees[1], glm::dvec3(0, 1, 0));
      rotz = glm::angleAxis(cameras[i][1].m_orientationDegrees[2], glm::dvec3(0, 0, 1));
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
        double xHalfWidth = tan(glm::radians(cameras[i][c].m_fovDegrees[0]) * 0.5) * depthForFOV;
        double yHalfWidth = tan(glm::radians(cameras[i][c].m_fovDegrees[1]) * 0.5) * depthForFOV;

        std::array<double, 3> corner = { xHalfWidth, yHalfWidth, -depthForFOV };
        std::array<double, 3> distortedCorner = cameras[i][c].m_distortion->MapPoint(corner);

        double hFOV = glm::degrees(2.0 * atan(fabs(distortedCorner[2]) / distortedCorner[0]));
        double vFOV = glm::degrees(2.0 * atan(fabs(distortedCorner[2]) / distortedCorner[1]));

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
      uint16_t maxX = std::max(cameras[i][0].m_resolutionPixels[0], cameras[i][1].m_resolutionPixels[0]);
      uint16_t maxY = std::max(cameras[i][0].m_resolutionPixels[1], cameras[i][1].m_resolutionPixels[1]);
      pixelCounts[0] = maxX * maxXRatio;
      if (pixelCounts[0] % m_nx != 0) { pixelCounts[0] += m_nx - (pixelCounts[0] % m_nx); }
      pixelCounts[1] = maxY * maxYRatio;
      if (pixelCounts[1] % m_ny != 0) { pixelCounts[1] += m_ny - (pixelCounts[1] % m_ny); }

      // Make the camera pair info.
      CameraPairInfo cameraPairInfo(toneMap, cameras[i][0], cameras[i][1],
        position, orientation, fovsDeg, pixelCounts,
        poseAdjuster, cameraFrameInterval, depths);
      if (!cameraPairInfo.m_constructorStatus.empty()) {
        m_constructorStatus = cameraPairInfo.m_constructorStatus;
        return;
      }
      m_cameraPairs.push_back(cameraPairInfo);
    }
  }

  std::string ComputeDepthEstimate(Time time)
  {
    // OpenGL fence objects to let us ensure that we're done with OpenGL rendering before we
    // start to map the buffers to CUDA and do the depth estimation.  There is one entry
    // for each camera with one entry for each depth with an entry for each of the pair of cameras.
    std::vector < std::vector< std::array<GLsync, 2> > > fences;

    // For each camera pair and depth, render the two cameras into the frame buffers and keep track of the fences.
    for (size_t c = 0; c < m_cameraPairs.size(); c++) {
      CameraPairInfo& cpi = m_cameraPairs[c];
      std::vector< std::array<GLsync, 2> > cFences;
      for (size_t d = 0; d < cpi.m_perDepths.size(); d++) {
        std::array<GLsync, 2> dFences;
        for (size_t b = 0; b < 2; b++) {
          // Fill in the render info.
          ViewRenderInfo vri;
          for (size_t i = 0; i < 3; i++) {
            vri.viewpoint[i] = cpi.m_position[i];
          }
          for (size_t i = 0; i < 4; i++) {
            vri.orientation[i] = cpi.m_orientation[i];
          }
          vri.leftHalfFOV = vri.rightHalfFOV = cpi.m_fovsDeg[0]/2.0f;
          vri.topHalfFOV = vri.bottomHalfFOV = cpi.m_fovsDeg[1]/2.0f;
          vri.frameBuffer = cpi.m_perDepths[d].m_frameBuffers[b];
          vri.colorBuffer = cpi.m_perDepths[d].m_colorBuffers[b];
          vri.depthBuffer = cpi.m_perDepths[d].m_depthBuffers[b];
          vri.x = 0;
          vri.y = 0;
          vri.width = cpi.m_pixelCounts[0];
          vri.height = cpi.m_pixelCounts[1];

          // Render the composite camera and construct a fence to indicate completion.
          cpi.m_perDepths[d].m_composites[b]->Render(time, {vri});
          dFences[b] = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
        }
        cFences.push_back(dFences);
      }
      fences.push_back(cFences);
    }

    // Loop back through the camera pairs and depths, waiting for both fences to complete
    // and then mapping the color buffers to CUDA and running the depth estimation.
    for (size_t c = 0; c < m_cameraPairs.size(); c++) {
      CameraPairInfo& cpi = m_cameraPairs[c];
      for (size_t d = 0; d < cpi.m_perDepths.size(); d++) {
        CameraPairInfo::PerDepth &pd = cpi.m_perDepths[d];

        // Wait for both fences to complete.
        for (size_t b = 0; b < 2; b++) {
          // 1-second timeout.
          GLenum ret = glClientWaitSync(fences[c][d][b], 0, 1000000000);
          if (ret != GL_ALREADY_SIGNALED && ret != GL_CONDITION_SATISFIED) {
            return "ComputeDepthEstimate: glClientWaitSync() failed for pair " + std::to_string(c)
              + " depth " + std::to_string(d) + " camera " + std::to_string(b);
          }
        }

        // Map the color buffers to CUDA, using the already-registered images.
        // Run the depth estimation kernels and then read back the values to CPU memory.
        for (size_t b = 0; b < 2; b++) {
          cudaError_t res = cudaGraphicsMapResources(1, &pd.m_cudaColorBuffers[b], *(pd.m_streams[b]));
          if (res != cudaSuccess) {
            return "ComputeDepthEstimate: cudaGraphicsMapResources() failed for pair " + std::to_string(c)
              + " depth " + std::to_string(d) + " camera " + std::to_string(b) + ": "
              + std::string(cudaGetErrorString(res));
          }

          // Run the depth estimation kernels, which must handle portions of a region that are outside
          // of the projected area (they will be blue).
          /// @todo

          // Copy the results back to CPU memory.
          /// @todo
        }
      }
    }

    // Wait for all the CUDA streams to complete.
    for (auto cpi : m_cameraPairs) {
      for (auto pd : cpi.m_perDepths) {
        for (size_t b = 0; b < 2; b++) {
          cudaStreamSynchronize(*(pd.m_streams[b]));
        }
      }
    }

    // Loop back through the camera pairs and depths, unmap the color buffers from CUDA.
    for (size_t c = 0; c < m_cameraPairs.size(); c++) {
      CameraPairInfo& cpi = m_cameraPairs[c];
      for (size_t d = 0; d < cpi.m_perDepths.size(); d++) {
        CameraPairInfo::PerDepth& pd = cpi.m_perDepths[d];
        for (size_t b = 0; b < 2; b++) {
          cudaError_t res = cudaGraphicsUnmapResources(1, &pd.m_cudaColorBuffers[b], *(pd.m_streams[b]));
          if (res != cudaSuccess) {
            return "ComputeDepthEstimate: cudaGraphicsUnmapResources() failed for pair " + std::to_string(c)
              + " depth " + std::to_string(d) + " camera " + std::to_string(b) + ": "
              + std::string(cudaGetErrorString(res));
          }
        }
      }
    }

    // Find the best depth value for each region in the depth map.
    // Determine the best-matched and worse-matched depth at each location.
    // Determine the difference between the best-matched and worse-matched depths as a quality of fit measure.
    // Compute a weighted Gaussian fit, where the weights include the quality of fit measure.


    /// @todo
    return "@todo";
  }

  DepthEstimator *m_parent;
  std::string m_constructorStatus;

  /// Camera pairs to use to estimate depth.
  std::vector<CameraPairInfo> m_cameraPairs;

  /// Number of points to create in the X direction.
  unsigned m_nx;

  /// Number of points to create in the Y direction.
  unsigned m_ny;

  /// Minimum Z rotation in degrees.
  float m_minZRotDeg;

  /// Maximum Z rotation in degrees.
  float m_maxZRotDeg;

  /// Minimum X rotation in degrees.
  float m_minXRotDeg;

  /// Maximum X rotation in degrees.
  float m_maxXRotDeg;

  /// Default depth to use if the depth cannot be estimated.
  float m_defaultDepth;
};


DepthEstimator::DepthEstimator(std::vector< std::array<CameraRenderInfo, 2> > cameras,
  std::shared_ptr<PoseAdjuster> poseAdjuster, Time cameraFrameInterval,
  unsigned nx, unsigned ny,
  float minZRotDeg, float maxZRotDeg,
  float minXRotDeg, float maxXRotDeg,
  std::vector<float> depths)
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
  if (cameras[0][0].m_resolutionPixels[0] / nx > MAX_BLOCK_SIZE) {
    m_constructorStatus = "DepthEstimator::DepthEstimator(): Block size must be less than or equal to "
      + std::to_string(MAX_BLOCK_SIZE);
    return;
  }
  if (ny == 0) {
    m_constructorStatus = "DepthEstimator::DepthEstimator(): ny must be greater than 0";
    return;
  }
  if (cameras[0][0].m_resolutionPixels[1] / ny > MAX_BLOCK_SIZE) {
    m_constructorStatus = "DepthEstimator::DepthEstimator(): Block size must be less than or equal to "
      + std::to_string(MAX_BLOCK_SIZE);
    return;
  }

  // Create the implementation.
  m_impl = std::make_unique<DepthEstimatorImpl>(this, cameras,
    poseAdjuster, cameraFrameInterval, nx, ny,
    minZRotDeg, maxZRotDeg, minXRotDeg, maxXRotDeg, depths);
  m_constructorStatus = m_impl->m_constructorStatus;
}

std::string DepthEstimator::ComputeDepthEstimate(Time time)
{
  if (!m_constructorStatus.empty()) {
    return "DepthEstimator::ComputeDepthEstimate(): constructor failed: " + m_constructorStatus;
  }

  return m_impl->ComputeDepthEstimate(time);
}

float DepthEstimator::EstimateDepth(const glm::vec3& point, const glm::vec3& direction) const
{
  /// @todo

  return m_impl->m_defaultDepth;
}


std::string DepthEstimator::Test()
{

  return "@todo Write tests for DepthEstimator";
}
