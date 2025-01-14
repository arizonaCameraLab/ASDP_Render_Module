/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include <iostream>   /// @todo Remove this
#include <memory>
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

  // Global coordinates in the images. We skip by total block size, not thread-block size.
  unsigned x = blockIdx.x * blockDim.x + threadIdx.x;
  unsigned y = blockIdx.y * rowsPerBlock + threadIdx.y;

  unsigned xLocal = threadIdx.x;
  for (unsigned iter = 0; iter < iterations; iter++) {
    unsigned yLocal = threadIdx.y + iter * rowsPerIteration;
    // The block size matches the number of threads in X, so we don't need to check bounds on that axis.
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
      if (val1.x != val1.z || val2.x != val2.z) {
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
    // The block size matches the number of threads in X, so we don't need to check bounds on that axis.
    if (yLocal < rowsPerBlock) {
      // @todo We can get better utilization if we have threads in different columns handle
      // the different rows.
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
    for (size_t i = 0; i < blockDim.y; i++) {
      sum += rowSums[i];
      count += rowCounts[i];
    }

    // Avoid division by zero, return 0 in the case of no valid values.
    if (count == 0) { count = 1; }
    out[blockIdx.x + blockIdx.y * gridDim.x] = sum / count;
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

      glGenRenderbuffers(2, depthInfo.m_depthBuffers.data());
      for (size_t b = 0; b < depthInfo.m_depthBuffers.size(); b++) {
        glBindRenderbuffer(GL_RENDERBUFFER, depthInfo.m_depthBuffers[b]);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, pixelCounts[0], pixelCounts[1]);
        glBindRenderbuffer(GL_RENDERBUFFER, 0);
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

      // Fill in the default depth for all regions.
      m_depths.resize(pixelCounts[0] * pixelCounts[1], defaultDepth);
    }
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

  std::array<CameraRenderInfo,2 > m_cameras;
  std::shared_ptr<PoseAdjuster> m_poseAdjuster;
  glm::dvec3 m_position;
  glm::dquat m_orientation;
  std::array<float, 2> m_fovsDeg;
  std::array<unsigned, 2> m_pixelCounts;
  GLuint m_toneMapTexture;

  typedef struct {
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
  } PerDepth;

  std::vector<PerDepth> m_perDepths;

  /// Computed estimated depth at every region location.
  std::vector<float> m_depths;

  std::string m_constructorStatus;
};

/// Provides implementation details for the DepthEstimator class
class DepthEstimator::DepthEstimatorImpl {
public:
  friend class DepthEstimator;
  DepthEstimatorImpl() = delete;
  DepthEstimatorImpl(DepthEstimator *parent,
      std::vector< std::array<CameraRenderInfo, 2> > cameras,
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
      std::shared_ptr<CameraPairInfo> cameraPairInfo = std::make_shared<CameraPairInfo>(
        toneMap, cameras[i][0], cameras[i][1],
        position, orientation, fovsDeg, pixelCounts,
        poseAdjuster, cameraFrameInterval, depths, m_defaultDepth);
      if (!cameraPairInfo->m_constructorStatus.empty()) {
        m_constructorStatus = cameraPairInfo->m_constructorStatus;
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
              + " depth " + std::to_string(d) + " camera " + std::to_string(b);
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
        unsigned rowsPerIteration = blockSize.x;
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
      std::vector<float> qualityOfFit(numRegions, 0.0f);
      for (size_t c = 0; c < m_cameraPairs.size(); c++) {
        CameraPairInfo& cpi = *m_cameraPairs[c];
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
      }

      // Determine the difference between the best-matched and worst-matched scores as a certainty/quality of fit measure.
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
      for (size_t i = 0; i < numRegions; i++) {
        if (qualityOfFit[i] < m_fitnessThreshold) {
          cpi.m_depths[i] = m_defaultDepth;
        } else {
          cpi.m_depths[i] = bestDepths[i];
        }
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
};

DepthEstimator::DepthEstimator(std::vector< std::array<CameraRenderInfo, 2> > cameras,
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

  // Find the coordinate of the ray's piercing point on the plane at the default depth.
  // The plane's orientation is the camera axis rotated by the camera orientation, and its
  // distance from the origin is the default depth.
  glm::dvec3 pierce;
  glm::dvec3 planeNormal = m_impl->m_cameraPairs[bestPair]->m_orientation * cameraDir;
  glm::dvec3 originOnPlane = m_impl->m_cameraPairs[bestPair]->m_position + double(m_impl->m_defaultDepth) * planeNormal;
  if (!intersectRayWithPlane(rayStart, rayDir, originOnPlane, planeNormal, pierce)) {
    return m_impl->m_defaultDepth;
  }

  // Determine the coordinates in the view frustum of the piercing point, clamping to the
  // range -1 to 1 in each dimension.  In helicopter space, the screen is in the XZ plane,
  // so screen Y will correspond to helicopter-space Z.
  double halfX = m_impl->m_defaultDepth * tan(glm::radians(m_impl->m_cameraPairs[bestPair]->m_fovsDeg[0] / 2));
  double halfY = m_impl->m_defaultDepth * tan(glm::radians(m_impl->m_cameraPairs[bestPair]->m_fovsDeg[1] / 2));
  glm::dvec3 xDir = m_impl->m_cameraPairs[bestPair]->m_orientation * glm::dvec3(1, 0, 0);
  glm::dvec3 yDir = m_impl->m_cameraPairs[bestPair]->m_orientation * glm::dvec3(0, 0, 1);
  double x = glm::dot(pierce - originOnPlane, xDir) / halfX;
  double y = glm::dot(pierce - originOnPlane, yDir) / halfY;
  x = glm::clamp(x, -1.0, 1.0);
  y = glm::clamp(y, -1.0, 1.0);

  // Convert the coordinates to the region index, which goes from 0 to 1 on each axis with the
  // -1 to 1 range covering half a pixel beyond the index point for each.  Clamp to the range 0 to 1 on each axis.
  // Interpolate the depth value based on the fractional piercing index.
  double xScaled = x * m_impl->m_nx / (m_impl->m_nx - 1.0);
  double yScaled = y * m_impl->m_ny / (m_impl->m_ny - 1.0);
  double xCoord = (xScaled + 1.0) / 2.0 * (m_impl->m_nx - 1.0);
  xCoord = glm::clamp(xCoord, 0.0, m_impl->m_nx - 1.0);
  double yCoord = (yScaled + 1.0) / 2.0 * (m_impl->m_ny - 1.0);
  yCoord = glm::clamp(yCoord, 0.0, m_impl->m_ny - 1.0);

  // Look up the four values (floor and ceiling) around the point and use bilinear interpolation
  // to determine the depth at the point.
  size_t xFloor = size_t(floor(xCoord));
  size_t xCeil = size_t(ceil(xCoord));
  size_t yFloor = size_t(floor(yCoord));
  size_t yCeil = size_t(ceil(yCoord));
  double xFrac = xCoord - xFloor;
  double yFrac = yCoord - yFloor;
  double depthFF = m_impl->m_cameraPairs[bestPair]->m_depths[yFloor * m_impl->m_nx + xFloor];
  double depthFC = m_impl->m_cameraPairs[bestPair]->m_depths[yFloor * m_impl->m_nx + xCeil];
  double depthCF = m_impl->m_cameraPairs[bestPair]->m_depths[yCeil * m_impl->m_nx + xFloor];
  double depthCC = m_impl->m_cameraPairs[bestPair]->m_depths[yCeil * m_impl->m_nx + xCeil];
  double depth = (1 - xFrac) * (1 - yFrac) * depthFF + xFrac * (1 - yFrac) * depthFC +
    (1 - xFrac) * yFrac * depthCF + xFrac * yFrac * depthCC;

  // Estimate the contact point as that depth from the camera pair origin in the direction of the
  // piercing point, scaled by ratio of the found depth to the default depth to make it match the
  // one that would have been found for a plane at the found depth.
  glm::dvec3 cameraToPierce = pierce - m_impl->m_cameraPairs[bestPair]->m_position;
  glm::dvec3 contactPoint = m_impl->m_cameraPairs[bestPair]->m_position
    + (depth / m_impl->m_defaultDepth) * cameraToPierce;

  // Return the distance from the ray start to that point.
  return glm::length(glm::vec3(contactPoint - rayStart));
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

std::string DepthEstimator::Test()
{
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
      return "intersectRayWithPlane() failed for perpendicular ray and plane";
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

    // Put into a block so that we destroy things in here before we destroy the context.
    {
      uint16_t nx = 12;   ///< Number of points to create in the X direction.  Must be divisible by 4 for our tests below.
      uint16_t ny = 12;

      uint16_t width = 50*nx;  ///< Width of the frame buffer.
      uint16_t height = 50*ny; ///< Height of the frame buffer.  Must be divisible by ny for our tests below.

      // Construct a DepthEstimator after making the objects required to construct it.
      std::vector< std::array<CameraRenderInfo, 2> > cameras;
      DistortionNone* dNone = new DistortionNone();
      std::shared_ptr<Distortion> distortion(dNone);
      VignetteNone* vNone = new VignetteNone();
      std::shared_ptr<Vignette> vignette(vNone);
      std::shared_ptr<ImageQueue> queue1(new ImageQueue);
      std::shared_ptr<ImageQueue> queue2(new ImageQueue);
      std::vector< std::shared_ptr<ImageQueue> > queues;
      queues.push_back(queue1);
      queues.push_back(queue2);
      CameraRenderInfo cam1(1, { -1, 0, 0 }, { 0, 0, 0 }, { width, height}, { 90.0, 90.0 },
        distortion, vignette, queues[0]);
      CameraRenderInfo cam2(2, { 1, 0, 0 }, { 0, 0, 0 }, { width, height}, { 90.0, 90.0 },
        distortion, vignette, queues[1]);
      cameras.push_back({ cam1, cam2 });

      std::shared_ptr<PoseAdjuster> poseAdjuster = std::make_shared<PoseAdjuster>();
      // Use the same value for the camera frame interval and the exposure time on the frames
      // so that we don't engage the time-varying brightness adjustment on the render system.
      float cameraFrameInterval = 1.0f;
      std::vector<float> testDepths = { 10, 20, 50, 100, 200, 500, 1000 };
      DepthEstimator de(cameras, poseAdjuster, cameraFrameInterval, nx, ny, testDepths);
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

      // Test in -Y screen (-Z world) and make sure that the answer is the centerDepth.
      dz = -dz;
      expectedDepth = sqrt(1 + dz * dz) * centerDepth;
      estimatedDepth = de.EstimateDepth(glm::vec3(0, 0, 0), glm::vec3(0, 1, dz));
      if (fabs(estimatedDepth - expectedDepth) > expectedDepth * 1e-6) {
        return "EstimateDepth() varying depth failed for interpolating between two points in -Y";
      }

      //================================================================================================
      // Produce a set of images with known depths for each camera and test ComputeDepthEstimate() on them.
      // We analytically render the images with different depths for different image regions in Y so that
      // each of the ny samples is completely at the same depth.  We use a sum of sinusoids at relatively
      // prime frequencies with different phases to make the image have contrast and a specific alignment.
      // We move this pattern to different Z depths for each region in the Y camera axis.

      // Start with a grey-filled single-color image.  This will be the background.
      std::vector<uint16_t> blankImage(width * height, 32768);

      // Make a copy of the blue image for the each camera and then fill its upper half with the test
      // pattern at different depths.  The image starts at the top left, so this is the first part of the
      // image.

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
          double halfX = depth * tan(glm::radians(de.m_impl->m_cameraPairs[0]->m_cameras[c].m_fovDegrees[0] / 2));
          double xLeft = -halfX * (double(width) / (width - 1));
          double xRight = halfX * (double(width) / (width - 1));
          double scale = (xRight - xLeft) / (width - 1);
          double offset = -xLeft + (0.5 * scale);

          // Add the camera's X center to the offset.
          offset += de.m_impl->m_cameraPairs[0]->m_cameras[c].m_positionMeters[0];

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
          de.m_impl->m_cameraPairs[0]->m_cameras[c].m_imageQueue->InsertImage(id);
        }
      }

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

      // Check the depth estimates using probe rays.
      /// @todo

    }
  }


  return "@todo Write more tests for DepthEstimator";
}
