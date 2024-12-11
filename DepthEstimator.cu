/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include <memory>
#include <ToneMap.h>
#include <DepthEstimator.h>
#include <Composite.h>
using namespace asdp;
using namespace asdp::render;

/// Encapsulates the multiple depths for each camera pair.
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
    , m_depths(depths)
  {
    // Construct the tone map to use.
    m_toneMapTexture = toneMap.GenerateTexture();

    // Create the composite cameras for each depth, adjusting the camera render info to suit.
    for (float depth : depths) {
      CameraRenderInfo depth1 = camera1;
      depth1.ComputePlanarCameraMeshInfo(100, 100, depth);
      std::vector<CameraRenderInfo> composites1;
      composites1.push_back(depth1);
      m_composites1.push_back(std::make_shared<CompositeCameras>(composites1, m_toneMapTexture,
        m_poseAdjuster, cameraFrameInterval));

      CameraRenderInfo depth2 = camera2;
      depth2.ComputePlanarCameraMeshInfo(100, 100, depth);
      std::vector<CameraRenderInfo> composites2;
      composites2.push_back(depth2);
      m_composites1.push_back(std::make_shared<CompositeCameras>(composites2, m_toneMapTexture,
        m_poseAdjuster, cameraFrameInterval));
    }
  }

  ~CameraPairInfo() {
    // Delete the tone map texture.
    glDeleteTextures(1, &m_toneMapTexture);
  }

  CameraRenderInfo m_camera1, m_camera2;
  std::shared_ptr<PoseAdjuster> m_poseAdjuster;
  glm::dvec3 position;
  glm::dquat m_orientation;
  std::array<float, 2> m_fovsDeg;
  std::array<unsigned, 2> m_pixelCounts;
  GLuint m_toneMapTexture;
  std::vector<float> m_depths;
  std::vector< std::shared_ptr<CompositeCameras> > m_composites1, m_composites2;
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

    // Generate two sets of CompositeCameras covering all depths for each pair of cameras, one set for the
    // left camera and one for the right camera.  The two sets for each depth will be rendered
    // separately into a pair of frame buffers with the same (average of the two cameras) view frustum
    // and then compared to estimate the depth.
    ToneMap toneMap;  /// @todo Optimize the tone map so that we get good contrast
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
      // the results are an even number of pixels in X and Y.
      /// @todo
      std::array<unsigned, 2> pixelCounts;
      uint16_t maxX = std::max(cameras[i][0].m_resolutionPixels[0], cameras[i][1].m_resolutionPixels[0]);
      uint16_t maxY = std::max(cameras[i][0].m_resolutionPixels[1], cameras[i][1].m_resolutionPixels[1]);
      pixelCounts[0] = maxX * maxXRatio;
      if (pixelCounts[0] % 2 != 0) { pixelCounts[0]++; }
      pixelCounts[1] = maxY * maxYRatio;
      if (pixelCounts[1] % 2 != 0) { pixelCounts[1]++; }

      // Make the camera pair info.
      CameraPairInfo cameraPairInfo(toneMap, cameras[i][0], cameras[i][1],
        position, orientation, fovsDeg, pixelCounts,
        poseAdjuster, cameraFrameInterval, depths);
      m_cameraPairs.push_back(cameraPairInfo);
    }

    /// @todo
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
  if (ny == 0) {
    m_constructorStatus = "DepthEstimator::DepthEstimator(): ny must be greater than 0";
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

  /// @todo
  return "";
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
