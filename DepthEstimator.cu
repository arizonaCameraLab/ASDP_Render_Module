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
  CameraPairInfo(CameraRenderInfo camera1, CameraRenderInfo camera2,
    std::shared_ptr<PoseAdjuster> poseAdjuster, Time cameraFrameInterval,
    std::vector<float> depths)
    : m_camera1(camera1)
    , m_camera2(camera2)
    , m_depths(depths)
  {
    // Construct the tone map to use.
    m_toneMapTexture = m_toneMap.GenerateTexture();

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
    /// @todo
  }

  ToneMap m_toneMap;
  std::shared_ptr<PoseAdjuster> m_poseAdjuster;
  GLuint m_toneMapTexture;
  CameraRenderInfo m_camera1;
  CameraRenderInfo m_camera2;
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
      unsigned nx, unsigned ny,
      float minZRotDeg, float maxZRotDeg,
      float minXRotDeg, float maxXRotDeg,
      std::vector<float> depths)
    : m_parent(parent)
    , m_cameras(cameras)
    , m_nx(nx)
    , m_ny(ny)
    , m_minZRotDeg(minZRotDeg)
    , m_maxZRotDeg(maxZRotDeg)
    , m_minXRotDeg(minXRotDeg)
    , m_maxXRotDeg(maxXRotDeg)
    , m_depths(depths)
  {
    // Find the default depth.
    if (m_depths.size() == 0) {
      // Error -- no default depth
      m_defaultDepth = -1.0f;
    } else {
      // Use the furthest depth as the default.
      m_defaultDepth = m_depths.back();
    }

    // Generate two sets of CompositeCameras covering all depths for each pair of cameras, one set for the
    // left camera and one for the right camera.  The two sets for each depth will be rendered
    // separately into a pair of frame buffers with the same (average of the two cameras) view frustum
    // and then compared to estimate the depth.
    /// @todo

  }

  DepthEstimator *m_parent;

  /// Camera pairs to use to estimate depth.
  std::vector< std::array<CameraRenderInfo, 2> > m_cameras;

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

  /// Depths to check in meters.
  std::vector<float> m_depths;

  /// Default depth to use if the depth cannot be estimated.
  float m_defaultDepth;
};


DepthEstimator::DepthEstimator(std::vector< std::array<CameraRenderInfo, 2> > cameras,
  std::shared_ptr<PoseAdjuster> poseAdjuster,
  unsigned nx, unsigned ny,
  float minZRotDeg, float maxZRotDeg,
  float minXRotDeg, float maxXRotDeg,
  std::vector<float> depths)
  : m_impl(std::make_unique<DepthEstimatorImpl>(this, cameras, poseAdjuster, nx, ny,
    minZRotDeg, maxZRotDeg, minXRotDeg, maxXRotDeg, depths))
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

  // For each pair, create a viewpoint that is halfway between
  // the two cameras with an orientation that is the average of the two.
  /// @todo


  // Determine the aspect ratio of the frame buffer that will be used to render the manifolds.
  // It should cover the range of the manifolds, including their distortion.  Then determine the
  // pixel count, which should be an even multiple of the number of samples in each dimension
  // and its ratio should be similar to the aspect ration of the frame buffer and it should have
  // at least as many pixels as the camera images in each dimension.
  /// @todo

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
