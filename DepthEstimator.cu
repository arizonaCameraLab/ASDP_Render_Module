/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include <DepthEstimator.h>
using namespace asdp::render;

/// Provides implementation details for the DepthEstimator class
class DepthEstimator::DepthEstimatorImpl {
public:
  DepthEstimatorImpl() = delete;
  DepthEstimatorImpl(DepthEstimator *parent,
      std::vector<CameraRenderInfo> cameras, unsigned nx, unsigned ny,
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
    if (m_depths.size() == 0) {
      // Error -- no default depth
      m_defaultDepth = -1.0f;
    } else {
      // Use the furthest depth as the default.
      m_defaultDepth = m_depths.back();
    }
  }

  DepthEstimator *m_parent;

  /// Cameras to use to estimate depth.
  std::vector<CameraRenderInfo> m_cameras;

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


DepthEstimator::DepthEstimator(std::vector<CameraRenderInfo> cameras, unsigned nx, unsigned ny,
  float minZRotDeg, float maxZRotDeg,
  float minXRotDeg, float maxXRotDeg,
  std::vector<float> depths)
  : m_impl(std::make_unique<DepthEstimatorImpl>(this, cameras, nx, ny,
    minZRotDeg, maxZRotDeg, minXRotDeg, maxXRotDeg, depths))
{
  // Check the parameters.
  if (cameras.size() % 2 != 0) {
    m_constructorStatus = "DepthEstimator::DepthEstimator(): cameras.size() must be even";
    return;
  }
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

  // Make pairs of cameras from the list that was passed it.  Initially, we assume that they
  // are listed in order by pairs.  For each pair, create a viewpoint that is halfway between
  // the two cameras with an orientation that is the average of the two.
  /// @todo Consider using the geometry of the situation to make general pairs.
  /// @todo

  // Generate two sets of manifolds covering all depths for each pair of cameras, one set for the
  // left camera and one for the right camera.  The two manifolds for each depth will be rendered
  // separately into a pair of frame buffers with the same (average of the two cameras) view frustum
  // and then compared to estimate the depth.
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
