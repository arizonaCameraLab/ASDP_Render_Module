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
  {}

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
};


DepthEstimator::DepthEstimator(std::vector<CameraRenderInfo> cameras, unsigned nx, unsigned ny,
  float minZRotDeg, float maxZRotDeg,
  float minXRotDeg, float maxXRotDeg,
  std::vector<float> depths)
{
  m_impl = std::make_unique<DepthEstimatorImpl>(this, cameras, nx, ny,
    minZRotDeg, maxZRotDeg, minXRotDeg, maxXRotDeg, depths);
}

std::string DepthEstimator::Test()
{

  return "@todo Write tests for DepthEstimator";
}
