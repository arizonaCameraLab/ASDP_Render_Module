/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include <DepthEstimator.h>
using namespace asdp::render;

/// Provides implementation details for the DepthEstimator class
class DepthEstimator::DepthEstimatorImpl {
public:
  DepthEstimatorImpl() = delete;
  DepthEstimatorImpl(DepthEstimator *parent) : m_parent(parent) {}

  DepthEstimator *m_parent;
};


DepthEstimator::DepthEstimator(std::vector<CameraRenderInfo> cameras, unsigned nx, unsigned ny,
  std::vector<float> depths)
  : m_cameras(cameras)
  , m_nx(nx)
  , m_ny(ny)
  , m_depths(depths) /// @todo May not need to store these, we can generate the manifolds here
{
  m_impl = std::make_unique<DepthEstimatorImpl>(this);
}

std::string DepthEstimator::Test()
{

  return "@todo Write tests for DepthEstimator";
}
