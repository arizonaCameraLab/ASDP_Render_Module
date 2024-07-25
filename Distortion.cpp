/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include <cmath>
#include <iostream>
#include <Distortion.h>
using namespace asdp::render;

DistortionRadialLERP::DistortionRadialLERP(std::array<double, 2> const& COP, std::vector<std::array<double, 2>> const& controlPoints)
  : m_COP(COP), m_ControlPoints(controlPoints)
{
  if (m_ControlPoints.size() < 2) {
    std::cerr << "DistortionRadialLERP: Not enough control points: " << std::to_string(m_ControlPoints.size()) << std::endl;
  }
  if (m_ControlPoints.front()[0] != 0) {
    std::cerr << "DistortionRadialLERP: First control point distance must be zero: " << std::to_string(m_ControlPoints.front()[0]) << std::endl;
  }
}

std::array<double, 3> DistortionRadialLERP::MapPoint(std::array<double, 3> point) const
{
  // If we don't have enough control points, return the point unchanged
  if (m_ControlPoints.size() < 2) {
    return point;
  }

  // If the point is on the Z = 0 plane, return the point unchanged
  if (point[2] == 0) {
    return point;
  }

  // Calculate the vector from the center of projection to the point.
  // Project the vector onto the Z = -1 plane (scales it to Z = -1)
  double scale = -1 / point[2];
  std::array<double, 2> vec = {point[0] - m_COP[0]/scale, point[1] - m_COP[1]/scale};
  vec[0] *= scale;
  vec[1] *= scale;
  double dist = std::sqrt(vec[0] * vec[0] + vec[1] * vec[1]);

  // If the distance is before the first control point or after the last control point, return the point unchanged
  if (dist < m_ControlPoints.front()[0] || dist > m_ControlPoints.back()[0]) {
    return point;
  }

  // Find the control points that bound the distance
  size_t i = 0;
  while (i < (m_ControlPoints.size() - 1) && m_ControlPoints[i + 1][0] < dist) {
    i++;
  }

  // Interpolate between the control points
  double t = (dist - m_ControlPoints[i][0]) / (m_ControlPoints[i + 1][0] - m_ControlPoints[i][0]);
  double d = m_ControlPoints[i][1] * (1 - t) + m_ControlPoints[i + 1][1] * t;

  // Scale the vector by both the inverse of the original scaling and the ratio of the interpolated distance to the
  // original and add it to the center of projection.  Handle the case where the point is right at the center of projection.
  double reScale = (1 / scale);
  if (dist > 0) {
    reScale *= (d / dist);
  }

  return {m_COP[0]/scale + vec[0] * reScale, m_COP[1]/scale + vec[1] * reScale, point[2]};
}


//==============================================================================
// Test and its helper functions

bool isNear(std::array<double, 3> const& a, std::array<double, 3> const& b, double epsilon = 1e-6)
{
  return std::abs(a[0] - b[0]) < epsilon && std::abs(a[1] - b[1]) < epsilon && std::abs(a[2] - b[2]) < epsilon;
}

std::string Distortion::Test()
{
  // Test the DistortionNone class
  {
    DistortionNone none;
    std::array<double, 3> point = {1, 2, 3};
    if (!isNear(none.MapPoint(point), point)) {
      return "DistortionNone: Point not returned unchanged";
    }
  }

  // Test the DistortionRadialLERP class
  {
    // Test an identity distortion that does not change anything with a nonzero
    // center of projection.
    std::array<double, 2> COP = {-0.15, 0.5};
    std::vector<std::array<double, 2>> controlPoints = {{0, 0}, {1, 1}};
    DistortionRadialLERP radialLERP(COP, controlPoints);
    std::array<double, 3> point = {COP[0], COP[1], -1};
    if (!isNear(radialLERP.MapPoint(point), point)) {
      return "DistortionRadialLERP: Unit radial point not returned unchanged";
    }
    point = { 2*COP[0], 2*COP[1], -2};
    if (!isNear(radialLERP.MapPoint(point), point)) {
      return "DistortionRadialLERP: Radial point not returned unchanged at Z = 2";
    }
    for (double x = -2; x <= 2; x += 0.1) {
      for (double y = -2; y <= 2; y += 0.1) {
        point = {x, y, -2};
        if (!isNear(radialLERP.MapPoint(point), point)) {
          return "DistortionRadialLERP: Point not returned unchanged at Z = 2";
        }
      }
    }
  }
  {
    // Test a factor-of-2 distortion both inside and outside the unit circle,
    // with a nonzero center of projection.
    std::array<double, 2> COP = { 0.5, 0.25 };
    std::vector<std::array<double, 2>> controlPoints = { {0, 0}, {10, 20} };
    DistortionRadialLERP radialLERP(COP, controlPoints);
    std::array<double, 3> point = { COP[0], COP[1], -1};
    if (!isNear(radialLERP.MapPoint(point), point)) {
      return "DistortionRadialLERP: 2x Unit radial point not returned unchanged";
    }
    point = { 2*COP[0], 2*COP[1], -2 };
    if (!isNear(radialLERP.MapPoint(point), point)) {
      return "DistortionRadialLERP: 2x Radial point not returned unchanged at Z = 2";
    }
    for (double x = -2; x <= 2; x += 0.1) {
      for (double y = -2; y <= 2; y += 0.1) {
        point = { x, y, -2 };
        std::array<double, 3> expected = { (x-2*COP[0]) * 2 + 2*COP[0], (y-2*COP[1]) * 2 + 2*COP[1], -2 };
        if (!isNear(radialLERP.MapPoint(point), expected)) {
          return "DistortionRadialLERP: 2x Point not changed as expected at Z = 2";
        }
      }
    }
    point = { -100, -100, -2 };
    if (!isNear(radialLERP.MapPoint(point), point)) {
      return "DistortionRadialLERP: 2x Far point not returned unchanged at Z = 2";
    }
  }
  {
    // Test a factor-of-3 distortion with a large number of interpolated points.
    std::array<double, 2> COP = { 0.5, -0.5 };
    std::vector<std::array<double, 2>> controlPoints;
    for (double i = 0; i < 10; i += 0.01) {
      controlPoints.push_back({ i, i * 3 });
    }
    DistortionRadialLERP radialLERP(COP, controlPoints);
    std::array<double, 3> point;
    for (double x = -2; x <= 2; x += 0.1) {
      for (double y = -2; y <= 2; y += 0.1) {
        point = { x, y, -2 };
        std::array<double, 3> expected = { (x - 2*COP[0]) * 3 + 2*COP[0], (y - 2*COP[1]) * 3 + 2*COP[1], -2};
        if (!isNear(radialLERP.MapPoint(point), expected)) {
          return "DistortionRadialLERP: 2x large count Point not changed as expected at Z = 2";
        }
      }
    }
    point = { -100, -100, -2 };
    if (!isNear(radialLERP.MapPoint(point), point)) {
      return "DistortionRadialLERP: 2x large count Far point not returned unchanged at Z = 2";
    }
  }

  return "";
}

