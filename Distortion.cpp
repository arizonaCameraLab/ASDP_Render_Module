/*
 * Copyright (C) 2024-2025: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include <cmath>
#include <iostream>
#include <map>
#include <Distortion.h>
#include <geogram/basic/common.h>
#include <geogram/mesh/mesh.h>
#include <geogram/delaunay/delaunay.h>
using namespace asdp::render;

//==============================================================================
// Radial LERP distortion class

DistortionRadialLERP::DistortionRadialLERP(std::array<double, 2> const& COP, std::vector<std::array<double, 2>> const& controlPoints)
  : m_COP(COP), m_controlPoints(controlPoints)
{
  if (m_controlPoints.size() < 2) {
    std::cerr << "DistortionRadialLERP: Not enough control points: " << std::to_string(m_controlPoints.size()) << std::endl;
  }
  if (m_controlPoints.front()[0] != 0) {
    std::cerr << "DistortionRadialLERP: First control point distance must be zero: " << std::to_string(m_controlPoints.front()[0]) << std::endl;
  }
}

std::array<double, 3> DistortionRadialLERP::MapPoint(std::array<double, 3> point) const
{
  // If we don't have enough control points, return the point unchanged
  if (m_controlPoints.size() < 2) {
    return point;
  }

  // If the point is on the Z = 0 plane, return the point unchanged
  if (point[2] == 0) {
    return point;
  }

  // Calculate the vector from the center of projection to the point.
  // Project the vector onto the Z = -1 plane (scales it to Z = -1)
  double scale = -1 / point[2];
  // Inverse scale so we can multiply by it rather than divide by it, speeding up the calculation.
  double invScale = 1 / scale;
  // Map the center of projection from the Z=-1 plane to the plane that includes the point, then
  // find the vector in that plane from the center of projection to the point.
  std::array<double, 2> vec = {point[0] - m_COP[0]*invScale, point[1] - m_COP[1]*invScale};
  // Scale the vector to the Z = -1 plane (now centered on the origin specified by the COP).
  vec[0] *= scale;
  vec[1] *= scale;

  // If the distance is before the first control point or after the last control point, return the point unchanged
  double dist = std::sqrt(vec[0] * vec[0] + vec[1] * vec[1]);
  if (dist < m_controlPoints.front()[0] || dist > m_controlPoints.back()[0]) {
    return point;
  }

  // Find the control points that bound the distance
  size_t i = 0;
  while (i < (m_controlPoints.size() - 1) && m_controlPoints[i + 1][0] < dist) {
    i++;
  }

  // Interpolate between the control points
  double t = (dist - m_controlPoints[i][0]) / (m_controlPoints[i + 1][0] - m_controlPoints[i][0]);
  double d = m_controlPoints[i][1] * (1 - t) + m_controlPoints[i + 1][1] * t;

  // Scale the vector by both the inverse of the original scaling and the ratio of the interpolated distance to the
  // original and add it to the scaled center of projection.  Handle the case where the point is right at the
  // center of projection.
  double reScale = invScale;
  if (dist > 0) {
    reScale *= (d / dist);
  }

  return {m_COP[0]*invScale + vec[0] * reScale, m_COP[1]*invScale + vec[1] * reScale, point[2]};
}


//==============================================================================
// Bag of mappings distortion class

DistortionBagOfMappings::DistortionBagOfMappings(Bag const& mappings)
  : m_mappings(mappings)
  , m_rangeX({0, 0})
  , m_rangeY({0, 0})
{
  // Construct the grid to use when accelerating mappings.  Do this by determining the
  // range of the mappings and dividing that range into a grid.  Then fill each mapping into
  // all cells within 1/4 of the grid distance from its location.  Each grid cell is a bag
  // of mappings.
  if (mappings.size() > 0) {
    // Find the range of the mappings
    double minX = mappings[0][0][0];
    double maxX = mappings[0][0][0];
    double minY = mappings[0][0][1];
    double maxY = mappings[0][0][1];
    for (auto const& mapping : mappings) {
      if (mapping[0][0] < minX) {
        minX = mapping[0][0];
      }
      if (mapping[0][0] > maxX) {
        maxX = mapping[0][0];
      }
      if (mapping[0][1] < minY) {
        minY = mapping[0][1];
      }
      if (mapping[0][1] > maxY) {
        maxY = mapping[0][1];
      }
    }
    m_rangeX = { minX, maxX };
    m_rangeY = { minY, maxY };

    // Fill each mapping into the subset of the grid that is within 1/4 of the grid distance (+/- 1/8).
    size_t quarterX = m_numSamplesX / 8;
    size_t quarterY = m_numSamplesY / 8;
    for (auto const& mapping : mappings) {
      size_t x = static_cast<size_t>(round((mapping[0][0] - minX) / (maxX - minX) * (m_numSamplesX-1)));
      size_t y = static_cast<size_t>(round((mapping[0][1] - minY) / (maxY - minY) * (m_numSamplesY-1)));
      for (size_t i = x > quarterX ? x - quarterX : 0; i < m_numSamplesX && i < x + quarterX; i++) {
        for (size_t j = y > quarterY ? y - quarterY : 0; j < m_numSamplesY && j < y + quarterY; j++) {
          m_grid[i][j].push_back(mapping);
        }
      }
    }
  }
}

bool DistortionBagOfMappings::NearlyCollinear(DistortionBagOfMappings::Point2D const& p1,
  DistortionBagOfMappings::Point2D const& p2,
  DistortionBagOfMappings::Point2D const& p3)
{
  double dx1 = p2[0] - p1[0];
  double dy1 = p2[1] - p1[1];
  double dx2 = p3[0] - p1[0];
  double dy2 = p3[1] - p1[1];
  double len1 = sqrt(dx1 * dx1 + dy1 * dy1);
  double len2 = sqrt(dx2 * dx2 + dy2 * dy2);

  // If either vector is zero length, they are collinear
  if (len1 * len2 == 0) {
    return true;
  }

  // Normalize the vectors
  double invLen1 = 1 / len1;
  double invLen2 = 1 / len2;
  dx1 *= invLen1;
  dy1 *= invLen1;
  dx2 *= invLen2;
  dy2 *= invLen2;

  // See if the magnitude of their dot products is close to 1.
  double dot = dx1 * dx2 + dy1 * dy2;
  return fabs(dot) > 0.8;
}

DistortionBagOfMappings::Bag DistortionBagOfMappings::FindThreeNearestPointsInBag(Point2D const &p, Bag const &points)
{
  DistortionBagOfMappings::Bag ret;

  // Make sure we have three points. If not, return an empty bag.
  if (points.size() < 3) {
    return ret;
  }

  // Find the three non-collinear points in the mesh that are nearest
  // to the normalized point we are trying to look up.  We start by
  // sorting the points based on distance from our location, selecting
  // the first two, and then looking through the rest until we find
  // one that is not collinear with the first two (normalized dot
  // product magnitude far enough from 1).  If we don't find such
  // points, we just go with the values from the closest point.
  typedef std::multimap<double, size_t> PointDistanceIndexMap;
  PointDistanceIndexMap map;

  for (size_t i = 0; i < points.size(); i++) {
    // Insertion into the multimap sorts them by distance.
    map.insert(std::make_pair(PointDistance(p, points[i][0]), i));
  }

  PointDistanceIndexMap::const_iterator it = map.begin();
  size_t first = it->second;
  it++;
  size_t second = it->second;
  it++;
  // In case we don't find a third point, re-use the first. We check for that later and only return two points
  // if this happens.
  size_t third = first;
  while (it != map.end()) {
    if (!NearlyCollinear(points[first][0], points[second][0], points[it->second][0])) {
      third = it->second;
      break;
    }
    it++;
  }

  // Push back all of the points we found, which may not include
  // a third point if the first is the same as the third.
  ret.push_back(points[first]);
  ret.push_back(points[second]);
  if (third != first) {
    ret.push_back(points[third]);
  }

  return ret;
}

double DistortionBagOfMappings::DetermineValue(Point2D const& p1, double v1,
  Point2D const& p2, double v2,
  Point2D const& p3, double v3,
  Point2D const& p)
{
  // Calculate the area of the triangle formed by p1, p2, and p3
  double area = 0.5 * (-p2[1] * p3[0] + p1[1] * (-p2[0] + p3[0]) + p1[0] * (p2[1] - p3[1]) + p2[0] * p3[1]);
  if (std::abs(area) < 1e-8) {
    // The points are collinear, so we can't interpolate.
    return v1;
  }

  // Calculate the barycentric coordinates of point p
  double s = 1 / (2 * area) * (p1[1] * p3[0] - p1[0] * p3[1] + (p3[1] - p1[1]) * p[0] + (p1[0] - p3[0]) * p[1]);
  double t = 1 / (2 * area) * (p1[0] * p2[1] - p1[1] * p2[0] + (p1[1] - p2[1]) * p[0] + (p2[0] - p1[0]) * p[1]);
  double u = 1 - s - t;

  // Interpolate the value using the barycentric coordinates
  return s * v2 + t * v3 + u * v1;
}

std::array<double, 3> DistortionBagOfMappings::MapPoint(std::array<double, 3> point) const
{
  // If we don't have any mappings, return the point unchanged
  if (m_mappings.size() < 0) {
    return point;
  }

  // If the point is on the Z = 0 plane, return the point unchanged
  if (point[2] == 0) {
    return point;
  }

  // Project the vector onto the Z = -1 plane (scales it to Z = -1)
  double scale = -1 / point[2];
  // Inverse scale so we can multiply by it rather than divide by it, speeding up the calculation.
  std::array<double, 2> ptInPlane = { point[0] * scale, point[1] * scale };

  // Find the closest three non-collinear points in the grid, first by checking the cell that the point is in
  // and then the whole grid if necessary.  If there are not enough points, return the point unchanged.
  size_t x = static_cast<size_t>(round((ptInPlane[0] - m_rangeX[0]) / (m_rangeX[1] - m_rangeX[0]) * (m_numSamplesX - 1)));
  size_t y = static_cast<size_t>(round((ptInPlane[1] - m_rangeY[0]) / (m_rangeY[1] - m_rangeY[0]) * (m_numSamplesY - 1)));
  x = std::max(std::min(x, m_numSamplesX - 1), static_cast<size_t>(0));
  y = std::max(std::min(y, m_numSamplesY - 1), static_cast<size_t>(0));
  Bag three = FindThreeNearestPointsInBag(ptInPlane, m_grid[x][y]);
  if (three.size() < 3) {
    three = FindThreeNearestPointsInBag(ptInPlane, m_mappings);
  }
  if (three.size() < 3) {
    return point;
  }

  // Interpolate or extrapolate the distortion location based on the three points.
  // We pass in the 2D coordinates of the undistorted point and then the X coordinate of the distorted point.
  // We then do the same for the Y coordiante of the distorted point.
  double xD = DetermineValue(three[0][0], three[0][1][0], three[1][0], three[1][1][0], three[2][0], three[2][1][0], ptInPlane);
  double yD = DetermineValue(three[0][0], three[0][1][1], three[1][0], three[1][1][1], three[2][0], three[2][1][1], ptInPlane);

  // Rescale the point and put it back onto same plane as the original point.
  double invScale = 1 / scale;
  return { xD * invScale, yD * invScale, point[2] };
}

//==============================================================================
// Test and its helper functions

static bool isNear(std::array<double, 3> const& a, std::array<double, 3> const& b, double epsilon = 1e-6)
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
      return "DistortionRadialLERP: Radial point not returned unchanged at Z = -2";
    }
    for (double x = -2; x <= 2; x += 0.1) {
      for (double y = -2; y <= 2; y += 0.1) {
        point = {x, y, -2};
        if (!isNear(radialLERP.MapPoint(point), point)) {
          return "DistortionRadialLERP: Point not returned unchanged at Z = -2";
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
      return "DistortionRadialLERP: 2x Radial point not returned unchanged at Z = -2";
    }
    for (double x = -2; x <= 2; x += 0.1) {
      for (double y = -2; y <= 2; y += 0.1) {
        point = { x, y, -2 };
        std::array<double, 3> expected = { (x-2*COP[0]) * 2 + 2*COP[0], (y-2*COP[1]) * 2 + 2*COP[1], -2 };
        if (!isNear(radialLERP.MapPoint(point), expected)) {
          return "DistortionRadialLERP: 2x Point not changed as expected at Z = -2";
        }
      }
    }
    point = { -100, -100, -2 };
    if (!isNear(radialLERP.MapPoint(point), point)) {
      return "DistortionRadialLERP: 2x Far point not returned unchanged at Z = -2";
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
          return "DistortionRadialLERP: 2x large count Point not changed as expected at Z = -2";
        }
      }
    }
    point = { -100, -100, -2 };
    if (!isNear(radialLERP.MapPoint(point), point)) {
      return "DistortionRadialLERP: 2x large count Far point not returned unchanged at Z = -2";
    }
  }

  // Test PointDistance
  {
    DistortionBagOfMappings::Point2D p1 = { 0, 0 };
    DistortionBagOfMappings::Point2D p2 = { 1, 0 };
    double dist = DistortionBagOfMappings::PointDistance(p1, p2);
    if (dist != 1) {
      return "DistortionBagOfMappings: PointDistance failed for X=1";
    }
    p2 = { 0, 1 };
    dist = DistortionBagOfMappings::PointDistance(p1, p2);
    if (dist != 1) {
      return "DistortionBagOfMappings: PointDistance failed for Y=1";
    }
    p2 = { 3, 4 };
    dist = DistortionBagOfMappings::PointDistance(p1, p2);
    if (dist != 5) {
      return "DistortionBagOfMappings: PointDistance failed for 3,4";
    }
  }

  // Test NearlyCollinear
  {
    DistortionBagOfMappings::Point2D p1 = { 0, 0 };
    DistortionBagOfMappings::Point2D p2 = { 1, 0 };
    DistortionBagOfMappings::Point2D p3 = { 2, 0 };
    if (!DistortionBagOfMappings::NearlyCollinear(p1, p2, p3)) {
      return "DistortionBagOfMappings: NearlyCollinear failed for 2,0";
    }
    p3 = { 2, 2 };
    if (DistortionBagOfMappings::NearlyCollinear(p1, p2, p3)) {
      return "DistortionBagOfMappings: NearlyCollinear failed for 2,2";
    }
    p3 = { 2, 3 };
    if (DistortionBagOfMappings::NearlyCollinear(p1, p2, p3)) {
      return "DistortionBagOfMappings: NearlyCollinear failed for 2,3";
    }
    p3 = { 10, 1 };
    if (!DistortionBagOfMappings::NearlyCollinear(p1, p2, p3)) {
      return "DistortionBagOfMappings: NearlyCollinear failed for 10,1";
    }
  }

  // Test FindThreeNearestPointsInBag
  {
    DistortionBagOfMappings::Point2D p = { 0, 0 };
    DistortionBagOfMappings::Point2D p1 = { 0, 0 };
    DistortionBagOfMappings::Point2D mp1 = { 1, 1 };
    DistortionBagOfMappings::Point2D p2 = { 1, 0 };
    DistortionBagOfMappings::Point2D mp2 = { 2, 2 };
    DistortionBagOfMappings::Point2D p3 = { 2, 2 };
    DistortionBagOfMappings::Point2D mp3 = { 3, 3 };
    DistortionBagOfMappings::Mapping m1 = { p1, mp1 };
    DistortionBagOfMappings::Mapping m2 = { p2, mp2 };
    DistortionBagOfMappings::Mapping m3 = { p3, mp3 };
    DistortionBagOfMappings::Bag points = { m1, m2, m3 };
    DistortionBagOfMappings::Bag three = DistortionBagOfMappings::FindThreeNearestPointsInBag(p, points);
    if (three.size() != 3) {
      return "DistortionBagOfMappings: FindThreeNearestPointsInBag failed for 0,0";
    }
    if (three[0][0] != points[0][0] || three[1][0] != points[1][0] || three[2][0] != points[2][0]) {
      return "DistortionBagOfMappings: FindThreeNearestPointsInBag failed for 0,0 points";
    }
    p = { 0.6, 0.6 };
    three = DistortionBagOfMappings::FindThreeNearestPointsInBag(p, points);
    if (three.size() != 3) {
      return "DistortionBagOfMappings: FindThreeNearestPointsInBag failed for 0.6,0.6";
    }
    if (three[0][0] != points[1][0] || three[1][0] != points[0][0] || three[2][0] != points[2][0]) {
      return "DistortionBagOfMappings: FindThreeNearestPointsInBag failed for 0.6,0.6 points";
    }
    points = { m1, m2 };
    three = DistortionBagOfMappings::FindThreeNearestPointsInBag(p, points);
    if (three.size() != 0) {
      return "DistortionBagOfMappings: FindThreeNearestPointsInBag failed for size 2";
    }
  }

  // Test DetermineValue
  {
    DistortionBagOfMappings::Point2D p1 = { 0, 0 };
    DistortionBagOfMappings::Point2D p2 = { 1, 0 };
    DistortionBagOfMappings::Point2D p3 = { 0, 1 };
    double v1 = 0;
    double v2 = 1;
    double v3 = 2;
    DistortionBagOfMappings::Point2D p = p1;
    double v = DistortionBagOfMappings::DetermineValue(p1, v1, p2, v2, p3, v3, p);
    if (v != v1) {
      return "DistortionBagOfMappings: DetermineValue failed for p1: " + std::to_string(v);
    }
    p = p2;
    v = DistortionBagOfMappings::DetermineValue(p1, v1, p2, v2, p3, v3, p);
    if (v != v2) {
      return "DistortionBagOfMappings: DetermineValue failed for p2: " + std::to_string(v);
    }
    p = p3;
    v = DistortionBagOfMappings::DetermineValue(p1, v1, p2, v2, p3, v3, p);
    if (v != v3) {
      return "DistortionBagOfMappings: DetermineValue failed for p3: " + std::to_string(v);
    }
    p = { 0.5, 0.0 };
    v = DistortionBagOfMappings::DetermineValue(p1, v1, p2, v2, p3, v3, p);
    if (v != 0.5) {
      return "DistortionBagOfMappings: DetermineValue failed for 0.5,0.0: " + std::to_string(v);
    }
    p = { 0.0, 0.5 };
    v = DistortionBagOfMappings::DetermineValue(p1, v1, p2, v2, p3, v3, p);
    if (v != 1.0) {
      return "DistortionBagOfMappings: DetermineValue failed for 0.0,0.5: " + std::to_string(v);
    }
    p = { 2.0, 0.0 };
    v = DistortionBagOfMappings::DetermineValue(p1, v1, p2, v2, p3, v3, p);
    if (v != 2.0) {
      return "DistortionBagOfMappings: DetermineValue failed for 2.0,0.0: " + std::to_string(v);
    }
  }

  // Test the DistortionBagOfMappings class.
  {
    // A mapping with an empty bag should always return the points unchanged.
    {
      DistortionBagOfMappings::Bag mappings;
      DistortionBagOfMappings distortion(mappings);
      std::array<double, 3> point = { 1, 2, 3 };
      if (!isNear(distortion.MapPoint(point), point)) {
        return "DistortionBagOfMappings: Empty mappings failed for 1,2,3";
      }
      point = { 0, 0, 1 };
      if (!isNear(distortion.MapPoint(point), point)) {
        return "DistortionBagOfMappings: Empty mappings failed for 0,0,1";
      }
    }

    // Make a mapping that doubles the distance for each point from the origin and ensure that this is the
    // behavior for both interpolation and extrapolation.
    {
      DistortionBagOfMappings::Bag mappings;
      for (double x = -10; x <= 10; x += 1) {
        for (double y = -10; y <= 10; y += 1) {
          DistortionBagOfMappings::Point2D point = { x, y };
          DistortionBagOfMappings::Point2D mapped = { 2 * x, 2 * y };
          mappings.push_back({ point, mapped });
        }
      }
      DistortionBagOfMappings distortion(mappings);
      std::array<double, 3> point = { 0, 0, 1 };
      if (!isNear(distortion.MapPoint(point), point)) {
        return "DistortionBagOfMappings: 2x mappings failed for 0,0,1";
      }
      for (double x = -15; x <= 15; x += 0.5) {
        for (double y = -15; y <= 15; y += 0.5) {
          if (!isNear(distortion.MapPoint({ x, y, 1 }), { 2 * x, 2 * y, 1 })) {
            return "DistortionBagOfMappings: 2x mappings failed for " + std::to_string(x) + "," + std::to_string(y) + ",1:"
              + " " + std::to_string(distortion.MapPoint({ x, y, 1 })[0])
              + "," + std::to_string(distortion.MapPoint({ x, y, 1 })[1])
              + "," + std::to_string(distortion.MapPoint({ x, y, 1 })[2]);
          }
        }
      }
    }
  }

  return "";
}
