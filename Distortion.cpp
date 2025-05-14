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

class DistortionBagOfMappings::DistortionBagOfMappings_impl {
public:
  DistortionBagOfMappings_impl(Bag const& mappings);
  ~DistortionBagOfMappings_impl() = default;

  /// @brief Keeps track of all of the points that were passed into the constructor.
  Bag m_bag;

  /// @brief Map from "from" points to "to" points required because Delaunay triangulation reorders points
  std::map<Point2D, Point2D> m_map;

  /// @brief The delaunay triangulation.
  GEO::Delaunay_var m_delaunay = nullptr;

  /// @brief Stored vector of "from" points used to construct the triangulation.
  std::vector<double> m_points;
};

DistortionBagOfMappings::DistortionBagOfMappings_impl::DistortionBagOfMappings_impl(Bag const& mappings)
  : m_bag(mappings)
{
  try {
    // Make sure that Geogram is initialized; okay to call more than once.
    GEO::initialize(GEO::GEOGRAM_INSTALL_HANDLERS);

    // Create an empty Delaunay triangulation
    m_delaunay = GEO::Delaunay::create(2, "BDEL2d");
    //m_delaunay->set_reorder(false);
    m_delaunay->set_thread_safe(true);
  }
  catch (std::exception const &e) {
    throw std::runtime_error("DistortionBagOfMappings: Error initializing Delaunay triangulation: "
      + std::string(e.what()));
  }

  if (m_bag.size() == 0) {
    return;
  }

  // Create a list of input points to use to generate the Delaunay triangulation.
  for (auto const& mapping : m_bag) {
    m_points.push_back(mapping[0][0]);
    m_points.push_back(mapping[0][1]);
  }

  // Create a Delaunay triangulation in 2D from the "from" points in the mapping.
  m_delaunay->set_vertices(m_bag.size(), m_points.data());

  // Create a mapping from the "from" points to the "to" points in the mapping.
  for (size_t i = 0; i < m_bag.size(); ++i) {
    m_map[m_bag[i][0]] = m_bag[i][1];
  }
}

DistortionBagOfMappings::DistortionBagOfMappings(Bag const& mappings)
  : m_impl(std::make_shared<DistortionBagOfMappings::DistortionBagOfMappings_impl>(mappings))
{
}

std::array<double, 3> DistortionBagOfMappings::BarycentricCoordinates(const Point2D& p,
  const Point2D& a, const Point2D& b, const Point2D& c)
{
  // Calculate the area of the triangle formed by p1, p2, and p3
  double area = 0.5 * (-b[1] * c[0] + a[1] * (-b[0] + c[0]) + a[0] * (b[1] - c[1]) + b[0] * c[1]);
  if (std::abs(area) < 1e-8) {
    // The points are collinear, so we can't interpolate. Return the coordinate of the first point.
    return { 1, 0, 0 };
  }

  // Calculate the barycentric coordinates of point p
  double s = 1 / (2 * area) * (a[1] * c[0] - a[0] * c[1] + (c[1] - a[1]) * p[0] + (a[0] - c[0]) * p[1]);
  double t = 1 / (2 * area) * (a[0] * b[1] - a[1] * b[0] + (a[1] - b[1]) * p[0] + (b[0] - a[0]) * p[1]);
  double u = 1 - s - t;

  // Return them in the order that allows you to interoplate points.
  return { u, s, t };
}

bool DistortionBagOfMappings::IsPointInTriangle(const Point2D& p,
  const Point2D& a, const Point2D& b, const Point2D& c)
{
  // Calculate the barycentric coordinates of the point p
  std::array<double, 3> coords = BarycentricCoordinates(p, a, b, c);

  return coords[0] >= 0 && coords[1] >= 0 && coords[2] >= 0;
}

double DistortionBagOfMappings::DetermineValue(Point2D const& p1, double v1,
  Point2D const& p2, double v2,
  Point2D const& p3, double v3,
  Point2D const& p)
{
  // Calculate the barycentric coordinates of the point p
  std::array<double, 3> coords = BarycentricCoordinates(p, p1, p2, p3);

  // Interpolate the value using the barycentric coordinates
  return coords[0] * v1 + coords[1] * v2 + coords[2] * v3;
}

std::array<double, 3> DistortionBagOfMappings::MapPoint(std::array<double, 3> point) const
{
  // If we don't have any mappings, return the point unchanged
  if (!m_impl || !m_impl->m_delaunay || m_impl->m_bag.size() == 0) {
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

  // Find the triangle that we're going to use to determine the Barycentric coordinates from.
  // Start by seeing if the point is inside any of the triangles in the Delaunay triangulation.
  // If it is, use that triangle.
  // Otherwise, find the triangle whose center is closet to the point.
  // This is a brute-force search, but it should be fast enough for most cases.
  GEO::Delaunay_var& d = m_impl->m_delaunay;
  double min_distance2 = std::numeric_limits<double>::max();
  GEO::index_t closest_triangle = 0;
  for (GEO::index_t t = 0; t < d->nb_finite_cells(); ++t) {
    // Get the vertices of the triangle
    double const* v;
    v = d->vertex_ptr(d->cell_vertex(t, 0));
    std::array<double, 2> A = { v[0], v[1] };
    v = d->vertex_ptr(d->cell_vertex(t, 1));
    std::array<double, 2> B = { v[0], v[1] };
    v = d->vertex_ptr(d->cell_vertex(t, 2));
    std::array<double, 2> C = { v[0], v[1] };

    // If we're inside this triangle, then we use it.
    if (IsPointInTriangle(ptInPlane, A, B, C)) {
      closest_triangle = t;
      min_distance2 = 0;
      break;
    }

    // Compute the average of the three vertices and find the squared distance (faster
    // to compute than the distance and still monotonically increasing) from the
    // point to this, use it to determine whether this triangle is closest
    // in case we're not inside any triangle.
    std::array<double, 2> center = { (A[0] + B[0] + C[0]) / 3.0, (A[1] + B[1] + C[1]) / 3.0 };
    double dx = center[0] - ptInPlane[0];
    double dy = center[1] - ptInPlane[1];
    double distance2 = dx * dx + dy * dy;
    if (distance2 < min_distance2) {
      closest_triangle = t;
      min_distance2 = distance2;
    }
  }

  // Find the vertices of the closest triangle.
  double const* v;
  v = d->vertex_ptr(d->cell_vertex(closest_triangle, 0));
  Point2D A = { v[0], v[1] };
  v = d->vertex_ptr(d->cell_vertex(closest_triangle, 1));
  Point2D B = { v[0], v[1] };
  v = d->vertex_ptr(d->cell_vertex(closest_triangle, 2));
  Point2D C = { v[0], v[1] };

  // Use the map to find the corresponding points in the "to" triangle.
  Point2D const& Ato = m_impl->m_map[A];
  Point2D const& Bto = m_impl->m_map[B];
  Point2D const& Cto = m_impl->m_map[C];

  // Use the "to" mapping X and Y coordinates based on weighting the Barcycentric coordinates
  // from the triangle and applying them individually to the two indices in the "to" triangle.
  double xD = DetermineValue(A, Ato[0], B, Bto[0], C, Cto[0], ptInPlane);
  double yD = DetermineValue(A, Ato[1], B, Bto[1], C, Cto[1], ptInPlane);

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
    p = { 0.0, -1.0 };
    v = DistortionBagOfMappings::DetermineValue(p1, v1, p2, v2, p3, v3, p);
    if (v != -2.0) {
      return "DistortionBagOfMappings: DetermineValue failed for -1.0,0.0: " + std::to_string(v);
    }
    p = { 1.0, 1.0 };
    v = DistortionBagOfMappings::DetermineValue(p1, v1, p2, v2, p3, v3, p);
    if (v != 3.0) {
      return "DistortionBagOfMappings: DetermineValue failed for 1.0,1.0: " + std::to_string(v);
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
