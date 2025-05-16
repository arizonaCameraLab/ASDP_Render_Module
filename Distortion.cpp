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

/// @brief Spatial query acceleration structure to determine which triangles are near a given location.
static class NearbyTriangles {
public:

  /// @brief Constructor
  /// @param delaunay The Delaunay triangulation to use for the spatial queries.
  /// @param binsPerSide The number of bins to use for the spatial queries.
  NearbyTriangles(GEO::Delaunay_var const& delaunay, size_t binsPerSide = 10)
    : m_binsPerSide(binsPerSide)
    , m_minX(0), m_maxX(0), m_minY(0), m_maxY(0)
  {
    if (delaunay->nb_cells() == 0 || binsPerSide == 0) {
      return;
    }

    // Determine the bounding box for all of the points in the Delaunay triangulation.
    m_minX = m_maxX = delaunay->vertex_ptr(0)[0];
    m_minY = m_maxY = delaunay->vertex_ptr(0)[1];
    for (GEO::index_t i = 1; i < delaunay->nb_vertices(); ++i) {
      double x = delaunay->vertex_ptr(i)[0];
      double y = delaunay->vertex_ptr(i)[1];
      if (x < m_minX) { m_minX = x; }
      if (x > m_maxX) { m_maxX = x; }
      if (y < m_minY) { m_minY = y; }
      if (y > m_maxY) { m_maxY = y; }
    }

    // Construct the (initially empty) bins.
    m_bins.resize(m_binsPerSide);
    for (size_t i = 0; i < m_binsPerSide; ++i) {
      m_bins[i].resize(m_binsPerSide);
    }

    // Fill each triangle into the bins that its vertices and center fall into along with the bins
    // to either side in X and Y.
    for (GEO::index_t t = 0; t < delaunay->nb_cells(); ++t) {

      // Get the vertices of the triangle
      GEO::index_t v0 = delaunay->cell_vertex(t, 0);
      GEO::index_t v1 = delaunay->cell_vertex(t, 1);
      GEO::index_t v2 = delaunay->cell_vertex(t, 2);

      // Get the center of the triangle
      double x = (delaunay->vertex_ptr(v0)[0] + delaunay->vertex_ptr(v1)[0] + delaunay->vertex_ptr(v2)[0]) / 3;
      double y = (delaunay->vertex_ptr(v0)[1] + delaunay->vertex_ptr(v1)[1] + delaunay->vertex_ptr(v2)[1]) / 3;

      // Find the bins for each vertex and the center
      std::array<size_t, 2> bin0 = BinForPoint(delaunay->vertex_ptr(v0)[0], delaunay->vertex_ptr(v0)[1]);
      std::array<size_t, 2> bin1 = BinForPoint(delaunay->vertex_ptr(v1)[0], delaunay->vertex_ptr(v1)[1]);
      std::array<size_t, 2> bin2 = BinForPoint(delaunay->vertex_ptr(v2)[0], delaunay->vertex_ptr(v2)[1]);
      std::array<size_t, 2> binC = BinForPoint(x, y);

      // Add the triangle to the bins
      AddTriangle(t, bin0);
      AddTriangle(t, bin1);
      AddTriangle(t, bin2);
      AddTriangle(t, binC);
    }
  }

  /// @brief Get the set of triangles that are near a given point.
  /// @param x The X coordinate of the point.
  /// @param y The Y coordinate of the point.
  /// @return A set of triangle indices that are near the point.
  std::set<GEO::index_t> GetTriangles(double x, double y) const
  {
    // If there are no bins, return an empty set
    if (m_bins.size() == 0) {
      return std::set<GEO::index_t>();
    }

    // Find the bin for the point
    std::array<size_t, 2> bin = BinForPoint(x, y);

    // Return the triangles in that bin
    return m_bins[bin[0]][bin[1]];
  }

protected:
  /// @brief Determine which bin a given point falls inside.
  std::array<size_t, 2> BinForPoint(double x, double y) const
  {
    // Find the coordinates of the bin
    int xi = static_cast<int>((x - m_minX) / (m_maxX - m_minX) * m_binsPerSide);
    int yi = static_cast<int>((y - m_minY) / (m_maxY - m_minY) * m_binsPerSide);

    // Clamp the bin to the valid range
    if (xi < 0) { xi = 0; }
    if (xi >= static_cast<int>(m_binsPerSide)) { xi = m_binsPerSide - 1; }
    if (yi < 0) { yi = 0; }
    if (yi >= static_cast<int>(m_binsPerSide)) { yi = m_binsPerSide - 1; }

    std::array<size_t, 2> bin = { xi, yi };
    return bin;
  }

  /// @brief Add a triangle to the specified bin and to the bins to either side in X and Y.
  /// @param t The index of the triangle to add.
  /// @param bin The bin to add the triangle to.
  void AddTriangle(GEO::index_t t, std::array<size_t, 2> const& bin)
  {
    m_bins[bin[0]][bin[1]].insert(t);
    if (bin[0] > 0) { m_bins[bin[0] - 1][bin[1]].insert(t); }
    if (bin[0] < m_binsPerSide - 1) { m_bins[bin[0] + 1][bin[1]].insert(t); }
    if (bin[1] > 0) { m_bins[bin[0]][bin[1] - 1].insert(t); }
    if (bin[1] < m_binsPerSide - 1) { m_bins[bin[0]][bin[1] + 1].insert(t); }
  }

  size_t m_binsPerSide; ///< Number of bins per side of the bounding box

  double m_minX; ///< Minimum X coordinate of the bounding box
  double m_minY; ///< Minimum Y coordinate of the bounding box
  double m_maxX; ///< Maximum X coordinate of the bounding box
  double m_maxY; ///< Maximum Y coordinate of the bounding box

  /// @brief Vector of bins in X by bins in Y by indices of triangles in this bin.
  std::vector< std::vector < std::set<GEO::index_t> > > m_bins;
};

class DistortionBagOfMappings::DistortionBagOfMappings_impl {
public:
  DistortionBagOfMappings_impl(Bag const& mappings);
  ~DistortionBagOfMappings_impl() = default;

  /// @brief Keeps track of all of the points that were passed into the constructor.
  Bag m_bag;

  /// @brief Map from "from" points to "to" points required because Delaunay triangulation can reorder points
  std::map<Point2D, Point2D> m_map;

  /// @brief The delaunay triangulation.
  GEO::Delaunay_var m_delaunay = nullptr;

  /// @brief Stored vector of "from" points used to construct the triangulation.
  std::vector<double> m_points;

  /// @brief Spatial query acceleration structure to determine which triangles are near a given location.
  std::shared_ptr<NearbyTriangles> m_nearbyTriangles = nullptr;
};

DistortionBagOfMappings::DistortionBagOfMappings_impl::DistortionBagOfMappings_impl(Bag const& mappings)
  : m_bag(mappings)
{
  try {
    // Make sure that Geogram is initialized; okay to call more than once.
    GEO::initialize();

    // Create an empty Delaunay triangulation
    m_delaunay = GEO::Delaunay::create(2, "BDEL2d");

    // Configure the triangulation to be deterministic, thread-safe, and to not add external infinite regions.
    m_delaunay->set_reorder(false);
    m_delaunay->set_thread_safe(true);
    m_delaunay->set_keeps_infinite(false);
  }
  catch (std::exception const &e) {
    throw std::runtime_error("DistortionBagOfMappings: Error initializing Delaunay triangulation: "
      + std::string(e.what()));
  }

  if (m_bag.size() == 0) {
    return;
  }

  // Create a list of "from" points to use to generate the Delaunay triangulation.
  m_points.reserve(m_bag.size() * 2);
  for (auto const& mapping : m_bag) {
    m_points.push_back(mapping[0][0]);
    m_points.push_back(mapping[0][1]);
  }

  // Create a Delaunay triangulation in 2D from the "from" points in the mapping.
  m_delaunay->set_vertices(m_bag.size(), m_points.data());

  /*
  // Write an OBJ file for debugging purposes that has all of the triangles in the Delaunay triangulation.
  static int ID = 0;
  std::ofstream objFile("XXX_delaunay" + std::to_string(ID++)+".obj");
  if (objFile.is_open()) {
    // Write the vertices
    for (size_t i = 0; i < m_delaunay->nb_vertices(); ++i) {
      objFile << "v " << m_delaunay->vertex_ptr(i)[0] << " " << m_delaunay->vertex_ptr(i)[1] << " 0" << std::endl;
    }
    // Write the triangles
    for (GEO::index_t t = 0; t < m_delaunay->nb_cells(); ++t) {
      objFile << "f";
      for (GEO::index_t j = 0; j < 3; ++j) {
        objFile << " " << m_delaunay->cell_vertex(t, j) + 1;
      }
      objFile << std::endl;
    }
    objFile.close();
  } else {
    std::cerr << "DistortionBagOfMappings: Unable to open Delaunay triangulation file for writing." << std::endl;
  }
  */

  // Create a mapping from the "from" points to the "to" points in the mapping
  // so that we can look them up after the points are re-ordered.
  for (size_t i = 0; i < m_bag.size(); ++i) {
    m_map[m_bag[i][0]] = m_bag[i][1];
  }

  // Create a spatial query acceleration structure to determine which triangles are near a given location.
  m_nearbyTriangles = std::make_shared<NearbyTriangles>(m_delaunay, 5);
}

DistortionBagOfMappings::DistortionBagOfMappings(Bag const& mappings)
  : m_impl(std::make_shared<DistortionBagOfMappings::DistortionBagOfMappings_impl>(mappings))
{
}

std::array<double, 3> DistortionBagOfMappings::BarycentricCoordinates(const Point2D& p,
  const Point2D& a, const Point2D& b, const Point2D& c)
{
  // Calculate 2x the area of the triangle formed by a, b, and c
  double doubleArea = (-b[1] * c[0] + a[1] * (-b[0] + c[0]) + a[0] * (b[1] - c[1]) + b[0] * c[1]);
  if (std::abs(doubleArea) < 1e-8) {
    // The points are collinear, so we can't interpolate. Return coordinate slightly outside of the triangle.
    return { 0.5, 0.4, -0.1 };
  }

  // Calculate the barycentric coordinates of point p
  double s = 1 / (doubleArea) * (a[1] * c[0] - a[0] * c[1] + (c[1] - a[1]) * p[0] + (a[0] - c[0]) * p[1]);
  double t = 1 / (doubleArea) * (a[0] * b[1] - a[1] * b[0] + (a[1] - b[1]) * p[0] + (b[0] - a[0]) * p[1]);
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

  // Interpolate/extrapolate the value using the barycentric coordinates
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
  std::array<double, 2> ptInPlane = { point[0] * scale, point[1] * scale };

  // Inverse scale so we can multiply by it rather than divide by it, speeding up the calculation.
  double invScale = -point[2];

  // Find the triangle that we're going to use to determine the Barycentric coordinates from.
  // Start by seeing if the point is inside any of the triangles in the Delaunay triangulation.
  // If it is, use that triangle.
  // Otherwise, find the triangle whose center is closet to the point.
  // Search only the triangles that are near the point using the spatial query acceleration structure.
  GEO::Delaunay_var& d = m_impl->m_delaunay;
  double min_distance2 = std::numeric_limits<double>::max();
  GEO::index_t closest_triangle = 0;
  std::set<GEO::index_t> triangles = m_impl->m_nearbyTriangles->GetTriangles(ptInPlane[0], ptInPlane[1]);
  Point2D A, B, C, center;
  for (auto t : triangles) {
    // Get the vertices of the triangle
    double const* v;
    v = d->vertex_ptr(d->cell_vertex(t, 0));
    A = { v[0], v[1] };
    v = d->vertex_ptr(d->cell_vertex(t, 1));
    B = { v[0], v[1] };
    v = d->vertex_ptr(d->cell_vertex(t, 2));
    C = { v[0], v[1] };

    // If we're inside this triangle, then we use it.
    if (IsPointInTriangle(ptInPlane, A, B, C)) {
      closest_triangle = t;
      min_distance2 = 0;
      break;
    }

    // Skip degenerate triangles that have small areas
    double area = (-B[1] * C[0] + A[1] * (-B[0] + C[0]) + A[0] * (B[1] - C[1]) + B[0] * C[1]);
    if (std::abs(area) < 1e-3) {
      continue;
    }

    // Compute the average of the three vertices and find the squared distance (faster
    // to compute than the distance and still monotonically increasing) from the
    // point to this center; use it to determine whether this triangle is closest
    // in case we're not inside any triangle.
    center = { (A[0] + B[0] + C[0]) / 3.0, (A[1] + B[1] + C[1]) / 3.0 };
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
  A = { v[0], v[1] };
  v = d->vertex_ptr(d->cell_vertex(closest_triangle, 1));
  B = { v[0], v[1] };
  v = d->vertex_ptr(d->cell_vertex(closest_triangle, 2));
  C = { v[0], v[1] };

  // Use the map to find the corresponding points in the "to" triangle.
  Point2D const& Ato = m_impl->m_map[A];
  Point2D const& Bto = m_impl->m_map[B];
  Point2D const& Cto = m_impl->m_map[C];

  // Use the "to" mapping X and Y coordinates based on weighting the Barycentric coordinates
  // from the triangle and applying them individually to the two indices in the "to" triangle.
  double xD = DetermineValue(A, Ato[0], B, Bto[0], C, Cto[0], ptInPlane);
  double yD = DetermineValue(A, Ato[1], B, Bto[1], C, Cto[1], ptInPlane);

  // Rescale the point and put it back onto same plane as the original point.
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
    /// Check NearbyTriangles
    {
      GEO::initialize();
      GEO::Delaunay_var delaunay = GEO::Delaunay::create(2, "BDEL2d");
      std::vector<std::array<double, 2>> points = { {0, 0}, {1, 0}, {0, 1}, {1, 1} };
      delaunay->set_vertices(points.size(), points[0].data());
      NearbyTriangles nearby(delaunay, 2);
      std::set<GEO::index_t> triangles = nearby.GetTriangles(0.5, 0.5);
      if (triangles.size() != 2) {
        return "DistortionBagOfMappings: NearbyTriangles failed for 0.5, 0.5";
      }
      triangles = nearby.GetTriangles(0,0);
      if (triangles.size() != 2) {
        return "DistortionBagOfMappings: NearbyTriangles failed for 0, 0";
      }
    }

    /// Check BarycentricCoordinates:
    {
      DistortionBagOfMappings::Point2D a = { 12.0, 3.0 };
      DistortionBagOfMappings::Point2D b = { 1.0, 0.0 };
      DistortionBagOfMappings::Point2D c = { 0.0, 1.0 };
      std::array<double, 3> coords = DistortionBagOfMappings::BarycentricCoordinates(a, a, b, c);
      if (std::abs(coords[0] - 1) > 1e-8) {
        return "DistortionBagOfMappings: Barycentric coordinates are not correct for point 1";
      }
      coords = DistortionBagOfMappings::BarycentricCoordinates(b, a, b, c);
      if (std::abs(coords[1] - 1) > 1e-8) {
        return "DistortionBagOfMappings: Barycentric coordinates are not correct for point 2";
      }
      coords = DistortionBagOfMappings::BarycentricCoordinates(c, a, b, c);
      if (std::abs(coords[2] - 1) > 1e-8) {
        return "DistortionBagOfMappings: Barycentric coordinates are not correct for point 3";
      }
    }

    // Check IsPointInTriangle:
    {
      DistortionBagOfMappings::Point2D a = { 0.0, 0.0 };
      DistortionBagOfMappings::Point2D b = { 1.0, 0.0 };
      DistortionBagOfMappings::Point2D c = { 0.0, 1.0 };
      if (!DistortionBagOfMappings::IsPointInTriangle(a, a, b, c)) {
        return "DistortionBagOfMappings: Point A is not in triangle";
      }
      if (!DistortionBagOfMappings::IsPointInTriangle(b, a, b, c)) {
        return "DistortionBagOfMappings: Point B is not in triangle";
      }
      if (!DistortionBagOfMappings::IsPointInTriangle(c, a, b, c)) {
        return "DistortionBagOfMappings: Point C is not in triangle";
      }

      if (!DistortionBagOfMappings::IsPointInTriangle({ 0.25, 0.25 }, a, b, c)) {
        return "DistortionBagOfMappings: Point inside triangle is not in triangle";
      }
      if (DistortionBagOfMappings::IsPointInTriangle({ 1.0, 1.0 }, a, b, c)) {
        return "DistortionBagOfMappings: Point outside triangle is in triangle";
      }
    }

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
