/*
 * Copyright (C) 2024-2026: Arizona Board of Regents on Behalf of the University of Arizona
 */

 /**
  * @file Distortion.h
  * @brief Apache Strap-Down Pilotage Render/Distortion class header file.
  *
  * @author ReliaSolve.
  * @date April 29, 2024.
  */

#pragma once

#include <cmath>
#include <array>
#include <vector>
#include <string>
#include <memory>

namespace asdp {
  namespace render {

    /// @brief Base class for distortion correction that defines the interface.
    class Distortion {
    public:

      /// @brief Virtual destructor so that proper deconstruction happens on pointers.
      virtual ~Distortion() = default;

      /// @brief Map a point from a projected sensor pixel location to its projected real-world location
      /// as seen by an ideal camera.
      /// @details This function moves points on a plane in front of an ideal camera based on a distortion
      /// map.  Different derived classes implement different distortion models with different parameters.
      /// In all cases, the distortion map will have been generated
      /// based on an ideal camera of known horizontal and vertical fields of view whose center of
      /// projection is the center of the image sensor.  The camera will be right-handed with its
      /// +X axis to the right in the image, its +Y axis pointing up in the image, looking down
      /// the -Z axis.  The center of projection for the camera is at the origin.  For example,
      /// the point (0, 0, -5) will project to the center of the image, 5 units away from the camera
      /// in an ideal camera's space.
      /// @param point Point in projected sensor pixel space, specified in ideal-camera space.  The point is given
      /// as (X, Y, Z) where Z is negative.  If an object is seen at a particular pixel in the image, the point
      /// would be the projection of that pixel into some -Z plane in the ideal camera's space.
      /// @return Projection of the actual location of the object, also in ideal camera space, onto the same -Z plane.
      /// The point will have the same Z value as the input point, but the X and Y values will be adjusted
      /// to account for the distortions.  When building a distortion mesh for rendering, a regular sampling of
      /// points in X and Y at a fixed Z value (e.g., Z = -200) would be fed into this function and the
      /// results used to build the deformed mesh that would make images appear undistorted with each pixel
      /// in the proper direction within the ideal camera space.
      /// Depending on the distortion model, the point may be shifted, scaled, or rotated.  It may
      /// lie outside of the field of view of the ideal camera.
      virtual std::array<double, 3> MapPoint(std::array<double, 3> point) const = 0;

      /// @brief Test function for all subclasses that returns an empty string on success or an error message on failure.
      static std::string Test();
    };

    /// @brief Null distortion model that returns the input point unchanged.
    class DistortionNone : public Distortion {
    public:

      std::array<double, 3> MapPoint(std::array<double, 3> point) const override { return point; }
    };

    /// @brief Piecewise linear distortion model that is radial from a center of projection.
    /// @details This distortion model is a piecewise linear model that is radial from a center of
    /// projection.  Linear interpolation is performed between control points to determine the
    /// distortion at any point.
    class DistortionRadialLERP : public Distortion {
    public:
      /// @brief Constructor that takes the center of projection and control points for the distortion.
      /// @param COP Center of projection for the distortion.  This (X,Y) is the location where the ray
      /// from the center of distortion on the image sensor intersects the Z = -1 plane in ideal camera space.
      /// For a camera whose center of distortion is at the center of the sensor, this will be (0,0).
      /// The principal ray is always along the -Z axis from the center of the image sensor (which will
      /// be between pixels for a sensor with an even pixel count).
      /// @param controlPoints Control points for the distortion.  These points are the radial distance
      /// from the center of projection and they lie on the Z =-1 plane.  The first
      /// element is the radial distance for the projection of a pixel that sees an object, and the second
      /// element is the radial distance for the actual location of the object as projected onto the Z = -1 plane
      /// using the ideal camera's coordinate system.  The control points must start at 0 and be in increasing
      /// order of radial distance.
      /// There must be at least two control points.  Points that project outside of the range of the control points will be
      /// left unchanged (remember that the control points must reach all the way to the corners of the image,
      /// not just the edges).
      DistortionRadialLERP(std::array<double, 2> const &COP, std::vector<std::array<double, 2>> const &controlPoints);

      std::array<double, 3> MapPoint(std::array<double, 3> point) const override;

    protected:
      std::array<double, 2> m_COP;
      std::vector<std::array<double, 2>> m_controlPoints;
    };

    /// @brief Distortion model that uses a bag of mappings from as-seen points to real-world points on the Z=-1 plane
    /// of an ideal camera model.
    /// @details This model is constructed from a vector of mappings from 2D as-seen points to real-world points
    /// in the ideal camera's Z = -1 projection plane.  The points need not lie in a grid or in any particular order.
    /// The mapping is done by finding the closest three non-collinear points in the bag and using them to perform
    /// a linear interpolation or extrapolation of the distortion.
    class DistortionBagOfMappings : public Distortion {
    public:
      /// @brief A 2D point location in the projection plane.
      typedef std::array<double, 2> Point2D;
      /// @brief A mapping from an as-seen point in the ideal camera's Z = -1 projection plane to the actual direction point
      /// in the same projection plane.
      /// @details The first point is the projected location of a pixel (perhaps to subpixel accuracy) that sees the center
      /// of an object in the ideal camera's Z = -1 projection plane.  The second point is the projected location of the
      /// actual object in the ideal camera's Z = -1 projection plane.  For example, if a real camera sees the center of a
      /// bright sphere at a specific pixel, the first point would be the projection of that pixel into the Z = -1 plane
      /// and the second point would be the projection of the actual location of the center of the bright sphere into
      /// the same Z = -1 plane using the ideal camera's parameters.
      typedef std::array<Point2D, 2> Mapping;
      /// @brief A vector of mappings in arbitrary order in the plane.
      typedef std::vector<Mapping> Bag;

      /// @brief Constructor that takes the bag of points in the projection plane.
      /// @param mappings A bag of mappings from as-seen points to real-world points in the Z=-1 plane.
      /// The same as-seen point must not appear more than once in the bag or points near it may not be
      /// properly distorted.
      DistortionBagOfMappings(Bag const& mappings);

      std::array<double, 3> MapPoint(std::array<double, 3> point) const override;

    protected:
      /// @brief Implementation class for the DistortionBagOfMappings class to hide details from
      /// the caller so they don't need to #include Boost headers.
      class DistortionBagOfMappings_impl;
      std::shared_ptr<DistortionBagOfMappings_impl> m_impl;

      /// @brief Compute the Barycentric coordinates for the point in the triangle whose vertices are given.
      /// @param p Point to compute the barycentric coordinates for.
      /// @param a First point of the triangle.
      /// @param b Second point of the triangle.
      /// @param c Third point of the triangle.
      /// @return The barycentric coordinates of the point in the triangle.  The coordinates always sum
      /// to 1 and are all positive if the point is inside the triangle and they are zero if the point
      /// is on the triangle. You can interpolate or extrapolate values at the triangle corners using
      /// return[0] * v1 + return[1] * v2 + return[2] * v3, where v1, v2, and v3 are the values at
      /// vertices a, b, and c respectively.
      static std::array<double, 3> BarycentricCoordinates(const Point2D& p,
        const Point2D& a, const Point2D& b, const Point2D& c);

      /// @brief Determine whether a point is inside the triangle whose three points are given.
      /// @param p Point to test.
      /// @param a First point of the triangle.
      /// @param b Second point of the triangle.
      /// @param c Third point of the triangle.
      /// @return True if the point is inside the triangle, false otherwise.
      static bool IsPointInTriangle(const Point2D& p,
        const Point2D& a, const Point2D& b, const Point2D& c);

      /// @brief Interpolate a value given three points with values on a triangle and a fourth point in the plane.
      /// @details This function interpolates a value at a point in the plane given three points in the plane
      /// and their values.  The value is interpolated using a linear interpolation of the values at the three
      /// points.  It can also extrapolate outside of the triangle.
      /// @param p1 First point in the triangle.
      /// @param v1 Value at the first point.
      /// @param p2 Second point in the triangle.
      /// @param v2 Value at the second point.
      /// @param p3 Third point in the triangle.
      /// @param v3 Value at the third point.
      /// @param p Fourth point in the plane, where the value will be calculated.
      /// @return The interpolated value at the fourth point.  Returns the value at the first point if the triangle
      /// is degenerate.
      static double DetermineValue(Point2D const& p1, double v1,
        Point2D const& p2, double v2,
        Point2D const& p3, double v3,
        Point2D const& p);

      friend std::string Distortion::Test();
    };

  } // namespace render
} // namespace asdp
