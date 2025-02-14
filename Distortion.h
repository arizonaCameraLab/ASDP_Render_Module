/*
 * Copyright (C) 2024-2025: Arizona Board of Regents on Behalf of the University of Arizona
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

namespace asdp {
  namespace render {

    /// @brief Base class for distortion correction that defines the interface.
    class Distortion {
    public:

      /// @brief Virtual destructor so that proper deconstruction happens on pointers.
      virtual ~Distortion() = default;

      /// @brief Map a point from the ideal camera to the distorted real camera location.
      /// @param point Point in ideal-camera space.  The distortion map will have been generated
      /// for an ideal camera of known horizontal and vertical fields of view whose center of
      /// projection is the center of the image sensor.  The camera will be right-handed with its
      /// +X axis to the right in the image, its +Y axis pointing up in the image, looking down
      /// the -Z axis.  The center of projection for the camera is at the origin.  For example,
      /// the point (0, 0, -5) will project to the center of the image, 5 units away from the camera
      /// in an ideal camera's space.
      /// @return Projection point for this pixel in the real camera.  The point will have the same
      /// Z value as the input point, but the X and Y values will be distorted to account for the
      /// lens distortion.
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
      /// @param COP Center of projection for the distortion.  This (X,Y) is the location where the center of
      /// projection pierces the Z=-1 plane.  For an ideal camera, the center of the sensor would be (0.0, 0.0).
      /// @param controlPoints Control points for the distortion.  These points are the radial distance
      /// from the center of projection and they lie on a plane that is 1 unit down the -Z axis.  The first
      /// element is the radial distance in the undistored case, and the second element is the radial distance
      /// in the distorted case.  The control points must start at 0 and be in increasing order of radial distance.
      /// There must be at least two control points.  Points outside of the range of the control points will be
      /// left unchanged (remember that the control points must reach all the way to the corners of the image,
      /// not just the edges).
      DistortionRadialLERP(std::array<double, 2> const &COP, std::vector<std::array<double, 2>> const &controlPoints);

      std::array<double, 3> MapPoint(std::array<double, 3> point) const override;

    protected:
      std::array<double, 2> m_COP;
      std::vector<std::array<double, 2>> m_controlPoints;
    };

    /// @brief Distortion model that uses a bag mappins from projection-plane points from the ideal camera to the distorted camera.
    /// @details This model is constructed with a vector of mappings from 2D points in the ideal camera's
    /// projection plane to points in the distorted camera's projection plane.  The points do not have to be
    /// in a grid or in any particular order.  The mapping is done by finding the closest three non-collinear
    /// points in the bag and using them to perform a linear interpolation or extrapolation of the distortion.
    class DistortionBagOfMappings : public Distortion {
    public:
      /// @brief A 2D point location in the projection plane.
      typedef std::array<double, 2> Point2D;
      /// @brief A mapping from a point in the ideal camera's projection plane to a distorted point also in the projection plane.
      typedef std::array<Point2D, 2> Mapping;
      /// @brief A row of points in the projection plane. Every row most have the same number of points.
      typedef std::vector<Mapping> Bag;

      /// @brief Constructor that takes the bag of points in the projection plane.
      /// @param mappings A bag of mappings from undistorted point to distorted points in the Z=-1 plane.
      /// The same undistorted point must not appear more than once in the bag or points near it may not be
      /// distorted.
      DistortionBagOfMappings(Bag const& mappings);

      std::array<double, 3> MapPoint(std::array<double, 3> point) const override;

    protected:
      /// @brief Keeps track of all of the points that were passed into the constructor.
      Bag m_mappings;

      /// @brief 2D grid of Bags to help speed up the search for the nearest points.
      /// @details The first dimension is the X coordinate, the second dimension is the Y coordinate.
      static const size_t m_numSamplesX = 20, m_numSamplesY = 16;
      std::array<std::array<Bag, m_numSamplesY>, m_numSamplesX> m_grid;

      /// @brief The range of the grid in X and Y.
      std::array<double, 2> m_rangeX, m_rangeY;

      /// @brief Find the distance between two 2D points.
      /// @param p1 First point.
      /// @param p2 Second point.
      /// @return The distance between the two points.
      inline static double PointDistance(Point2D const& p1, Point2D const& p2)
      {
        double dx = p2[0] - p1[0];
        double dy = p2[1] - p1[1];
        return std::sqrt(dx * dx + dy * dy);
      }

      /// @brief Determine if three points are nearly collinear.  If so, they are not suitable for interpolation.
      /// @param p1 First point.
      /// @param p2 Second point.
      /// @param p3 Third point.
      /// @return True if the points are nearly collinear, false otherwise.
      static bool NearlyCollinear(DistortionBagOfMappings::Point2D const& p1,
        DistortionBagOfMappings::Point2D const& p2,
        DistortionBagOfMappings::Point2D const& p3);

      /// @brief Find the three nearest non-collinear points in the bag to a given point.
      /// @param p Point to find the three nearest points to.
      /// @param points Bag of points to search.
      /// @return Bag of the three nearest points unless there are not enough, in which case the
      /// bag will contain fewer than three points.
      static Bag FindThreeNearestPointsInBag(Point2D const& p, Bag const& points);

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
