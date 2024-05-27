/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

 /**
  * @file Distortion.h
  * @brief Apache Strap-Down Pilotage Render/Distortion class header file.
  *
  * @author ReliaSolve.
  * @date April 29, 2024.
  */

#pragma once

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
      /// the -Z axis.  The center of projection for the camera is at the origin.  For example.
      /// the point (0, 0, -5) will project to the center of the image, 5 units away from the camera
      /// in any ideal camera's space.
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
      /// @param COP Center of projection for the distortion.  This is the normalized point in the range
      /// [-1..1] for each axis going from one side of the sensor to the other.  For an ideal camera, the
      /// center of the sensor would be (0.0, 0.0).
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
      std::vector<std::array<double, 2>> m_ControlPoints;
    };

  } // namespace render
} // namespace asdp