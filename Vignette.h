/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

 /**
  * @file Vignette.h
  * @brief Apache Strap-Down Pilotage Render/Vignette class header file.
  *
  * @author ReliaSolve.
  * @date December 31, 2024.
  */

#pragma once

#include <array>
#include <vector>
#include <string>

namespace asdp {
  namespace render {

    /// @brief Base class for vignette correction that defines the interface.
    class Vignette {
    public:

      /// @brief Virtual destructor so that proper deconstruction happens on pointers.
      virtual ~Vignette() = default;

      /// @brief Evaluate the scale factor at a normalized 2D point within the image.
      /// @param point Point in 2D space that spans the range [-1,1] in both X and Y,
      /// covering the range of the image. X is higher to the right, Y is higher at the top;
      /// so (1,1) is the upper right corner of the image.
      /// @return Value to multiply the image brightness by at this location to correct
      /// for vignetting.
      virtual double EvaluateAtPoint(std::array<double, 2> point) const = 0;

      /// @brief Test function for all subclasses that returns an empty string on success or an error message on failure.
      static std::string Test();
    };

    /// @brief Null vignette model that returns 1.0 everywhere.
    class VignetteNone : public Vignette {
    public:
      double EvaluateAtPoint(std::array<double, 2> point) const override { return 1.0; }
    };

    /// @brief Polynomial model that is radial from a center of projection.
    /// @details This vignette model is radial from a specified center of projection.
    class VignetteRadialPolynomail : public Vignette {
    public:
      /// @brief Constructor that takes the center of projection and coefficients for the polynomial.
      /// @param COP Center of projection for the distortion.  This (X,Y) is the center of the vignette
      /// in normalized image coordinates, in [-1,1] for both X and Y.
      /// For an ideal camera, the center of the sensor would be (0.0, 0.0).  Location (1,1) is the upper
      /// right.
      /// @param FOVsDegrees Field of view for the camera in degrees, horizontal then vertical.
      /// This is needed to convert from normalized image coordinates to a consistent 2D space.
      /// @param coefficients Coefficients for the polynomial that defines the vignette.  These
      /// are in the space of image half-widths, so that a value of 1.0 goes from the center of the
      /// image to the right or left edge.
      VignetteRadialPolynomail(std::array<double, 2> const &COP, std::array<double, 2> const &FOVsDegrees,
        std::vector<double> const &coefficients);

      double EvaluateAtPoint(std::array<double, 2> point) const override;

    protected:
      std::array<double, 2> m_COP;
      std::array<double, 2> m_halfSizes;
      double m_vScale;
      std::vector<double> m_coefficients;
    };

  } // namespace render
} // namespace asdp