/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include <cmath>
#include <iostream>
#include <Vignette.h>
using namespace asdp::render;

static double Pi = 3.14159265358979323846;

VignetteRadialPolynomail::VignetteRadialPolynomail(std::array<double, 2> const& COP, std::array<double, 2> const& FOVsDegrees,
    std::vector<double> const& coefficients)
  : m_COP(COP)
  , m_coefficients(coefficients)
{
  if (FOVsDegrees[0] <= 0) {
    std::cerr << "VignetteRadialPolynomail: Invalid horizontal field of view: " << std::to_string(FOVsDegrees[0]) << std::endl;
    m_coefficients.clear();
  }
  if (FOVsDegrees[1] <= 0) {
    std::cerr << "VignetteRadialPolynomail: Invalid vertical field of view: " << std::to_string(FOVsDegrees[1]) << std::endl;
    m_coefficients.clear();
  }
  if (m_coefficients.size() == 0) {
    std::cerr << "VignetteRadialPolynomail: Not enough coefficients: " << std::to_string(m_coefficients.size()) << std::endl;
    m_coefficients.clear();
  }

  // Compute the half-width and half-height sizes of the image.
  m_halfSizes[0] = std::tan(FOVsDegrees[0] * 0.5 * Pi / 180.0);
  m_halfSizes[1] = std::tan(FOVsDegrees[1] * 0.5 * Pi / 180.0);

  // Compute the scale factor to take vertical distances to horizontal distances.
  m_vScale = m_halfSizes[0] / m_halfSizes[1];
}

double VignetteRadialPolynomail::EvaluateAtPoint(std::array<double, 2> point) const
{
  // If we don't have enough coefficients, return 1.0
  if (m_coefficients.size() == 0) {
    return 1.0;
  }

  // Find our radial distance in the image.
  double dx = point[0] - m_COP[0];
  double dy = (point[1] - m_COP[1]) * m_vScale;
  double r = std::sqrt(dx*dx + dy*dy);

  // Evaluate the polynomial at this radius.
  double value = 0.0;
  double multiplier = 1.0;
  double r2 = r * r;
  for (double coefficient : m_coefficients) {
    value += coefficient * multiplier;
    multiplier *= r2;
  }

  return value;
}


//==============================================================================
// Test and its helper functions

bool isNear(double a, double b, double epsilon = 1e-6)
{
  return std::abs(a-b) < epsilon;
}

std::string Vignette::Test()
{
  // Test the VignetteNone class.  All return values should be 1.0.
  {
    VignetteNone none;
    if (none.EvaluateAtPoint({0.0, 0.0}) != 1.0) {
      return "VignetteNone: Expected 1.0 at (0,0)";
    }
    if (none.EvaluateAtPoint({-1.0, 0.0}) != 1.0) {
      return "VignetteNone: Expected 1.0 at (-1,0)";
    }
    if (none.EvaluateAtPoint({1.0, 0.0}) != 1.0) {
      return "VignetteNone: Expected 1.0 at (1,0)";
    }
    if (none.EvaluateAtPoint({0.0, -1.0}) != 1.0) {
      return "VignetteNone: Expected 1.0 at (0,-1)";
    }
    if (none.EvaluateAtPoint({0.0, 1.0}) != 1.0) {
      return "VignetteNone: Expected 1.0 at (0,1)";
    }
  }

  // Test the VignetteRadialPolynomail class.
  {
    std::array<double, 2> COP = {0.5, 0.1};
    // This has an aspect ratio of 2:1 on the image, so we only need to move half as
    // far in Y to match X motion.
    std::array<double, 2> FOVsDegrees = {90.0, 53.13};
    std::vector<double> coefficients = {1.0, 0.1};

    VignetteRadialPolynomail vig(COP, FOVsDegrees, coefficients);
    if (!isNear(vig.EvaluateAtPoint({0.5, 0.1}), 1.0)) {
      return "VignetteRadialPolynomail: Expected 1.0 at (0.5,0.1)";
    }
    if (!isNear(vig.EvaluateAtPoint({-0.5, 0.1}), 1.1)) {
      return "VignetteRadialPolynomail: Expected 1.1 at (-0.5,0.1)";
    }
    if (!isNear(vig.EvaluateAtPoint({0.5, 0.6}), 1.1)) {
      return "VignetteRadialPolynomail: Expected 1.1 at (0.5,0.6)";
    }
  }

  // Test with a four-term polynomial.
  {
    std::array<double, 2> COP = {0.0, 0.0};
    std::array<double, 2> FOVsDegrees = {90.0, 90.0};
    std::vector<double> coefficients = {1.0, 0.1, 0.01, 0.001};

    VignetteRadialPolynomail vig(COP, FOVsDegrees, coefficients);
    if (!isNear(vig.EvaluateAtPoint({0.0, 0.0}), 1.0)) {
      return "VignetteRadialPolynomail (four terms): Expected 1.0 at (0.0,0.0)";
    }
    double expected = 1.0 + 0.1 + 0.01 + 0.001;
    if (!isNear(vig.EvaluateAtPoint({1.0, 0.0}), expected)) {
      return "VignetteRadialPolynomail (four terms): Expected " + std::to_string(expected) + " at (1.0,0.0)";
    }
    expected = 1.0 + 0.1*4 + 0.01*16 + 0.001*64;
    if (!isNear(vig.EvaluateAtPoint({2.0, 0.0}), expected)) {
      return "VignetteRadialPolynomail (four terms): Expected " + std::to_string(expected) + " at (2.0,0.0)";
    }
  }

  return "";
}
