/*
 * Copyright (C) 2025: Arizona Board of Regents on Behalf of the University of Arizona
 */

/**
 * @file Calibration_Helpers.h
 * @brief Apache Strap-Down Pilotage configuration calibration program.
 *
* @author ReliaSolve.
* @date April 2nd, 2025.
*/

#include <CameraRenderInfo.h>
#include <glm/glm.hpp>
#include <string>

namespace asdp { namespace render { namespace calibration {

/// @brief Compute the world-space ray from the camera through the specified pixel, ignoring distortion.
/// @param cri The camera render information to use to determine the ray in camera space.
/// @param xPixels The X pixel coordinate, may not be centered on a pixel.
/// @param yPixels The Y pixel coordinate, may not be centered on a pixel.
/// @param zRotationDegrees The gimbal rotation around the Z axis in degrees.
/// @param xRotationDegrees The gimbal rotation around the X axis in degrees.
/// @param outRayStartInWorld The start of the ray in world (gimbol helicopter) coordinates.
/// @param outRayDirectionInWorld The direction of the ray in world (gimbol helicopter) coordinates.
/// @param verbose If true, print debugging information.
void WorldSpaceRayNoDistortion(const asdp::render::CameraRenderInfo& cri, double xPixels, double yPixels,
  double zRotationDegrees, double xRotationDegrees,
  glm::dvec3& outRayStartInWorld, glm::dvec3& outRayDirectionInWorld,
  bool verbose = false);

/// @brief Compute the gimbal angles to point a camera pixel at the specified 3D target location
/// as closely as possible.
/// @param cri The camera render information to use to determine the ray in camera space.
/// @param xPixels The X pixel coordinate, may not be centered on a pixel.
/// @param yPixels The Y pixel coordinate, may not be centered on a pixel.
/// @param minXRotationDegrees The minimum gimbal rotation around the X axis in degrees, probably negative.
/// @param maxXRotationDegrees The maximum gimbal rotation around the X axis in degrees, probably positive.
/// @param target The 3D target location in world (gimbol helicopter) coordinates.
/// @param outZRotationDegrees The gimbal rotation around the Z axis in degrees.
/// @param outXRotationDegrees The gimbal rotation around the X axis in degrees.
/// @param precisionDegrees The required precision for the gimbal angles.
/// @param verbose If true, print debugging information.
void PointPixelAtTarget(const asdp::render::CameraRenderInfo& cri, double xPixels, double yPixels,
  double minXRotationDegrees, double maxXRotationDegrees,
  glm::dvec3 target,
  double &outZRotationDegrees, double &outXRotationDegrees,
  double precisionDegrees = 0.01,
  bool verbose = false);

/// @brief Test the calibration helpers.
/// @return An empty string if the test passes, otherwise an error message.
std::string Test();

} } };
