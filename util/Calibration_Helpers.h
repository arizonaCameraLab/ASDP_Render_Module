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

#include <iostream>
#include <CameraRenderInfo.h>
#include <glm/glm.hpp>

/// @brief Compute the world-space ray from the camera through the specified pixel, ignoring distortion.
/// @param cri The camera render information to use to determine the ray in camera space.
/// @param xPixels The X pixel coordinate, may not be centered on a pixel.
/// @param yPixels The Y pixel coordinate, may not be centered on a pixel.
/// @param zRotationDegrees The gimbal rotation around the Z axis in degrees.
/// @param xRotationDegrees The gimbal rotation around the X axis in degrees.
/// @param outRayStartInWorld The start of the ray in world (gimbol helicopter) coordinates.
/// @param outRayDirectionInWorld The direction of the ray in world (gimbol helicopter) coordinates.
void WorldSpaceRayNoDistortion(const asdp::render::CameraRenderInfo& cri, double xPixels, double yPixels,
  double zRotationDegrees, double xRotationDegrees,
  glm::dvec3& outRayStartInWorld, glm::dvec3& outRayDirectionInWorld,
  bool verbose = false);
