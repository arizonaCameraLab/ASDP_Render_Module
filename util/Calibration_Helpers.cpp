/*
 * Copyright (C) 2025: Arizona Board of Regents on Behalf of the University of Arizona
 */

/**
 * @file Calibration_Helpers.cpp
 * @brief Apache Strap-Down Pilotage configuration calibration program.
 *
* @author ReliaSolve.
* @date April 2nd, 2025.
*/

#include <Calibration_Helpers.h>
#include <glm/gtc/quaternion.hpp>
#include <iostream>

using namespace asdp::render;
using namespace asdp::render::calibration;

void asdp::render::calibration::WorldSpaceRayNoDistortion(const CameraRenderInfo& cri,
  double xPixels, double yPixels,
  double zRotationDegrees, double xRotationDegrees,
  glm::dvec3& outRayStartInWorld, glm::dvec3& outRayDirectionInWorld,
  bool verbose)
{
  int width = cri.m_resolutionPixels[0];
  int height = cri.m_resolutionPixels[1];

  glm::dvec3 rayStart;
  rayStart.x = cri.m_positionMeters[0];
  rayStart.y = cri.m_positionMeters[1];
  rayStart.z = cri.m_positionMeters[2];

  double normalizedX = xPixels / (width - 1);
  double normalizedY = yPixels / (height - 1);
  double halfWidth = atan(glm::radians(cri.m_fovDegrees[0] / 2.0));
  double halfHeight = atan(glm::radians(cri.m_fovDegrees[1] / 2.0));
  double xScaled = halfWidth * (2 * normalizedX - 1);
  double yScaled = halfHeight * (1 - 2 * normalizedY);    // Flip the Y axis to make it right handed

  // The pixel centers are half a pixel in from the edge, so we scale to make this happen.
  double xM = xScaled * (width - 1.0) / width;
  double yM = yScaled * (height - 1.0) / height;
  double zM = -1.0;

  // Convert from +X, +Y camera space pointing along -Z to helicopter space with +Z up and +Y forward.
  glm::dvec3 rayDirection = glm::normalize(glm::dvec3(xM, -zM, yM));
  if (verbose) {
    std::cout << "  Camera-space ray direction = " << rayDirection.x << " " << rayDirection.y << " " << rayDirection.z << std::endl;
  }

  // Rotate the ray direction by the camera orientation; X, then Y then Z.
  glm::dvec3 rayDirectionInBall;
  {
    glm::dquat rotationX = glm::angleAxis(glm::radians(cri.m_orientationDegrees[0]), glm::dvec3(1.0, 0.0, 0.0));
    glm::dquat rotationY = glm::angleAxis(glm::radians(cri.m_orientationDegrees[1]), glm::dvec3(0.0, 1.0, 0.0));
    glm::dquat rotationZ = glm::angleAxis(glm::radians(cri.m_orientationDegrees[2]), glm::dvec3(0.0, 0.0, 1.0));
    glm::dquat rotationTotal = rotationX * rotationY * rotationZ;
    rayDirectionInBall = rotationTotal * rayDirection;
  }
  if (verbose) {
    std::cout << "  Ball-space ray direction = " << rayDirectionInBall.x << " " << rayDirectionInBall.y << " " << rayDirectionInBall.z << std::endl;
  }

  // Okay, we now have the ray start and direction in the camera ball's coordinate system.
  // We must rotate both by the gimbal angle, around Z first and then around X.
  {
    glm::dquat rotationZ = glm::angleAxis(glm::radians(zRotationDegrees), glm::dvec3(0.0, 0.0, 1.0));
    glm::dquat rotationX = glm::angleAxis(glm::radians(xRotationDegrees), glm::dvec3(1.0, 0.0, 0.0));
    glm::dquat rotationTotal = rotationZ * rotationX;
    outRayDirectionInWorld = rotationTotal * rayDirectionInBall;
    outRayStartInWorld = rotationTotal * rayStart;
  }
  if (verbose) {
    std::cout << "  World-space ray direction = " << outRayDirectionInWorld.x << " " << outRayDirectionInWorld.y << " " << outRayDirectionInWorld.z << std::endl;
    std::cout << "  World-space ray start = " << outRayStartInWorld.x << " " << outRayStartInWorld.y << " " << outRayStartInWorld.z << std::endl;
  }
}

void asdp::render::calibration::PointPixelAtTarget(const CameraRenderInfo& cri,
  double xPixels, double yPixels,
  double minXRotationDegrees, double maxXRotationDegrees,
  glm::dvec3 target,
  double& outZRotationDegrees, double& outXRotationDegrees,
  double precisionDegrees,
  bool verbose)
{
  // First solve for the X angle that points the specific pixel at the height of the target
  // when it passes through the cylinder at the radius of the target from the center of the world.
  // Do this by brute-force testing for the minimum miss distance across the specified precision
  // across the specified range.
  double minMissDistance = 1e30;
  double bestXRotation = 0.0;
  glm::dvec3 bestClosestPoint = { 0, 0, 0 };
  for (double xRotation = minXRotationDegrees; xRotation <= maxXRotationDegrees; xRotation += precisionDegrees) {
    glm::dvec3 rayStartInWorld, rayDirectionInWorld;
    WorldSpaceRayNoDistortion(cri, xPixels, yPixels, 0.0, xRotation, rayStartInWorld, rayDirectionInWorld, false);

    // The ray is defined by the start and direction.  The target is defined by the center.
    // Find the point of closest approach of the ray to the target.  Do this by projecting
    // the target onto the ray and then finding the distance between the target and the projection.
    glm::dvec3 targetToRay = target - rayStartInWorld;
    double distanceAlongRay = glm::dot(targetToRay, rayDirectionInWorld);
    glm::dvec3 closestPoint = rayStartInWorld + distanceAlongRay * rayDirectionInWorld;
    double missDistance = glm::length(target - closestPoint);
    if (missDistance < minMissDistance) {
      minMissDistance = missDistance;
      bestXRotation = xRotation;
      bestClosestPoint = closestPoint;
    }
  }

  // Find the angle in the Z=0 plane of the closest point.
  double zRotation = atan2(bestClosestPoint.y, bestClosestPoint.x);

  // Find the angle in the Z=0 plane of the target.
  double targetZRotation = atan2(target.y, target.x);

  // The difference between the two is the Z rotation.
  double zDelta = targetZRotation - zRotation;

  outZRotationDegrees = glm::degrees(zDelta);
  outXRotationDegrees = glm::degrees(bestXRotation);
}

std::string asdp::render::calibration::Test()
{
  return "@todo Test";
}
