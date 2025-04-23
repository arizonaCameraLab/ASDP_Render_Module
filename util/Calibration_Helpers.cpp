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
#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <filesystem>

using namespace asdp::render;
using namespace asdp::render::calibration;
using json = nlohmann::json;

std::vector<CameraRenderInfo> asdp::render::calibration::GetCameraRenderInfos(
  const std::string& configFileName)
{
  // Read the configuration files.
  if (!std::filesystem::exists(configFileName)) {
    throw std::runtime_error("Configuration file not found: " + configFileName);
  }
  std::ifstream configFile1(configFileName);
  json camConfig = json::parse(configFile1);

  // Construct CameraRenderInfos for each configuration file.
  std::vector<asdp::render::CameraRenderInfo> cameraRenderInfos;
  for (const auto& camera : camConfig["cameras"]) {
    std::shared_ptr<Distortion> dist;
    json distortion = camera["distortion"];
    if (distortion["type"] == "none") {
      DistortionNone* distortion = new DistortionNone;
      dist = std::shared_ptr<Distortion>(distortion);
    } else if (distortion["type"] == "radial") {
      json parameters = distortion["parameters"];
      std::array<double, 2> center = parameters["COP"];
      json map = parameters["map"];
      std::vector< std::array<double, 2> > mapPoints = map;
      DistortionRadialLERP* distortion = new DistortionRadialLERP(center, mapPoints);
      dist = std::shared_ptr<Distortion>(distortion);
    } else {
      throw std::runtime_error("Unknown distortion type: " + std::string(distortion["type"]));
    }

    std::shared_ptr<Vignette> vig(new VignetteNone);
    try {
      json vignette = camera["vignette"];
      if (vignette["type"] == "evenPolynomial") {
        json parameters = vignette["parameters"];
        std::array<double, 2> center = parameters["COP"];
        std::array<double, 2> cArray = parameters["coefficients"];
        std::vector<double> coefficients(cArray.begin(), cArray.end());
        VignetteRadialPolynomail* vignette = new VignetteRadialPolynomail(center, camera["fieldOfViewDegrees"], coefficients);
        vig = std::shared_ptr<Vignette>(vignette);
      } else if (vignette["type"] == nullptr) {
        // No vignette specified, so use the default.
      } else {
        throw std::runtime_error("Unknown vignette type: " + std::string(vignette["type"]));
      }
    } catch (...) {
      // No vignette specified, so use the default.
    }

    asdp::render::CameraRenderInfo info(camera["id"],
      camera["positionMeters"], camera["orientationDegrees"],
      camera["resolutionPixels"], camera["fieldOfViewDegrees"],
      dist, vig, std::make_shared<asdp::render::ImageQueue>(), -1.0f);
    cameraRenderInfos.push_back(info);
  }

  return cameraRenderInfos;
}

std::vector<TargetInfo> asdp::render::calibration::GetTargetInfos(
  const std::string& configFileName)
{
  if (!std::filesystem::exists(configFileName)) {
    throw std::runtime_error("Configuration file not found: " + configFileName);
  }
  std::ifstream configFile(configFileName);
  try {
    json targetConfig = json::parse(configFile);

    std::vector<TargetInfo> targetInfos;
    for (const auto& target : targetConfig["targets"]) {
      try {
        TargetInfo info;
        info.id = target["id"];
        info.position.x = target["positionMeters"][0];
        info.position.y = target["positionMeters"][1];
        info.position.z = target["positionMeters"][2];
        targetInfos.push_back(info);
      }
      catch (...) {
        throw std::runtime_error("Unable to parse target information.");
      }
    }
    return targetInfos;
  }
  catch (...) {
    throw std::runtime_error("Unable to parse target configuration file: " + configFileName);
  }
}

std::vector<PoseInfo> asdp::render::calibration::GetPoseInfos(
  const std::string& filename)
{
  if (!std::filesystem::exists(filename)) {
    throw std::runtime_error("Pose file not found: " + filename);
  }
  std::ifstream poseFile(filename);
  std::vector<PoseInfo> poseInfos;
  std::string line;
  // Skip the header line.
  std::getline(poseFile, line);
  // Parse each comma-separated line.
  while (std::getline(poseFile, line)) {
    std::istringstream iss(line);
    PoseInfo info;
    char comma;
    // Read the comma-separated values.
    // Note that we use the >> operator to read the values, which will skip whitespace.
    // The format is: frameIndex,zRotationDegrees,xRotationDegrees,cameraID,numFrames
    if (!(iss >> info.frameIndex >> comma >> info.zRotationDegrees >> comma >>
          info.xRotationDegrees >> comma >> info.cameraID >> comma >> info.numFrames)) {
      throw std::runtime_error("Unable to parse pose information.");
    }
    poseInfos.push_back(info);
  }
  return poseInfos;
}

GimbalInfo asdp::render::calibration::GetGimbalInfo(
  const std::string& filename)
{
  if (!std::filesystem::exists(filename)) {
    throw std::runtime_error("Gimbal file not found: " + filename);
  }
  std::ifstream configFile(filename);
  GimbalInfo gimbalInfo;
  // Get the required fields
  json gimbalConfig;
  try {
    gimbalConfig = json::parse(configFile);
    gimbalInfo.name = gimbalConfig["name"];
    gimbalInfo.pitchFirst = gimbalConfig["pitchFirst"];
    gimbalInfo.minYawDegrees = gimbalConfig["minYawDegrees"];
    gimbalInfo.maxYawDegrees = gimbalConfig["maxYawDegrees"];
    gimbalInfo.minPitchDegrees = gimbalConfig["minPitchDegrees"];
    gimbalInfo.maxPitchDegrees = gimbalConfig["maxPitchDegrees"];
  }
  catch (...) {
    throw std::runtime_error("Unable to parse gimbal configuration file: " + filename);
  }
  // Get the optional fields
  try { gimbalInfo.comPort = gimbalConfig["comPort"]; } catch (...) { }
  try { gimbalInfo.baud = gimbalConfig["baud"]; } catch (...) { }
  try { gimbalInfo.speed = gimbalConfig["speed"]; } catch (...) { }
  try { gimbalInfo.acceleration = gimbalConfig["acceleration"]; } catch (...) { }

  return gimbalInfo;
}

std::shared_ptr<Gimbal> asdp::render::calibration::ConstructGimbal(
  const GimbalInfo& gimbalInfo)
{
  // Create a gimbal object based on the information provided.
  std::shared_ptr<Gimbal> gimbal;
  if (gimbalInfo.name == "Zaber_X_G_RST") {
    gimbal = std::make_shared<GimbalZaber_X_G_RST>(gimbalInfo.comPort, 1, gimbalInfo.speed,
      gimbalInfo.acceleration);
  }
  else if (gimbalInfo.name == "iOptron_CEM40") {
    gimbal = std::make_shared<Gimbal_iOptron_CEM40>(gimbalInfo.comPort);
  }
  else if (gimbalInfo.name == "Fake") {
    gimbal = std::make_shared<GimbalFake>();
  }
  else {
    throw std::runtime_error("Unknown gimbal type: " + gimbalInfo.name);
  }
  return gimbal;
}

void asdp::render::calibration::WorldSpaceRayNoDistortion(const CameraRenderInfo& cri,
  double xPixels, double yPixels, bool rotateXFirst,
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
  double halfWidth = tan(glm::radians(cri.m_fovDegrees[0] / 2.0));
  double halfHeight = tan(glm::radians(cri.m_fovDegrees[1] / 2.0));
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
  // We must rotate both by the gimbal angle, rotating around the two axes in the specified order.
  {
    glm::dquat rotationZ = glm::angleAxis(glm::radians(zRotationDegrees), glm::dvec3(0.0, 0.0, 1.0));
    glm::dquat rotationX = glm::angleAxis(glm::radians(xRotationDegrees), glm::dvec3(1.0, 0.0, 0.0));
    glm::dquat rotationTotal;
    if (rotateXFirst) {
      rotationTotal = rotationX * rotationZ;
    } else {
      rotationTotal = rotationZ * rotationX;
    }
    outRayDirectionInWorld = rotationTotal * rayDirectionInBall;
    outRayStartInWorld = rotationTotal * rayStart;
  }
  if (verbose) {
    std::cout << "  World-space ray direction = " << outRayDirectionInWorld.x << " " << outRayDirectionInWorld.y << " " << outRayDirectionInWorld.z << std::endl;
    std::cout << "  World-space ray start = " << outRayStartInWorld.x << " " << outRayStartInWorld.y << " " << outRayStartInWorld.z << std::endl;
  }
}

void asdp::render::calibration::PointPixelAtTargetNoDistortion(const CameraRenderInfo& cri,
  double xPixels, double yPixels,
  double minXRotationDegrees, double maxXRotationDegrees,
  glm::dvec3 target,
  double rotateXFirst,
  double& outZRotationDegrees, double& outXRotationDegrees,
  double precisionDegrees,
  bool verbose)
{
  if (rotateXFirst) { // Handle the case of rotating around X first as a separate case.
    // We brute-force search for the pair of rotations that points as close to the target as possible.
    // We start with a step size of 4 degrees on each axis to reduce the amount of time the global
    // search takes and then refine the search by dividing the step size by 2 until we reach the
    // specified precision.  Within the local search we search the local range in each axis, adding the delta
    // to the previous best solution.  We do this until we reach the specified precision.

    double minMissDistance = 1e30;
    double bestXRotationDegrees = 0.0;
    double bestZRotationDegrees = 0.0;
    double minZRotationDegrees = -180.0;
    double maxZRotationDegrees = 179.999;
    double stepSize = 4.0;

    do {
      for (double rz = minZRotationDegrees; rz <= maxZRotationDegrees; rz += stepSize) {
        for (double rx = minXRotationDegrees; rx <= maxXRotationDegrees; rx += stepSize) {
          // Compute the ray start and direction in world space.
          glm::dvec3 rayStartInWorld, rayDirectionInWorld;
          WorldSpaceRayNoDistortion(cri, xPixels, yPixels, rotateXFirst, rz, rx,
            rayStartInWorld, rayDirectionInWorld, verbose);

          // Find the point on the line through rayStartInWorld and rayDirectionInWorld
          // that is closest to the target.  We find the point on the line that is closest to
          // the target by moving along the direction by the dot product of the target offset
          // and the direction.  If this is behind the ray start, we use the ray start instead.
          glm::dvec3 CP = rayStartInWorld + glm::dot(target - rayStartInWorld, rayDirectionInWorld)
            * rayDirectionInWorld;
          if (glm::dot(CP - rayStartInWorld, rayDirectionInWorld) <= 0) {
            CP = rayStartInWorld;
          }

          // Compute the distance from the target to the closest point on the ray.
          double missDistance = glm::length(CP - target);
          if (missDistance < minMissDistance) {
            minMissDistance = missDistance;
            bestXRotationDegrees = rx;
            bestZRotationDegrees = rz;
          }
        }
      }

      // We have found the best solution for this step size, so we need to refine the search.
      // We do this by dividing the step size by 2 and then searching again.
      minXRotationDegrees = bestXRotationDegrees - stepSize;
      maxXRotationDegrees = bestXRotationDegrees + stepSize;
      minZRotationDegrees = bestZRotationDegrees - stepSize;
      maxZRotationDegrees = bestZRotationDegrees + stepSize;
      stepSize /= 2.0;

    } while (stepSize >= precisionDegrees / 2);

    // Make sure we haven't wrapped around the angles.
    if (bestXRotationDegrees < -180) { bestXRotationDegrees += 360; }
    if (bestXRotationDegrees > 180) { bestXRotationDegrees -= 360; }
    if (bestZRotationDegrees < -180) { bestZRotationDegrees += 360; }
    if (bestZRotationDegrees > 180) { bestZRotationDegrees -= 360; }

    outXRotationDegrees = bestXRotationDegrees;
    outZRotationDegrees = bestZRotationDegrees;
  }
  else { // not rotateXFirst
    // First solve for the X angle that points the specific pixel at the height of the target
    // when it passes through the cylinder at the radius of the target from the center of the world.
    // Do this by brute-force testing for the minimum miss distance across the specified precision
    // across the specified range.
    double minMissDistance = 1e30;
    double bestXRotation = 0.0;

    // To avoid toggling back randomly between two solutions, we keep track of whether we have passed
    // the target height or not.  If we have passed it, we can stop looking for a solution.
    int previousSign = 0;

    glm::dvec2 bestPierce = { 0, 0 };
    for (double xRotation = minXRotationDegrees; xRotation <= maxXRotationDegrees; xRotation += precisionDegrees) {
      glm::dvec3 rayStartInWorld, rayDirectionInWorld;
      WorldSpaceRayNoDistortion(cri, xPixels, yPixels, rotateXFirst, 0.0, xRotation, rayStartInWorld, rayDirectionInWorld, false);

      // The ray is defined by the start and direction.
      // Find the location along the ray that pierces the vertical cylinder whose radius matches
      // that of the target.  The error is the absolute difference between the height of the target
      // and the height of the piercing point.
      // The equation of the cylinder in the plane is the circle x^2 + y^2 = r^2.
      // Its center is (0,0) and its radius is r.
      // The ray is defined by the start (U) and direction in the plane (V).
      // From Graphics Gems p 5-6, the intersection of the ray and the circle is
      // G <- U - C = U because the circle is centered at the origin.
      // a <- V dot V
      // b <- 2 * V dot G
      // c <- G dot G - r^2
      // d <- b^2 - 4ac
      // If d < 0, there is no intersection.
      // P1 (corresponding to t1 on the line?) <- (-b + sqrt(d)) / 2a
      // P2 (corresponding to t2 on the line?) <- (-b - sqrt(d)) / 2a
      // We want P1, further along the line, which is at U + P1 * V.
      double rad = glm::length(glm::dvec2(target.x, target.y));
      glm::dvec2 U = glm::dvec2(rayStartInWorld.x, rayStartInWorld.y);
      glm::dvec2 V = glm::dvec2(rayDirectionInWorld.x, rayDirectionInWorld.y);
      if (glm::length(V) < 1e-6) {
        continue; // No direction, so no intersection.
      }
      glm::dvec2 G = U;
      double a = glm::dot(V, V);
      double b = 2 * glm::dot(V, G);
      double c = glm::dot(G, G) - rad * rad;
      double d = b * b - 4 * a * c;
      if (d < 0) {
        continue;
      }
      double P1 = (-b + sqrt(d)) / (2 * a);
      glm::dvec2 intersect = U + P1 * V;
      // Knowing the distance to the (X,Y) location, solve for Z at the same distance along the ray.
      double height = rayStartInWorld.z + P1 * rayDirectionInWorld.z;
      double missDistance = fabs(height - target.z);
      if (missDistance < minMissDistance) {
        minMissDistance = missDistance;
        bestXRotation = xRotation;
        bestPierce = intersect;
      }

      // If we have passed the target height, then our minimum so far is within the precision and
      // we us it.  The previous sign starts out at 0, so we won't break on until we've had one that
      // is non-negative and one that is negative.
      int sign = (height >= target.z) ? 1 : -1;
      if (sign * previousSign == -1) {
        break;
      }
      previousSign = sign;
    }

    // Find the angle in the Z=0 plane of the piercing point.
    double zRotation = atan2(bestPierce.y, bestPierce.x);

    // Find the angle in the Z=0 plane of the target.
    double targetZRotation = atan2(target.y, target.x);

    // The difference between the two is the Z rotation.
    double zDelta = glm::degrees(targetZRotation - zRotation);
    if (zDelta < -180) { zDelta += 360; }
    if (zDelta > 180) { zDelta -= 360; }

    outZRotationDegrees = zDelta;
    outXRotationDegrees = bestXRotation;
  }
}

bool asdp::render::calibration::TargetProjectedLocationNoDistortion(
  const asdp::render::CameraRenderInfo& cri,
  bool rotateXFirst,
  double zRotationDegrees, double xRotationDegrees, const glm::dvec3& targetPoint,
  double& xPixels, double& yPixels)
{
  // Set return to defaults for failure in case of failure.
  xPixels = -1e6;
  yPixels = -1e6;

  // Shift the target location from world space to camera space, which is done by performing
  // the inverse of the gimbal rotation and then translating by the inverse of its position
  // and then the inverse of the camera rotation.
  // We do the opposite rotations in the opposite order to get the inverse.
  glm::dvec3 targetPointInCamera = targetPoint;
  {
    // Rotate the target point by the INVERSE of the gimbal rotation (negative and backwards order).
    glm::dquat rotationZ = glm::angleAxis(glm::radians(-zRotationDegrees), glm::dvec3(0.0, 0.0, 1.0));
    glm::dquat rotationX = glm::angleAxis(glm::radians(-xRotationDegrees), glm::dvec3(1.0, 0.0, 0.0));
    glm::dquat rotationTotal;
    if (rotateXFirst) {
      rotationTotal = rotationZ * rotationX;
    } else {
      rotationTotal = rotationX * rotationZ;
    }
    targetPointInCamera = rotationTotal * targetPointInCamera;
  }
  {
    // Translate the target point by the inverse of the camera position.
    targetPointInCamera.x -= cri.m_positionMeters[0];
    targetPointInCamera.y -= cri.m_positionMeters[1];
    targetPointInCamera.z -= cri.m_positionMeters[2];
  }
  {
    // Rotate the target point by the inverse of the camera rotation.
    glm::dquat rotationX = glm::angleAxis(glm::radians(-cri.m_orientationDegrees[0]), glm::dvec3(1.0, 0.0, 0.0));
    glm::dquat rotationY = glm::angleAxis(glm::radians(-cri.m_orientationDegrees[1]), glm::dvec3(0.0, 1.0, 0.0));
    glm::dquat rotationZ = glm::angleAxis(glm::radians(-cri.m_orientationDegrees[2]), glm::dvec3(0.0, 0.0, 1.0));
    glm::dquat rotationTotal = rotationZ * rotationY * rotationX;
    targetPointInCamera = rotationTotal * targetPointInCamera;
  }

  // Switch from helicopter space (+X right, +Y forward, +Z up) to camera space (+X right, +Y up, -Z forward).
  glm::dvec3 temp = targetPointInCamera;
  targetPointInCamera.x = temp.x;
  targetPointInCamera.y = temp.z;
  targetPointInCamera.z = -temp.y;

  // If the target is at or behind the camera (along +Z), it is not in the frustum.
  if (targetPointInCamera.z >= 0) {
    return false;
  }

  // Project the target point onto the Z = -1 plane.
  // The projection is done by scaling the X and Y coordinates by the -Z coordinate.
  targetPointInCamera.x /= -targetPointInCamera.z;
  targetPointInCamera.y /= -targetPointInCamera.z;
  targetPointInCamera.z = -1;

  // Find the four edges of the camera frustum in the Z = -1 plane.
  double maxX = tan(glm::radians(cri.m_fovDegrees[0] / 2.0));
  double minX = -maxX;
  double maxY = tan(glm::radians(cri.m_fovDegrees[1] / 2.0));
  double minY = -maxY;

  // Invert the Y coordinate to match the right-handed coordinate system of the image.
  targetPointInCamera.y = -targetPointInCamera.y;

  // Compute the normalized coordinates of the target point (0 at min and 1 at max).
  double xFrac = (targetPointInCamera.x - minX) / (maxX - minX);
  double yFrac = (targetPointInCamera.y - minY) / (maxY - minY);

  if (xFrac < 0 || xFrac > 1 || yFrac < 0 || yFrac > 1) {
    // The target point is outside the camera frustum.
    return false;
  }

  // Compute the pixel coordinates of the target point.
  // The pixel coordinates are in the range [0, width-1] and [0, height-1].
  xPixels = xFrac * (cri.m_resolutionPixels[0] - 1);
  yPixels = yFrac * (cri.m_resolutionPixels[1] - 1);
  return true;
}

std::string asdp::render::calibration::Test()
{
  // Used in multiple tests.
  std::shared_ptr<Distortion> distNull = std::make_shared<DistortionNone>();
  std::shared_ptr<Vignette> vigNull = std::make_shared<VignetteNone>();
  glm::dvec3 rayStart, rayDirection;

  // Test WorldSpaceRayNoDistortion()
  {
    {
      //===========================================================================
      // Make a square camera with a 90 degree field of view offset one unit in Y
      // with no rotation.
      // Test different pixels at different gimbal rotations for z-rotation first.
      CameraRenderInfo cri(1, { 0, 1, 0 }, { 0, 0, 0 }, { 1024, 1024 }, { 90, 90 },
        distNull, vigNull, nullptr, 1.0);

      // Center of the image, halfway between center pixels, no rotation.
      WorldSpaceRayNoDistortion(cri, 511.5, 511.5, false, 0, 0, rayStart, rayDirection);
      if (glm::length(rayStart - glm::dvec3(0, 1, 0)) > 1e-6) {
        return "Test failed: WorldSpaceRayNoDistortion() center of image ray start.";
      }
      if (glm::length(rayDirection - glm::dvec3(0, 1, 0)) > 1e-6) {
        return "Test failed: WorldSpaceRayNoDistortion() center of image ray direction.";
      }

      // Right center edge (halfway past last pixel), no rotation.
      WorldSpaceRayNoDistortion(cri, 1023.5, 511.5, false, 0, 0, rayStart, rayDirection);
      if (glm::length(rayStart - glm::dvec3(0, 1, 0)) > 1e-6) {
        return "Test failed: WorldSpaceRayNoDistortion() right center edge ray start.";
      }
      if (glm::length(rayDirection - glm::normalize(glm::dvec3(1, 1, 0))) > 1e-6) {
        return "Test failed: WorldSpaceRayNoDistortion() right center edge ray direction: "
          + std::to_string(rayDirection[0]) + ", " + std::to_string(rayDirection[1])
          + ", " + std::to_string(rayDirection[2]);
      }

      // Left center edge (halfway past first pixel), no rotation.
      WorldSpaceRayNoDistortion(cri, -0.5, 511.5, false, 0, 0, rayStart, rayDirection);
      if (glm::length(rayStart - glm::dvec3(0, 1, 0)) > 1e-6) {
        return "Test failed: WorldSpaceRayNoDistortion() left center edge ray start.";
      }
      if (glm::length(rayDirection - glm::normalize(glm::dvec3(-1, 1, 0))) > 1e-6) {
        return "Test failed: WorldSpaceRayNoDistortion() left center edge ray direction.";
      }

      // Top center edge (halfway past last pixel), no rotation.
      WorldSpaceRayNoDistortion(cri, 511.5, -0.5, false, 0, 0, rayStart, rayDirection);
      if (glm::length(rayStart - glm::dvec3(0, 1, 0)) > 1e-6) {
        return "Test failed: WorldSpaceRayNoDistortion() top center edge ray start.";
      }
      if (glm::length(rayDirection - glm::normalize(glm::dvec3(0, 1, 1))) > 1e-6) {
        return "Test failed: WorldSpaceRayNoDistortion() top center edge ray direction: "
          + std::to_string(rayDirection[0]) + ", " + std::to_string(rayDirection[1])
          + ", " + std::to_string(rayDirection[2]);
      }

      // Bottom center edge (halfway past first pixel), no rotation.
      WorldSpaceRayNoDistortion(cri, 511.5, 1023.5, false, 0, 0, rayStart, rayDirection);
      if (glm::length(rayStart - glm::dvec3(0, 1, 0)) > 1e-6) {
        return "Test failed: WorldSpaceRayNoDistortion() bottom center edge ray start.";
      }
      if (glm::length(rayDirection - glm::normalize(glm::dvec3(0, 1, -1))) > 1e-6) {
        return "Test failed: WorldSpaceRayNoDistortion() bottom center edge ray direction.";
      }

      // Center pixel after rotating gimbal 90 degrees around Z.
      WorldSpaceRayNoDistortion(cri, 511.5, 511.5, false, 90, 0, rayStart, rayDirection);
      if (glm::length(rayStart - glm::dvec3(-1, 0, 0)) > 1e-6) {
        return "Test failed: WorldSpaceRayNoDistortion() center of image rotated 90 degrees Z ray start.";
      }
      if (glm::length(rayDirection - glm::dvec3(-1, 0, 0)) > 1e-6) {
        return "Test failed: WorldSpaceRayNoDistortion() center of image rotated 90 degrees Z ray direction.";
      }

      // Center pixel after rotating gimbal 90 degrees around X.
      WorldSpaceRayNoDistortion(cri, 511.5, 511.5, false, 0, 90, rayStart, rayDirection);
      if (glm::length(rayStart - glm::dvec3(0, 0, 1)) > 1e-6) {
        return "Test failed: WorldSpaceRayNoDistortion() center of image rotated 90 degrees X ray start.";
      }
      if (glm::length(rayDirection - glm::dvec3(0, 0, 1)) > 1e-6) {
        return "Test failed: WorldSpaceRayNoDistortion() center of image rotated 90 degrees X ray direction.";
      }
    }

    {
      //===========================================================================
      // Make a square camera with a 90 degree field of view offset one unit in Y
      // with rotation around Y by 90 degrees.
      // Test different pixels at different gimbal rotations, rotating around Z first.
      CameraRenderInfo cri(1, { 0, 1, 0 }, { 0, 90, 0 }, { 1024, 1024 }, { 90, 90 },
        distNull, vigNull, nullptr, 1.0);

      // Center of the image, halfway between center pixels, no rotation.
      WorldSpaceRayNoDistortion(cri, 511.5, 511.5, false, 0, 0, rayStart, rayDirection);
      if (glm::length(rayStart - glm::dvec3(0, 1, 0)) > 1e-6) {
        return "Test failed: WorldSpaceRayNoDistortion() center of 90Y image ray start.";
      }
      if (glm::length(rayDirection - glm::dvec3(0, 1, 0)) > 1e-6) {
        return "Test failed: WorldSpaceRayNoDistortion() center of 90Y image ray direction.";
      }

      // Right center right pixel, which should be rotated down.
      WorldSpaceRayNoDistortion(cri, 1023.5, 511.5, false, 0, 0, rayStart, rayDirection);
      if (glm::length(rayStart - glm::dvec3(0, 1, 0)) > 1e-6) {
        return "Test failed: WorldSpaceRayNoDistortion() right center right pixel 90Y image ray start.";
      }
      if (glm::length(rayDirection - glm::normalize(glm::dvec3(0, 1, -1))) > 1e-6) {
        return "Test failed: WorldSpaceRayNoDistortion() right center right pixel 90Y image ray direction: "
          + std::to_string(rayDirection[0]) + ", " + std::to_string(rayDirection[1])
          + ", " + std::to_string(rayDirection[2]);
      }

      // Right center pixel again, but rotating the gimbal 90 degrees around Z.
      WorldSpaceRayNoDistortion(cri, 1023.5, 511.5, false, 90, 0, rayStart, rayDirection);
      if (glm::length(rayStart - glm::dvec3(-1, 0, 0)) > 1e-6) {
        return "Test failed: WorldSpaceRayNoDistortion() right center right pixel 90Y image rotated 90Z ray start.";
      }
      if (glm::length(rayDirection - glm::normalize(glm::dvec3(-1, 0, -1))) > 1e-6) {
        return "Test failed: WorldSpaceRayNoDistortion() right center right pixel 90Y image rotated 90Z ray direction: "
          + std::to_string(rayDirection[0]) + ", " + std::to_string(rayDirection[1])
          + ", " + std::to_string(rayDirection[2]);
      }
    }

    {
      //===========================================================================
      // Test rotating first around Z and first around X to ensure that we get different
      // (and expected) results.
      CameraRenderInfo cri(1, { 0, 0, 0 }, { 0, 0, 0 }, { 1024, 1024 }, { 90, 90 },
        distNull, vigNull, nullptr, 1.0);

      // Center of the image, halfway between center pixels, Z-first rotation.
      WorldSpaceRayNoDistortion(cri, 511.5, 511.5, false, 90, 90, rayStart, rayDirection);
      if (glm::length(rayDirection - glm::dvec3(0, 0, 1)) > 1e-6) {
        return "Test failed: WorldSpaceRayNoDistortion() center of image ray direction Z first.";
      }

      // Center of the image, halfway between center pixels, X-first rotation.
      WorldSpaceRayNoDistortion(cri, 511.5, 511.5, true, 90, 90, rayStart, rayDirection);
      if (glm::length(rayDirection - glm::dvec3(-1, 0, 0)) > 1e-6) {
        return "Test failed: WorldSpaceRayNoDistortion() center of image ray direction X first.";
      }
    }
  }

  // Test PointPixelAtTarget()
  {
    double zRotation, xRotation;
    {
      //===========================================================================
      // Test a camera at the center and a target down the +Y axis at the same height
      // when rotating around Z first.
      CameraRenderInfo cri(1, { 0, 0, 0 }, { 0, 0, 0 }, { 1024, 1024 }, { 90, 90 },
        distNull, vigNull, nullptr, 1.0);

      // Center of the image, no rotation.
      PointPixelAtTargetNoDistortion(cri, 511.5, 511.5, -60, 240, { 0, 3, 0 }, true, zRotation, xRotation, 0.01);
      if (fabs(zRotation) > 0.01) {
        return "Test failed: PointPixelAtTargetNoDistortion() center of image Z first no rotation Z.";
      }
      if (fabs(xRotation) > 0.01) {
        return "Test failed: PointPixelAtTargetNoDistortion() center of image Z first no rotation X.";
      }

      // Right side of the image, should be rotated by 45 degrees around Z.
      PointPixelAtTargetNoDistortion(cri, 1023.5, 511.5, -60, 240, { 0, 3, 0 }, false, zRotation, xRotation, 0.01);
      if (fabs(zRotation - 45) > 0.01) {
        return "Test failed: PointPixelAtTargetNoDistortion() right side of image Z first no rotation Z.";
      }
      if (fabs(xRotation) > 0.01) {
        return "Test failed: PointPixelAtTargetNoDistortion() right side of image Z first no rotation X.";
      }

      // Top of the image, should be rotated by -45 degrees around X.
      PointPixelAtTargetNoDistortion(cri, 511.5, -0.5, -60, 240, { 0, 3, 0 }, false, zRotation, xRotation, 0.01);
      if (fabs(zRotation) > 0.01) {
        return "Test failed: PointPixelAtTargetNoDistortion() top of image Z first no rotation Z.";
      }
      if (fabs(xRotation - (-45)) > 0.01) {
        return "Test failed: PointPixelAtTargetNoDistortion() top of image Z first no rotation X: "
          + std::to_string(xRotation);
      }
    }

    {
      //===========================================================================
      // Test a camera that is offset by +1 down the Y axis and rotated -90 degrees
      // around Z so that it is looking to the right.  Using a target at the same height
      // that is -1 in X and +3 in Y should require a gimbal rotation of 90 degrees
      // around Z.
      CameraRenderInfo cri(1, { 0, 1, 0 }, { 0, 0, -90 }, { 1024, 1024 }, { 90, 90 },
        distNull, vigNull, nullptr, 1.0);

      // Center of the image, no rotation and rotating around Z first.
      PointPixelAtTargetNoDistortion(cri, 511.5, 511.5, -60, 240, { -1, 3, 0 }, false, zRotation, xRotation, 0.01);
      if (fabs(zRotation - 90) > 0.01) {
        return "Test failed: PointPixelAtTargetNoDistortion() center of image Z first rotated 90Z Z: "
          + std::to_string(zRotation);
      }

      {
        //===========================================================================
        // Test a camera at the center and a target down the +Y axis at the same height
        // when rotating around X first.
        CameraRenderInfo cri(1, { 0, 0, 0 }, { 0, 0, 0 }, { 1024, 1024 }, { 90, 90 },
          distNull, vigNull, nullptr, 1.0);

        // Center of the image, no rotation.
        PointPixelAtTargetNoDistortion(cri, 511.5, 511.5, -60, 60, { 0, 3, 0 }, true, zRotation, xRotation, 0.01);
        if (fabs(zRotation) > 0.01) {
          return "Test failed: PointPixelAtTargetNoDistortion() center of image X first no rotation Z.";
        }
        if (fabs(xRotation) > 0.01) {
          return "Test failed: PointPixelAtTargetNoDistortion() center of image no rotation X.";
        }

        // Right side of the image, should be rotated by 45 degrees around Z.
        PointPixelAtTargetNoDistortion(cri, 1023.5, 511.5, -60, 60, { 0, 3, 0 }, true, zRotation, xRotation, 0.01);
        if (fabs(zRotation - 45) > 0.01) {
          return "Test failed: PointPixelAtTargetNoDistortion() right side of image X first no rotation Z.";
        }
        if (fabs(xRotation) > 0.01) {
          return "Test failed: PointPixelAtTargetNoDistortion() right side of image X first no rotation X.";
        }

        // Top of the image, should be rotated by -45 degrees around X.
        PointPixelAtTargetNoDistortion(cri, 511.5, -0.5, -60, 60, { 0, 3, 0 }, true, zRotation, xRotation, 0.01);
        if (fabs(zRotation) > 0.01) {
          return "Test failed: PointPixelAtTargetNoDistortion() top of image X first no rotation Z.";
        }
        if (fabs(xRotation - (-45)) > 0.01) {
          return "Test failed: PointPixelAtTargetNoDistortion() top of image X first no rotation X: "
            + std::to_string(xRotation);
        }
      }

      {
        //===========================================================================
        // Test a camera that is offset by +1 down the Y axis and rotated -90 degrees
        // around Z so that it is looking to the right.  Using a target at the same height
        // that is -1 in X and +3 in Y should require a gimbal rotation of 90 degrees
        // around Z.
        CameraRenderInfo cri(1, { 0, 1, 0 }, { 0, 0, -90 }, { 1024, 1024 }, { 90, 90 },
          distNull, vigNull, nullptr, 1.0);

        // Center of the image, no rotation and rotating around Z first.
        PointPixelAtTargetNoDistortion(cri, 511.5, 511.5, -60, 60, { -1, 3, 0 }, true, zRotation, xRotation, 0.01);
        if (fabs(zRotation - 90) > 0.01) {
          return "Test failed: PointPixelAtTargetNoDistortion() center of image X first rotated 90Z Z: "
            + std::to_string(zRotation);
        }
      }
    }

    // Test TargetProjectedLocationNoDistortion()
    {
      double xPixel, yPixel;
      {
        //===========================================================================
        // Test a camera at the center and a target down the +Y axis at the same height
        // that rotates around Z first.
        CameraRenderInfo cri(1, { 0, 0, 0 }, { 0, 0, 0 }, { 1024, 1024 }, { 90, 90 },
          distNull, vigNull, nullptr, 1.0);

        // No gimbal rotation, center of the image.
        if (!TargetProjectedLocationNoDistortion(cri, false, 0, 0, { 0, 3, 0 }, xPixel, yPixel)) {
          return "Test failed: TargetProjectedLocationNoDistortion() center of image no rotation.";
        }
        if (fabs(xPixel - 511.5) > 0.01) {
          return "Test failed: TargetProjectedLocationNoDistortion() center of image no rotation X.";
        }
        if (fabs(yPixel - 511.5) > 0.01) {
          return "Test failed: TargetProjectedLocationNoDistortion() center of image no rotation Y.";
        }

        // No gimbal rotation, right edge of image.
        if (!TargetProjectedLocationNoDistortion(cri, false, 0, 0, { 2.9999999, 3, 0 }, xPixel, yPixel)) {
          return "Test failed: TargetProjectedLocationNoDistortion() right edge of image no rotation.";
        }
        if (fabs(xPixel - 1023) > 0.01) {
          return "Test failed: TargetProjectedLocationNoDistortion() right edge of image no rotation X.";
        }
        if (fabs(yPixel - 511.5) > 0.01) {
          return "Test failed: TargetProjectedLocationNoDistortion() right edge of image no rotation Y.";
        }

        // No gimbal rotation, outside of image.
        if (TargetProjectedLocationNoDistortion(cri, false, 0, 0, { 0, -3, 0 }, xPixel, yPixel)) {
          return "Test failed: TargetProjectedLocationNoDistortion() center of image no rotation behind.";
        }
        if (TargetProjectedLocationNoDistortion(cri, false, 0, 0, { 0, 3, 4 }, xPixel, yPixel)) {
          return "Test failed: TargetProjectedLocationNoDistortion() center of image no rotation out of range.";
        }
        if (fabs(xPixel - (-1e6)) > 0.01) {
          return "Test failed: TargetProjectedLocationNoDistortion() center of image no rotation out of range X.";
        }
        if (fabs(yPixel - (-1e6)) > 0.01) {
          return "Test failed: TargetProjectedLocationNoDistortion() center of image no rotation out of range Y.";
        }

        // 90 degree Z gimbal rotation, center of image.
        if (!TargetProjectedLocationNoDistortion(cri, false, 90, 0, { -3, 0, 0 }, xPixel, yPixel)) {
          return "Test failed: TargetProjectedLocationNoDistortion() center of image rotated 90Z.";
        }
        if (fabs(xPixel - 511.5) > 0.01) {
          return "Test failed: TargetProjectedLocationNoDistortion() center of image rotated 90Z X.";
        }
        if (fabs(yPixel - 511.5) > 0.01) {
          return "Test failed: TargetProjectedLocationNoDistortion() center of image rotated 90Z Y.";
        }

        // 90 degree X gimbal rotation, center of image.
        if (!TargetProjectedLocationNoDistortion(cri, false, 0, 90, { 0, 0, 3 }, xPixel, yPixel)) {
          return "Test failed: TargetProjectedLocationNoDistortion() center of image rotated 90X.";
        }

        // 90 gimbal rotation about X and Z with Z first, center of the image.
        if (!TargetProjectedLocationNoDistortion(cri, false, 90, 90, { 0, 0, 3 }, xPixel, yPixel)) {
          return "Test failed: TargetProjectedLocationNoDistortion() center of image 90/90 Z first.";
        }
        if (fabs(xPixel - 511.5) > 0.01) {
          return "Test failed: TargetProjectedLocationNoDistortion() center of image 90/90 Z first.";
        }
        if (fabs(yPixel - 511.5) > 0.01) {
          return "Test failed: TargetProjectedLocationNoDistortion() center of image 90/90 Z first.";
        }

        // 90 gimbal rotation about X and Z with X first, center of the image.
        if (!TargetProjectedLocationNoDistortion(cri, true, 90, 90, { -3, 0, 0 }, xPixel, yPixel)) {
          return "Test failed: TargetProjectedLocationNoDistortion() center of image 90/90 X first.";
        }
        if (fabs(xPixel - 511.5) > 0.01) {
          return "Test failed: TargetProjectedLocationNoDistortion() center of image 90/90 X first.";
        }
        if (fabs(yPixel - 511.5) > 0.01) {
          return "Test failed: TargetProjectedLocationNoDistortion() center of image 90/90 X first.";
        }
      }

      {
        //===========================================================================
        // Test a camera that is offset by +1 down the Y axis and rotated 90 degrees
        // that rotates around Z first.
        CameraRenderInfo cri(1, { 0, 1, 0 }, { 0, 0, 90 }, { 1024, 1024 }, { 90, 90 },
          distNull, vigNull, nullptr, 1.0);

        // No gimbal rotation, center of the image.
        if (!TargetProjectedLocationNoDistortion(cri, false, 0, 0, { -1, 1, 0 }, xPixel, yPixel)) {
          return "Test failed: TargetProjectedLocationNoDistortion() offset angled center of image.";
        }
        if (fabs(xPixel - 511.5) > 0.01) {
          return "Test failed: TargetProjectedLocationNoDistortion() offset angled center of image X.";
        }
        if (fabs(yPixel - 511.5) > 0.01) {
          return "Test failed: TargetProjectedLocationNoDistortion() offset angled center of image Y.";
        }

      }
    }
  }

  return "";
}
