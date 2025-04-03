/*
 * Copyright (C) 2025: Arizona Board of Regents on Behalf of the University of Arizona
 */

/**
 * @file Camera_Calibration_Make_Scan.cpp
 * @brief Apache Strap-Down Pilotage configuration calibration program.
 *
* @author ReliaSolve.
* @date April 3rd, 2025.
*/

#include <iostream>
#include <fstream>
#include <string>
#include <filesystem>
#include <vector>
#include <set>
#include <cmath>
#include <CameraRenderInfo.h>
#include <nlohmann/json.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include "Calibration_Helpers.h"

using namespace asdp;
using namespace asdp::render;
using namespace asdp::render::calibration;

static std::string VERSION = "0.1.0";

void usage(std::string name)
{
  std::cerr << "Usage: " << name << " [options] camConfig.json targetConfig.json" << std::endl;
  std::cerr << "  camConfig.json                Camera configuration file." << std::endl;
  std::cerr << "  targetConfig.json             Target configuration file." << std::endl;
  std::cerr << "  Options:" << std::endl;
  std::cerr << "    --frames <int>              Number of frames per location (default 10)." << std::endl;
  std::cerr << "    --step <float>              Step size in degrees (default 1.0)." << std::endl;
  std::cerr << "    --help                      Print this information and quit." << std::endl;
  std::cerr << "  Writes poses.csv file." << std::endl;
};

int main(int argc, char** argv)
{
  std::string camConfigFile, targetConfigFile;
  int frames = 10;
  double step = 1.0;
  size_t realParams = 0;          ///< The number of non-flag parameters we've seen.

  // Parse the command line arguments, with the first non-flag argument being the
  // name of the IP address to listen on.
  for (int i = 1; i < argc; ++i) {
    if (std::string("--help") == argv[i]) {
      usage(argv[0]);
    } else if (std::string("--frames") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 1;
      }
      frames = std::stoi(argv[i]);
    } else if (std::string("--step") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 1;
      }
      step = std::stod(argv[i]);
    } else if (argv[i][0] == '-') {
      usage(argv[0]);
      return 1;
    } else switch (realParams++) {
    case 0:
      camConfigFile = argv[i];
      break;
    case 1:
      targetConfigFile = argv[i];
      break;
    default:
      usage(argv[0]);
      return 2;
    }
  }
  if (realParams != 2) {
    usage(argv[0]);
    return 2;
  }

  // Run inside a block so that the destructors will be called for all objects before we exit.
  {
    std::cout << "Camera_Calibration_Make_Scan version " << VERSION << std::endl;

    // Construct CameraRenderInfos for each configuration file.
    std::vector<asdp::render::CameraRenderInfo> cameraRenderInfos;
    try {
      cameraRenderInfos = asdp::render::calibration::GetCameraRenderInfos(camConfigFile);
    }
    catch (const std::exception& e) {
      std::cerr << "Error: Unable to read camera configuration file: " << e.what() << std::endl;
      return 10;
    }
    std::cout << "Read camera configuration from " << camConfigFile << std::endl;

    // Read the target information for each target.
    std::vector<TargetInfo> targetInfos;
    try {
      targetInfos = asdp::render::calibration::GetTargetInfos(targetConfigFile);
    }
    catch (const std::exception& e) {
      std::cerr << "Error: Unable to read target configuration file: " << e.what() << std::endl;
      return 11;
    }
    std::cout << "Read target configuration from " << targetConfigFile << std::endl;

    /// @todo

    // Find the largest field of view (either horizontal or vertical) in any of the
    // cameras.
    double maxFov = 0;
    for (const auto& camera : cameraRenderInfos) {
      maxFov = std::max(maxFov, camera.m_fovDegrees[0]);
      maxFov = std::max(maxFov, camera.m_fovDegrees[1]);
    }
    std::cout << std::endl;
    std::cout << "Maximum field of view: " << maxFov << " degrees" << std::endl;

    // Find the maximum and minimum angle away from the +Y axis that any
    // camera points.  Do this by transforming the +Y axis by the camera orientation
    // and then finding the angle between the resulting angle in the XZ plane of
    // the transformed axis.
    // Find the maximum and minimum angle above and below the XY plane that any rotated +Y axis goes.
    double minHAngle = 180;
    double maxHAngle = -180;
    double minVAngle = 180;
    double maxVAngle = -180;
    for (const auto& camera : cameraRenderInfos) {
      // Rotate around X, then Y, then Z.
      glm::dquat rotationX = glm::angleAxis(glm::radians(camera.m_orientationDegrees[0]), glm::dvec3(1.0, 0.0, 0.0));
      glm::dquat rotationY = glm::angleAxis(glm::radians(camera.m_orientationDegrees[1]), glm::dvec3(0.0, 1.0, 0.0));
      glm::dquat rotationZ = glm::angleAxis(glm::radians(camera.m_orientationDegrees[2]), glm::dvec3(0.0, 0.0, 1.0));
      glm::dquat rotationTotal = rotationX * rotationY * rotationZ;

      // Rotate the Y axis by the quaternion.
      glm::dvec3 y(0, 1, 0);
      glm::dvec3 newY = rotationTotal * y;

      // The atan2 arguments are swapped here: X is along +Y and Y is along -X.
      double angle = glm::degrees(std::atan2(-newY.x, newY.y));
      minHAngle = std::min(minHAngle, angle);
      maxHAngle = std::max(maxHAngle, angle);

      // This is a normalized vector, so the Z component is the sine of the angle.
      double angle2 = glm::degrees(std::asin(newY.z));
      minVAngle = std::min(minVAngle, angle2);
      maxVAngle = std::max(maxVAngle, angle2);
    }
    std::cout << std::endl;
    std::cout << "Minimum horizontal angle from +Y axis: " << minHAngle << " degrees" << std::endl;
    std::cout << "Maximum horizontal angle from +Y axis: " << maxHAngle << " degrees" << std::endl;

    minHAngle -= maxFov / 2;
    maxHAngle += maxFov / 2;
    std::cout << "Adjusted horizontal minimum angle: " << minHAngle << " degrees" << std::endl;
    std::cout << "Adjusted horizontal maximum angle: " << maxHAngle << " degrees" << std::endl;

    std::cout << std::endl;
    std::cout << "Minimum vertical angle from XY plane: " << minVAngle << " degrees" << std::endl;
    std::cout << "Maximum vertical angle from XY plane: " << maxVAngle << " degrees" << std::endl;

    minVAngle -= maxFov / 2;
    maxVAngle += maxFov / 2;
    std::cout << "Adjusted vertical minimum angle: " << minVAngle << " degrees" << std::endl;
    std::cout << "Adjusted vertical maximum angle: " << maxVAngle << " degrees" << std::endl;

    std::string filename = "poses.csv";
    std::cout << std::endl;
    std::cout << "Writing poses to " << filename << std::endl;
    std::ofstream outFile(filename);
    if (!outFile) {
      std::cerr << "Error: Unable to open output file " << filename << std::endl;
      return 20;
    }
    // Write the header line.
    outFile << "FrameIndex,ZRotationDegrees,XRotationDegrees,Camera,NumFrames" << std::endl;

    // For each target, generate a series of poses, writing them to the file.
    // When more than one camera can see the same target, request images from all of them
    // using the same frame index.
    int frameIndex = 0;
    for (size_t i = 0; i < targetInfos.size(); i++) {
      const TargetInfo& target = targetInfos[i];
      // Determine the angle of the target in the XY plane, with 0 degrees being the +Y axis
      // and 90 degrees being the -X axis.
      double targetHAngle = glm::degrees(std::atan2(-target.position.x, target.position.y));
      std::cout << " Target horizontal angle: " << targetHAngle << " degrees" << std::endl;

      // Determine the angle of the target out of the XY plane.
      double targetVAngle = glm::degrees(std::asin(target.position.z));
      std::cout << " Target vertical angle: " << targetVAngle << " degrees" << std::endl;

      // The specified transforms are applied in the order rotation around Z followed by
      // rotation around X.

      // Generate two types of poses.  The first goes horizontally across the target
      // covering the full range that might possibly be covered by any camera.  The second
      // goes vertically across the target, covering the full range that might possibly be
      // covered by any camera.  They step with the specified interval in degrees.
      // These poses are used to determine the set of cameras that are the closest match across
      // some part of this sweep -- we then use the centers of those cameras to generate the
      // list of gimbal angle + camera to use.
      std::set<uint16_t> camerasUsed;
      for (double a = targetHAngle + minHAngle; a <= targetHAngle + maxHAngle; a += step) {

        // Determine the ID of the camera whose +Y axis has the largest dot product with the specified
        // transform.  This is the one we'll ask for images from.  Note that we must rotate by
        // the inverse gimbal transform to line it up with the camera's vector when the stage is
        // rotated (it will rotate the camera vector to the origin).
        glm::dquat rotationZ = glm::angleAxis(glm::radians(a), glm::dvec3(0.0, 0.0, 1.0));
        glm::dquat rotationX = glm::angleAxis(glm::radians(targetHAngle), glm::dvec3(1.0, 0.0, 0.0));
        glm::dquat rotationTotal = rotationZ * rotationX;
        glm::dvec3 targetY = glm::inverse(rotationTotal) * glm::dvec3(0, 1, 0);

        size_t whichCamera = 0;
        double bestDot = -2;
        for (size_t c = 0; c < cameraRenderInfos.size(); c++) {
          auto const& camera = cameraRenderInfos[c];
          glm::dquat cameraRotationX = glm::angleAxis(glm::radians(camera.m_orientationDegrees[0]), glm::dvec3(1.0, 0.0, 0.0));
          glm::dquat cameraRotationY = glm::angleAxis(glm::radians(camera.m_orientationDegrees[1]), glm::dvec3(0.0, 1.0, 0.0));
          glm::dquat cameraRotationZ = glm::angleAxis(glm::radians(camera.m_orientationDegrees[2]), glm::dvec3(0.0, 0.0, 1.0));
          glm::dquat cameraRotationTotal = cameraRotationX * cameraRotationY * cameraRotationZ;
          glm::dvec3 cameraY = cameraRotationTotal * glm::dvec3(0, 1, 0);

          double dot = glm::dot(targetY, cameraY);
          if (dot > bestDot) {
            bestDot = dot;
            whichCamera = c;
          }
        }
        camerasUsed.insert(cameraRenderInfos[whichCamera].m_ID);
      }
      for (double a = targetVAngle + minVAngle; a < targetVAngle + maxVAngle; a += step) {
        // Determine the ID of the camera whose +Y axis has the largest dot product with the specified
        // transform after we rotate the whole camera so that its principal ray points in the
        // direction of the target in the XY plane (so we go straight up and down the middle).
        // This is the one we'll ask for images from.  Note that we must rotate by
        // the inverse gimbal transform to line it up with the camera's vector when the stage is
        // rotated (it will rotate the camera vector to the origin).
        //glm::dquat rotationZ = glm::angleAxis(glm::radians(targetHAngle), glm::dvec3(0.0, 0.0, 1.0));
        glm::dquat rotationZ = glm::angleAxis(glm::radians(0.0), glm::dvec3(0.0, 0.0, 1.0));
        glm::dquat rotationX = glm::angleAxis(glm::radians(a), glm::dvec3(1.0, 0.0, 0.0));
        glm::dquat rotationTotal = rotationZ * rotationX;
        glm::dvec3 targetY = glm::inverse(rotationTotal) * glm::dvec3(0, 1, 0);

        size_t whichCamera = 0;
        double bestDot = -2;
        for (size_t c = 0; c < cameraRenderInfos.size(); c++) {
          auto const& camera = cameraRenderInfos[c];
          glm::dquat cameraRotationX = glm::angleAxis(glm::radians(camera.m_orientationDegrees[0]), glm::dvec3(1.0, 0.0, 0.0));
          glm::dquat cameraRotationY = glm::angleAxis(glm::radians(camera.m_orientationDegrees[1]), glm::dvec3(0.0, 1.0, 0.0));
          glm::dquat cameraRotationZ = glm::angleAxis(glm::radians(camera.m_orientationDegrees[2]), glm::dvec3(0.0, 0.0, 1.0));
          glm::dquat cameraRotationTotal = cameraRotationX * cameraRotationY * cameraRotationZ;
          glm::dvec3 cameraY = cameraRotationTotal * glm::dvec3(0, 1, 0);

          double dot = glm::dot(targetY, cameraY);
          if (dot > bestDot) {
            bestDot = dot;
            whichCamera = c;
          }
        }
        camerasUsed.insert(cameraRenderInfos[whichCamera].m_ID);
      }

      // For each camera that was the closest in part of one of the sweeps, take an image using that
      // camera with the gimbal rotated to point the camera at the target.
      for (auto const &cri : cameraRenderInfos) if (camerasUsed.count(cri.m_ID)) {
        // Find the amount of rotation around Z that points the camera at the target.
        // This is the the angle between the target and the camera's Y axis.
        glm::dquat cameraRotationX = glm::angleAxis(glm::radians(cri.m_orientationDegrees[0]), glm::dvec3(1.0, 0.0, 0.0));
        glm::dquat cameraRotationY = glm::angleAxis(glm::radians(cri.m_orientationDegrees[1]), glm::dvec3(0.0, 1.0, 0.0));
        glm::dquat cameraRotationZ = glm::angleAxis(glm::radians(cri.m_orientationDegrees[2]), glm::dvec3(0.0, 0.0, 1.0));
        glm::dquat cameraRotationTotal = cameraRotationX * cameraRotationY * cameraRotationZ;
        glm::dvec3 cameraY = cameraRotationTotal * glm::dvec3(0, 1, 0);
        // The atan2 arguments are swapped here: X is along +Y and Y is along -X.
        double cameraAngle = glm::degrees(std::atan2(-cameraY.x, cameraY.y));
        double zRotation = targetHAngle - cameraAngle;

        // Find the amount of rotation about X that brings the camera Y axis into the XY plane
        // and subtract that from the angle that points at the target.
        // Here the Z axis corresponds to atan Y and the Y axis to atan X.
        double xRotation = targetVAngle;
        if (cameraY.y != 0) {
          xRotation -= glm::degrees(std::atan2(cameraY.z, cameraY.y));
        }

        outFile << ++frameIndex << "," << zRotation << "," << xRotation
          << "," << cri.m_ID << "," << frames << std::endl;
      }

    } // End of loop over targets.
    outFile.close();
  } // End of block to ensure that all objects are destructed before we exit.

  return 0;
}
