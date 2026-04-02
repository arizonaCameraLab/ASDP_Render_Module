/*
 * Copyright (C) 2025: Arizona Board of Regents on Behalf of the University of Arizona
 */

/**
 * @file Target_Calibration_Make_Scan.cpp
 * @brief Apache Strap-Down Pilotage configuration calibration program.
 *
* @author ReliaSolve.
* @date March 26th, 2025.
*/

#include <iostream>
#include <fstream>
#include <string>
#include <filesystem>
#include <vector>
#include <set>
#include <cmath>
#include <CameraRenderInfo.h>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include "Calibration_Helpers.h"

using namespace asdp;
using namespace asdp::render;
using namespace asdp::render::calibration;

static std::string VERSION = "5.0.1";

void usage(std::string name)
{
  std::cerr << "Usage: " << name << " [options] camConfig.json targetConfig.json gimbalConfig.json" << std::endl;
  std::cerr << "  camConfig.json                Camera configuration file." << std::endl;
  std::cerr << "  targetConfig.json             Target configuration file." << std::endl;
  std::cerr << "  gimbalConfig.json             Gimbal configuration file." << std::endl;
  std::cerr << "  Options:" << std::endl;
  std::cerr << "    --frames <int>              Number of frames per location (default 10)." << std::endl;
  std::cerr << "    --help                      Print this information and quit." << std::endl;
  std::cerr << "  Writes target_N_poses.csv files, where N goes from 1 through the number of targets." << std::endl;
};

int main(int argc, char** argv)
{
  std::string camConfigFile, targetConfigFile, gimbalConfigFile;
  int frames = 10;
  size_t realParams = 0;          ///< The number of non-flag parameters we've seen.

  // Parse the command line arguments, with the first non-flag argument being the
  // name of the IP address to listen on.
  for (int i = 1; i < argc; ++i) {
    if (std::string("--help") == argv[i]) {
      usage(argv[0]);
      return 0;
    } else if (std::string("--frames") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 1;
      }
      frames = std::stoi(argv[i]);
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
    case 2:
      gimbalConfigFile = argv[i];
      break;
    default:
      usage(argv[0]);
      return 2;
    }
  }
  if (realParams != 3) {
    usage(argv[0]);
    return 2;
  }

  // Run inside a block so that the destructors will be called for all objects before we exit.
  {
    std::cout << "Target_Calibration_Make_Scan version " << VERSION << std::endl;

    // Construct CameraRenderInfos for each configuration file.
    std::vector<asdp::render::CameraRenderInfo> cameraRenderInfos;
    try {
      cameraRenderInfos = asdp::render::calibration::GetCameraRenderInfos(camConfigFile);
    } catch (const std::exception& e) {
      std::cerr << "Error: Unable to read camera configuration file: " << e.what() << std::endl;
      return 10;
    }
    std::cout << "Read camera configuration from " << camConfigFile << std::endl;

    // Find the maximum and minimum angle away from the +Y axis that any
    // camera points.  Do this by transforming the +Y axis by the camera orientation
    // and then finding the angle between the resulting angle in the XZ plane of
    // the transformed axis.
    // Find the maximum and minimum angle above and below the XY plane that any rotated +Y axis goes.
    double minHAngle = 180;
    double maxHAngle = -180;
    double minVAngle = 90;
    double maxVAngle = -90;
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

    // Read the target information for each target.
    std::vector<TargetInfo> targetInfos;
    try {
      targetInfos = asdp::render::calibration::GetTargetInfos(targetConfigFile);
    } catch (const std::exception& e) {
      std::cerr << "Error: Unable to read target configuration file: " << e.what() << std::endl;
      return 11;
    }
    std::cout << "Read target configuration from " << targetConfigFile << std::endl;

    // Read the gimbal information.
    GimbalInfo gimbalInfo;
    try {
      gimbalInfo = asdp::render::calibration::GetGimbalInfo(gimbalConfigFile);
    }
    catch (const std::exception& e) {
      std::cerr << "Error: Unable to read gimbal configuration file: " << e.what() << std::endl;
      return 12;
    }

    // Add the offset to the camera positions.
    std::cout << "Adding cameraOffset: " << gimbalInfo.cameraOffset[0] << "," << gimbalInfo.cameraOffset[1] << "," << gimbalInfo.cameraOffset[2] << std::endl;
    for (auto& camera : cameraRenderInfos) {
      camera.m_positionMeters[0] += gimbalInfo.cameraOffset[0];
      camera.m_positionMeters[1] += gimbalInfo.cameraOffset[1];
      camera.m_positionMeters[2] += gimbalInfo.cameraOffset[2];
    }

    // For each target, generate a series of poses and write them to a file.
    for (size_t i = 0; i < targetInfos.size(); i++) {
      const TargetInfo& target = targetInfos[i];
      std::string filename = "target_" + std::to_string(target.id) + "_poses.csv";
      std::cout << std::endl;
      std::cout << "Writing poses for target " << target.id << " to " << filename << std::endl;
      std::ofstream outFile(filename);
      if (!outFile) {
        std::cerr << "Error: Unable to open output file " << filename << std::endl;
        return 20;
      }
      // Write the header line.
      outFile << "FrameIndex,ZRotationDegrees,XRotationDegrees,Camera,NumFrames,TargetID" << std::endl;

      // Determine the angle of the target in the XY plane, with 0 degrees being the +Y axis
      // and 90 degrees being the -X axis.
      double targetHAngle = glm::degrees(std::atan2(-target.position.x, target.position.y));
      std::cout << " Target horizontal angle: " << targetHAngle << " degrees" << std::endl;

      // Determine the angle of the target out of the XY plane.
      double targetVAngle = glm::degrees(std::asin(target.position.z));
      std::cout << " Target vertical angle: " << targetVAngle << " degrees" << std::endl;

      // The specified transforms are applied in the order rotation around Z followed by
      // rotation around X.

      // For each camera, take an image using that camera with the gimbal rotated to point its center at the target.

      int frameIndex = 0;
      for (auto const &cri : cameraRenderInfos) {

        double zRotationDegrees, xRotationDegrees;
        PointPixelAtTargetNoDistortion(cri,
          0.5 * cri.m_resolutionPixels[0] - 0.5, 0.5 * cri.m_resolutionPixels[1] - 0.5,
          minVAngle, maxVAngle,
          target.position,
          gimbalInfo.pitchFirst,
          zRotationDegrees, xRotationDegrees);

        outFile << ++frameIndex << "," << zRotationDegrees << "," << xRotationDegrees
          << "," << cri.m_ID << "," << frames << "," << target.id << std::endl;
      }

      outFile.close();

    } // End of loop over targets.
  } // End of block to ensure that all objects are destructed before we exit.

  return 0;
}
