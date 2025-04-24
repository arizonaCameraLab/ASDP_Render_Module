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

static std::string VERSION = "2.0.0";

void usage(std::string name)
{
  std::cerr << "Usage: " << name << " [options] camConfig.json targetConfig.json, gimbalConfigFile" << std::endl;
  std::cerr << "  camConfig.json                Camera configuration file." << std::endl;
  std::cerr << "  targetConfig.json             Target configuration file." << std::endl;
  std::cerr << "  gimbalConfig.json             Gimbal configuration file." << std::endl;
  std::cerr << "  Options:" << std::endl;
  std::cerr << "    --frames <int>              Number of frames per location (default 10)." << std::endl;
  std::cerr << "    --stepPixels <int>          Step size in pixels (default 30)." << std::endl;
  std::cerr << "    --topMarginPixels <int>     Margin away from top of image (default 10)." << std::endl;
  std::cerr << "    --bottomMarginPixels <int>  Margin away from bottom of image (default 10)." << std::endl;
  std::cerr << "    --leftMarginPixels <int>    Margin away from left of image (default 10)." << std::endl;
  std::cerr << "    --rightMarginPixels <int>   Margin away from right of image (default 10)." << std::endl;
  std::cerr << "    --help                      Print this information and quit." << std::endl;
  std::cerr << "  Writes poses.csv file." << std::endl;
};

static void RunAlongLine(std::ofstream& outFile, int& frameIndex,
  std::vector<CameraRenderInfo> const& cameraRenderInfos, int numFrames,
  CameraRenderInfo const& cri, glm::dvec3 const& targetPoint, int targetID,
  GimbalInfo const& gimbalInfo,
  int topMarginPixels, int bottomMarginPixels,
  int leftMarginPixels, int rightMarginPixels,
  int startX, int startY, int stepX, int stepY, int numSteps
  )
{
  for (int i = 0; i < numSteps; ++i) {
    // Compute the pixel location.
    double x = startX + i * stepX;
    double y = startY + i * stepY;

    // Compute the gimbal angles to point the camera at the target point.
    double xRotationDegrees, zRotationDegrees;
    PointPixelAtTargetNoDistortion(cri, x, y, gimbalInfo.minPitchDegrees, gimbalInfo.maxPitchDegrees,
      targetPoint, gimbalInfo.pitchFirst,
      zRotationDegrees, xRotationDegrees);

    // Check each camera to see if it can see the target within its margin.
    // If so, write the pose to the file.  We bump the frame index once for
    // all cameras and then reset it if no cameras saw the target (so we don't
    // have inadvertent gaps in the frame index).
    ++frameIndex;
    for (auto const& camera : cameraRenderInfos) {
      double xPixels, yPixels;
      TargetProjectedLocationNoDistortion(camera, gimbalInfo.pitchFirst, zRotationDegrees, xRotationDegrees, targetPoint,
        xPixels, yPixels);
      // The pixel is always in range in the reference camera, by construction.
      if (camera.m_ID == cri.m_ID ||
        (xPixels >= leftMarginPixels && xPixels <= (camera.m_resolutionPixels[0] - 1) - rightMarginPixels &&
        yPixels >= topMarginPixels && yPixels <= (camera.m_resolutionPixels[1] - 1) - bottomMarginPixels)) {
        outFile << frameIndex << "," << zRotationDegrees << "," << xRotationDegrees << ","
          << camera.m_ID << "," << numFrames << "," << targetID << std::endl;
      }
    }
  }
}

int main(int argc, char** argv)
{
  std::string camConfigFile, targetConfigFile, gimbalConfigFile;
  int frames = 10;
  int stepPixels = 30;
  int topMarginPixels = 10;
  int bottomMarginPixels = 10;
  int leftMarginPixels = 10;
  int rightMarginPixels = 10;
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
    } else if (std::string("--stepPixels") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 1;
      }
      stepPixels = std::stoi(argv[i]);
    } else if (std::string("--topMarginPixels") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 1;
      }
      topMarginPixels = std::stoi(argv[i]);
    } else if (std::string("--bottomMarginPixels") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 1;
      }
      bottomMarginPixels = std::stoi(argv[i]);
    } else if (std::string("--leftMarginPixels") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 1;
      }
      leftMarginPixels = std::stoi(argv[i]);
    } else if (std::string("--rightMarginPixels") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 1;
      }
      rightMarginPixels = std::stoi(argv[i]);
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

    // Read the gimbal information.
    GimbalInfo gimbalInfo;
    try {
      gimbalInfo = asdp::render::calibration::GetGimbalInfo(gimbalConfigFile);
    }
    catch (const std::exception& e) {
      std::cerr << "Error: Unable to read gimbal configuration file: " << e.what() << std::endl;
      return 12;
    }

    // Open the output file
    std::string filename = "poses.csv";
    std::cout << std::endl;
    std::cout << "Writing poses to " << filename << std::endl;
    std::ofstream outFile(filename);
    if (!outFile) {
      std::cerr << "Error: Unable to open output file " << filename << std::endl;
      return 20;
    }
    // Write the header line.
    outFile << "FrameIndex,ZRotationDegrees,XRotationDegrees,Camera,NumFrames,TargetID" << std::endl;

    // For each target, generate a series of poses, writing them to the file.
    // When more than one camera can see the same target, request images from all of them
    // using the same frame index.
    int frameIndex = 0;
    for (size_t i = 0; i < targetInfos.size(); i++) {
      const TargetInfo& target = targetInfos[i];
      glm::dvec3 targetPoint = glm::dvec3(target.position[0], target.position[1], target.position[2]);

      // For each camera, run along each edge of the camera, asking for images from all cameras that can
      // see the requested point.  Then repeat with ever-smaller rectangles coming towards the center
      // with less-dense points, so that we have more samples near the edges where distortion gradient
      // magnitude is expected to be the largest.
      for (auto const &cri : cameraRenderInfos) {

        // Compute quantities useful to determine our paths
        int xCenter = cri.m_resolutionPixels[0] / 2;
        int yCenter = cri.m_resolutionPixels[1] / 2;
        int xMin = leftMarginPixels;
        int xMax = cri.m_resolutionPixels[0] - rightMarginPixels - 1;
        int yMin = topMarginPixels;
        int yMax = cri.m_resolutionPixels[1] - bottomMarginPixels - 1;
        double curStep = stepPixels;

        constexpr int numRects = 8;
        for (int i = 0; i < numRects; i++) {

          //===========================================================
          // Run along each edge of the rectangle, asking for images from all cameras that can
          // see the requested point within their margins.
          int numXSteps = 1 + (xMax - xMin) / curStep;
          int numYSteps = 1 + (yMax - yMin) / curStep;

          // Go smoothly around the edges so we minimize motion.
          RunAlongLine(outFile, frameIndex, cameraRenderInfos, frames, cri, targetPoint, target.id,
            gimbalInfo,
            topMarginPixels, bottomMarginPixels,
            leftMarginPixels, rightMarginPixels,
            xMin, yMin, curStep, 0, numXSteps);
          RunAlongLine(outFile, frameIndex, cameraRenderInfos, frames, cri, targetPoint, target.id,
            gimbalInfo,
            topMarginPixels, bottomMarginPixels,
            leftMarginPixels, rightMarginPixels,
            xMax, yMin, 0, curStep, numYSteps);
          RunAlongLine(outFile, frameIndex, cameraRenderInfos, frames, cri, targetPoint, target.id,
            gimbalInfo,
            topMarginPixels, bottomMarginPixels,
            leftMarginPixels, rightMarginPixels,
            xMax, yMax, -curStep, 0, numXSteps);
          RunAlongLine(outFile, frameIndex, cameraRenderInfos, frames, cri, targetPoint, target.id,
            gimbalInfo,
            topMarginPixels, bottomMarginPixels,
            leftMarginPixels, rightMarginPixels,
            xMin, yMax, 0, -curStep, numYSteps);

          //===========================================================
          // Adjust the rectangle to be smaller and move towards the center and adjust the
          // pixel density.
          // We move towards the center less for the further steps, so that we
          // have more samples near the edges where distortion gradient magnitude is expected.
          double rangeScale;
          rangeScale = 1.0 - (i + 1) * 0.1;
          curStep *= 2;
          xMin = xCenter - (xCenter - xMin) * rangeScale;
          xMax = xCenter + (xMax - xCenter) * rangeScale;
          yMin = yCenter - (yCenter - yMin) * rangeScale;
          yMax = yCenter + (yMax - yCenter) * rangeScale;
        }

      } // End of loop over cameras.

    } // End of loop over targets.
    outFile.close();
  } // End of block to ensure that all objects are destructed before we exit.

  return 0;
}
