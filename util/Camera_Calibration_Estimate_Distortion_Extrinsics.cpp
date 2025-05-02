/*
 * Copyright (C) 2025: Arizona Board of Regents on Behalf of the University of Arizona
 */

/**
 * @file Camera_Calibration_Estimate_Distortion_Extrinsics.cpp
 * @brief Apache Strap-Down Pilotage configuration calibration program.
 *
* @author ReliaSolve.
* @date April 24th, 2025.
*/

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <CameraRenderInfo.h>
#include <ASDP_ImageSource.h>
#include <Calibration_Helpers.h>
#include <spot_tracker.h>
#include <nlohmann/json.hpp>

using namespace asdp;
using namespace asdp::render;
using namespace asdp::render::calibration;
using json = nlohmann::json;

static std::string VERSION = "0.9.0";

void usage(std::string name)
{
  std::cerr << "Usage: " << name << " [options] camConfig.json targetConfig.json gimbalConfig.json poses.csv imageDirectory threshold outputConfig.json" << std::endl;
  std::cerr << "  camConfig.json                Camera configuration file." << std::endl;
  std::cerr << "  targetConfig.json             Target configuration file." << std::endl;
  std::cerr << "  gimbalConfig.json             Gimbal configuration file." << std::endl;
  std::cerr << "  poses.csv                     CSV file with the poses of the camera." << std::endl;
  std::cerr << "  imageDirectory                Directory with the images." << std::endl;
  std::cerr << "  threshold                     Threshold brightness (int value) for target center." << std::endl;
  std::cerr << "  outputConfig.json             Output configuration file." << std::endl;
  std::cerr << "  Options:" << std::endl;
  std::cerr << "    --help                      Print this information and quit." << std::endl;
  std::cerr << "  Writes camConfig_opt.json." << std::endl;
};

int main(int argc, char** argv)
{
  std::string camConfigFile, targetConfigFile, gimbalConfigFile, posesFile, imageDirectory, outputFile;
  int targetBrightnessThreshold = 35767;
  size_t realParams = 0;          ///< The number of non-flag parameters we've seen.

  // Parse the command line arguments, with the first non-flag argument being the
  // name of the IP address to listen on.
  for (int i = 1; i < argc; ++i) {
    if (std::string("--help") == argv[i]) {
      usage(argv[0]);
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
    case 3:
      posesFile = argv[i];
      break;
    case 4:
      imageDirectory = argv[i];
      break;
    case 5:
      targetBrightnessThreshold = std::stoi(argv[i]);
      break;
    case 6:
      outputFile = argv[i];
      break;
    default:
      usage(argv[0]);
      return 2;
    }
  }
  if (realParams != 7) {
    usage(argv[0]);
    return 2;
  }

  // Run inside a block so that the destructors will be called for all objects before we exit.
  {
    std::cout << "Camera_Calibration_Estimate_Distortion_Extrinsics version " << VERSION << std::endl;

    // Read the configuration files.
    std::vector<asdp::render::CameraRenderInfo> cameraRenderInfos;
    try {
      cameraRenderInfos = GetCameraRenderInfos(camConfigFile);
    }
    catch (...) {
      std::cerr << "Error: Unable to read camera configuration file: " << camConfigFile << std::endl;
      return 10;
    }
    std::cout << "Read camera configuration from " << camConfigFile << std::endl;

    std::vector<TargetInfo> targetInfos;
    try {
      targetInfos = GetTargetInfos(targetConfigFile);
    }
    catch (...) {
      std::cerr << "Error: Unable to read target configuration file: " << targetConfigFile << std::endl;
      return 11;
    }
    std::cout << "Read target configuration from " << targetConfigFile << std::endl;

    GimbalInfo gimbalInfo;
    try {
      gimbalInfo = GetGimbalInfo(gimbalConfigFile);
    }
    catch (const std::exception& e) {
      std::cerr << "Error: Unable to read gimbal configuration file: " << gimbalConfigFile
        << ": " << e.what() << std::endl;
      return 12;
    }
    std::cout << "Read gimbal configuration from " << gimbalConfigFile << std::endl;

    // Read the pose information from the specified CSV file.
    std::vector<PoseInfo> poseInfos;
    try {
      poseInfos = GetPoseInfos(posesFile);
    }
    catch (const std::exception& e) {
      std::cerr << "Error: Unable to read pose information from file: " << posesFile
        << ": " << e.what() << std::endl;
      return 13;
    }
    std::cout << "Read pose information from " << posesFile << std::endl;

    // We always do a bag-of-mappings distortion model.
    // Bag of mappings per camera, looked up by camera ID.
    std::map<uint16_t, DistortionBagOfMappings::Bag> bags;
    for (auto& cri : cameraRenderInfos) {
      // Create an empty bag of mappings for each camera.
      bags[cri.m_ID] = DistortionBagOfMappings::Bag();
    }

    if (targetInfos.size() == 1) {
      // We have a single target, so we do a direct bag-of-mappings distortion model to make
      // all points line up at the single target's location (only works for a single depth).
      std::cout << "Using single-depth distortion model" << std::endl;

      // Fill in a mapping entry for the appropriate camera and pose.
      int count = 0;
#pragma omp parallel for shared(count)
      for (int p = 0; p < poseInfos.size(); p++) {
        auto const& pose = poseInfos[p];

        // Read the set of images associated with this pose and average them into a double-precision
        // floating-point array in a double_image object, which will be usable by the spot-tracker
        // library.  Start by reading the first one to get the size.
        int index = 1;
        std::string filename = imageDirectory + "/" + std::to_string(pose.frameIndex)
          + "_" + std::to_string(pose.cameraID) + "_" + std::to_string(index) + ".pgm";
        int width, height;
        std::shared_ptr<double_image> avg;
        try {
          asdp::ImageSource::Image firstPPM(filename);
          width = firstPPM.getWidth();
          height = firstPPM.getHeight();
          avg = std::make_shared<double_image>(0, width - 1, 0, height - 1);
          std::shared_ptr< std::vector<uint16_t> > data = firstPPM.getData();
          for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
              avg->write_pixel(x, y, (*data)[y * width + x]);
            }
          }
        }
        catch (const std::exception& e) {
          std::cerr << "Error: Unable to read PGM file" << filename << ": " << e.what() << std::endl;
          exit(20);
        }
        for (index = 2; index <= pose.numFrames; ++index) {
          filename = imageDirectory + "/" + std::to_string(pose.frameIndex)
            + "_" + std::to_string(pose.cameraID) + "_" + std::to_string(index) + ".pgm";
          try {
            asdp::ImageSource::Image ppm(filename);
            if (ppm.getWidth() != width || ppm.getHeight() != height) {
              std::cerr << "Error: Image " << filename << " has different dimensions from the first image." << std::endl;
              exit(21);
            }
            std::shared_ptr< std::vector<uint16_t> > data = ppm.getData();
            for (int y = 0; y < height; ++y) {
              for (int x = 0; x < width; ++x) {
                double value;
                if (avg->read_pixel(x, y, value)) {
                  value += (*data)[y * width + x];
                  avg->write_pixel(x, y, value);
                }
              }
            }
          }
          catch (const std::exception& e) {
            std::cerr << "Error: Unable to read PGM file" << filename << ": " << e.what() << std::endl;
            exit(22);
          }
        }
        double scale = 1 / static_cast<double>(pose.numFrames);
        for (int y = 0; y < height; ++y) {
          for (int x = 0; x < width; ++x) {
            double value;
            if (avg->read_pixel(x, y, value)) {
              value *= scale;
              avg->write_pixel(x, y, value);
            }
          }
        }

        // Find the camera whose ID matches the one in the pose.
        CameraRenderInfo* cri = nullptr;
        for (auto& info : cameraRenderInfos) {
          if (info.m_ID == pose.cameraID) {
            cri = &info;
            break;
          }
        }
        if (cri == nullptr) {
          std::cerr << "Error: Camera ID " << pose.cameraID << " not found in camera configuration." << std::endl;
          exit(23);
        }

        // Find the target location by looking up in the targetInfos vector for the one with the
        // matching ID.
        TargetInfo* target = nullptr;
        for (auto& info : targetInfos) {
          if (info.id == pose.targetID) {
            target = &info;
            break;
          }
        }
        if (target == nullptr) {
          std::cerr << "Error: Target ID " << pose.targetID << " not found in target configuration." << std::endl;
          exit(24);
        }

        // Find the expected location of the target in the image in pixels based on ideal camera parameters.
        std::array<double, 2> expectedLocation;
        if (!TargetProjectedLocationNoDistortion(*cri, gimbalInfo.pitchFirst,
            pose.zRotationDegrees, pose.xRotationDegrees, target->position,
            expectedLocation[0], expectedLocation[1])) {
          // The target does not hit the image plane, so skip this pose.
          std::cout << "Warning: Target " << target->id << " does not hit the image plane for camera "
            << pose.cameraID << " at pose " << pose.frameIndex << std::endl;
          continue;
        }

        // Find the pixel above threshold closest to the expected location of the point in the average image.
        int centerX = -1;
        int centerY = -1;
        double minSquaredDistance = 1e30;
        double maxVal = avg->read_pixel_nocheck(0, 0);
        for (int y = 0; y < height; ++y) {
          for (int x = 0; x < width; ++x) {
            maxVal = std::max(maxVal, avg->read_pixel_nocheck(x, y));
            if (avg->read_pixel_nocheck(x, y) >= targetBrightnessThreshold) {
              double squaredDistance = (expectedLocation[0] - x) * (expectedLocation[0] - x)
                + (expectedLocation[1] - y) * (expectedLocation[1] - y);
              if (squaredDistance < minSquaredDistance) {
                minSquaredDistance = squaredDistance;
                centerX = x;
                centerY = y;
              }
            }
          }
        }
        if (minSquaredDistance == 1e30) {
          std::cerr << "Warning: No target found in pose " << pose.frameIndex << " for camera " << pose.cameraID << std::endl;
          continue;
        }
        
        // Optimize a symmetric spot tracker starting at the specified location with radius 10 pixels.
        symmetric_spot_tracker_interp symmetrictracker(10);
        symmetrictracker.set_pixel_accuracy(0.01);
        double x, y;
        symmetrictracker.optimize_xy(*avg, 0, x, y, centerX, centerY);

        // Add a mapping entry from the expected location to the actual location in the image
        // and tell what we did.  Make a critical section to avoid thread contention during this time.
#pragma omp critical
        {
          auto& bag = bags[pose.cameraID];
          // Convert from pixel coordinates to 2D coordinates in the Z=-1 plane based on the ideal-
          // camera parameters.
          std::array<double, 2> idealCameraLocation;

          DistortionBagOfMappings::Point2D expected = PlaneIntersectionForPixel(*cri, expectedLocation);
          DistortionBagOfMappings::Point2D actual = PlaneIntersectionForPixel(*cri, { x, y });

          // Map from the actual (as rendered) position to the ideal (expected) position.
          DistortionBagOfMappings::Mapping mapping = { actual, expected };
          bag.push_back(mapping);

          count++;
          std::cout << count << " / " << poseInfos.size() << " processed; pose " << pose.frameIndex
            << " for camera " << pose.cameraID << std::endl;
          std::cout << "  Target expected at (" << expectedLocation[0] << ", " << expectedLocation[1] << ")" << std::endl;
          std::cout << "  Target initialized at (" << centerX << ", " << centerY << ")" << std::endl;
          std::cout << "  Target optimized to (" << x << ", " << y << ")" << std::endl;
        }
      }
    }
    else {
      // We have multiple targets, so do full estimation of position, orientation, and distortion
      // based on the ideal-camera FOV and the target locations.

      /// @todo
      std::cerr << "Error: Multiple targets not yet implemented." << std::endl;
      return 100;

      /// @todo Remember to map FROM actual location TO ideal (expected) location.
    }

    // Parse the JSON configuration file for the camera configuration directly, then replace
    // the extrinsic parameters and distortion correction for each camera with the optimized values
    // from the entry that has the same ID as the camera.
    json cameraConfig;
    try {
      std::ifstream configFile(camConfigFile);
      cameraConfig = json::parse(configFile);
    }
    catch (const std::exception& e) {
      std::cerr << "Error: Unable to read camera configuration file: " << camConfigFile
        << ": " << e.what() << std::endl;
      return 200;
    }
    for (auto& camera : cameraConfig["cameras"]) {
      uint16_t id = camera["id"];
      for (auto& cri : cameraRenderInfos) {
        if (cri.m_ID == id) {
          camera["positionMeters"] = cri.m_positionMeters;
          camera["orientationDegrees"] = cri.m_orientationDegrees;

          // Build the JSON object for the distortion map, which has a "type" field with
          // "bagOfMappings", and a "map" field with the bag of mappings. Fill this into
          // the distortion field in the JSON structure.
          json jsonObject;
          jsonObject["type"] = "bagOfMappings";
          jsonObject["map"] = json::array();

          const auto& bag = bags[cri.m_ID];
          json mappingJson = json::array();
          for (const auto& mapping : bag) {
            // Each mapping is a pair of 2D points, so we need to convert them to JSON.
            // The first point is the ideal camera position, and the second point is the distorted camera position.
            json mappingJsonEntry = json::array();
            mappingJsonEntry.push_back({ mapping[0][0], mapping[0][1] });
            mappingJsonEntry.push_back({ mapping[1][0], mapping[1][1] });
            mappingJson.push_back(mappingJsonEntry);
          }
          jsonObject["map"] = mappingJson;
          camera["distortion"] = jsonObject;
          break;
        }
      }
    }

    // Write the optimized camera configuration to the specified JSON file in the root directory.
    std::cout << "Writing optimized camera configuration to " << outputFile << std::endl;
    std::ofstream outFile(outputFile);
    if (!outFile) {
      std::cerr << "Error: Unable to open output file " << outputFile << std::endl;
      return 50;
    }
    outFile << cameraConfig.dump(2) << std::endl;
    outFile.close();

  } // End of block to ensure that all objects are destructed before we exit.

  return 0;
}
