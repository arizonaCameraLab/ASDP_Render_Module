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
#include <sstream>
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

static std::string VERSION = "1.1.0";

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
  std::cerr << "    --writeMaps <filename.csv>  Write the expected to as-seen mappings to the specified CSV file." << std::endl;
  std::cerr << "    --readMaps <filename.csv>   Read the expected to as-seen mappings from the specified CSV file, don't compute." << std::endl;
  std::cerr << "    --invert                    Invert each image (useful for dark targets)." << std::endl;
};

int main(int argc, char** argv)
{
  std::string camConfigFile, targetConfigFile, gimbalConfigFile, posesFile, imageDirectory, outputFile;
  std::string writeMapsFile, readMapsFile;
  int targetBrightnessThreshold = 35767;
  bool invert = false; ///< Whether to invert the images (useful for dark targets).
  size_t realParams = 0;          ///< The number of non-flag parameters we've seen.

  // Parse the command line arguments, with the first non-flag argument being the
  // name of the IP address to listen on.
  for (int i = 1; i < argc; ++i) {
    if (std::string("--help") == argv[i]) {
      usage(argv[0]);
    } else if (std::string("--writeMaps") == argv[i]) {
      if (i + 1 >= argc) {
        std::cerr << "Error: --writeMaps option requires a filename." << std::endl;
        return 1;
      }
      writeMapsFile = argv[++i];
    } else if (std::string("--readMaps") == argv[i]) {
      if (i + 1 >= argc) {
        std::cerr << "Error: --readMaps option requires a filename." << std::endl;
        return 1;
      }
      readMapsFile = argv[++i];
    } else if (std::string("--invert") == argv[i]) {
      invert = true;
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

  if (!writeMapsFile.empty() && !readMapsFile.empty()) {
    std::cerr << "Error: Cannot specify both --writeMaps and --readMaps options." << std::endl;
    return 3;
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

    // Add the offset to the camera positions.
    std::cout << "Adding cameraOffset: " << gimbalInfo.cameraOffset[0] << "," << gimbalInfo.cameraOffset[1] << "," << gimbalInfo.cameraOffset[2] << std::endl;
    for (auto& camera : cameraRenderInfos) {
      camera.m_positionMeters[0] += gimbalInfo.cameraOffset[0];
      camera.m_positionMeters[1] += gimbalInfo.cameraOffset[1];
      camera.m_positionMeters[2] += gimbalInfo.cameraOffset[2];
    }

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

    // Map per targetID of a map per cameraID of a bag of mappings.
    std::map<uint16_t, std::map<uint16_t, DistortionBagOfMappings::Bag> > perTargetBags;

    // We always do a bag-of-mappings distortion model.
    // Bag of mappings per camera, looked up by camera ID.
    // This is filled in by the optimization routines using the information from the
    // per-target mappings.
    std::map<uint16_t, DistortionBagOfMappings::Bag> bags;
    for (auto& cri : cameraRenderInfos) {
      // Create an empty bag of mappings for each camera.
      bags[cri.m_ID] = DistortionBagOfMappings::Bag();
    }

    // If we are writing the mappings to a file, open that file and write the header line.
    std::ofstream outMapFile;
    if (!writeMapsFile.empty()) {
      outMapFile = std::ofstream(writeMapsFile);
      if (!outMapFile) {
        std::cerr << "Error: Unable to open output file " << writeMapsFile << std::endl;
        return 40;
      }
      outMapFile << "targetID,frameIndex,cameraID,expectedX,expectedY,actualX,actualY" << std::endl;
    }

    // If we are reading the mappings from a file, then read them in and skip the calculations.
    if (!readMapsFile.empty()) {
      std::ifstream inFile(readMapsFile);
      if (!inFile) {
        std::cerr << "Error: Unable to open input file " << readMapsFile << std::endl;
        return 50;
      }
      std::string line;
      // Skip the header line.
      std::getline(inFile, line);
      // Read the mappings from the file.
      for (int p = 0; p < poseInfos.size(); p++) {
        auto const& pose = poseInfos[p];
        if (std::getline(inFile, line)) {
          // Parse the line as a mapping from expected to actual location.
          std::istringstream iss(line);
          int targetID, frameIndex, cameraID;
          double x1, y1, x2, y2;
          char comma;
          if (!(iss >> targetID >> comma >> frameIndex >> comma >> cameraID >> comma
            >> x1 >> comma >> y1 >> comma >> x2 >> comma >> y2)) {
            std::cerr << "Error: Unable to parse mapping line: " << line << std::endl;
            return 51;
          }
          // Add the mapping to the appropriate bag of mappings for the appropriate target.
          // We map from the actual (as rendered) position to the ideal (expected) position.
          int whichCamera = -1;
          for (size_t i = 0; i < cameraRenderInfos.size(); ++i) {
            if (cameraRenderInfos[i].m_ID == cameraID) {
              whichCamera = i;
              break;
            }
          }
          if (whichCamera == -1) {
            std::cerr << "Error: Camera ID " << cameraID << " not found in camera configuration." << std::endl;
            return 52;
          }
          DistortionBagOfMappings::Point2D expected = PlaneIntersectionForPixelNoDistortion(cameraRenderInfos[whichCamera], {x1, y1});
          DistortionBagOfMappings::Point2D actual = PlaneIntersectionForPixelNoDistortion(cameraRenderInfos[whichCamera], { x2, y2 });
          DistortionBagOfMappings::Mapping mapping = { actual, expected };
          perTargetBags[pose.targetID][pose.cameraID].push_back(mapping); // Assuming all mappings are for camera ID 0 for now.
        } else {
          std::cerr << "Error: Unable to read mapping line for pose " << pose.frameIndex << " camera " << pose.cameraID << std::endl;
          return 53;
        }
      }
      inFile.close();
      std::cout << "Read mappings from " << readMapsFile << std::endl;

    } else {
      // Fill in a mapping entry for the appropriate camera and pose.
      std::atomic_int count = 0;
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
          asdp::ImageSource::Image firstPGM(filename);
          width = firstPGM.getWidth();
          height = firstPGM.getHeight();
          if (invert) {
            for (size_t i = 0; i < width * height; i++) {
              uint16_t pixelValue = firstPGM.getData()->at(i);
              firstPGM.getData()->at(i) = 65535 - pixelValue; // Invert the pixel value.
            }
          }
          avg = std::make_shared<double_image>(0, width - 1, 0, height - 1);
          std::shared_ptr< std::vector<uint16_t> > data = firstPGM.getData();
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
            asdp::ImageSource::Image pgm(filename);
            if (pgm.getWidth() != width || pgm.getHeight() != height) {
              std::cerr << "Error: Image " << filename << " has different dimensions from the first image." << std::endl;
              exit(21);
            }
            if (invert) {
              for (size_t i = 0; i < width * height; i++) {
                uint16_t pixelValue = pgm.getData()->at(i);
                pgm.getData()->at(i) = 65535 - pixelValue; // Invert the pixel value.
              }
            }
            std::shared_ptr< std::vector<uint16_t> > data = pgm.getData();
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
              avg->write_pixel(x, y, value * scale);
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
        
        // Optimize a bright-centered cone tracker starting at the specified location to robustly lock onto the bright spot and
        // then optimize a symmetric spot tracker with radius 10 pixels starting there for more precision.
        cone_spot_tracker_interp coneTracker(10);
        double x, y;
        coneTracker.optimize_xy(*avg, 0, x, y, centerX, centerY);

        symmetric_spot_tracker_interp symmetrictracker(10);
        symmetrictracker.set_pixel_accuracy(0.01);
        symmetrictracker.optimize_xy(*avg, 0, x, y, x, y);

        // Add a mapping entry from the expected location to the actual location in the image
        // and tell what we did.  Make a critical section to avoid thread contention during this time.
#pragma omp critical
        {
          auto& bag = perTargetBags[pose.targetID][pose.cameraID];
          // Convert from pixel coordinates to 2D coordinates in the Z=-1 plane based on the ideal-
          // camera parameters.
          std::array<double, 2> idealCameraLocation;

          DistortionBagOfMappings::Point2D expected = PlaneIntersectionForPixelNoDistortion(*cri, expectedLocation);
          DistortionBagOfMappings::Point2D actual = PlaneIntersectionForPixelNoDistortion(*cri, { x, y });

          // Map from the actual (as rendered) position to the ideal (expected) position.
          DistortionBagOfMappings::Mapping mapping = { actual, expected };
          bag.push_back(mapping);

          count++;
          std::cout << count << " / " << poseInfos.size() << " processed; pose " << pose.frameIndex
            << " for camera " << pose.cameraID << "\n";
          std::cout << "  Target expected at (" << expectedLocation[0] << ", " << expectedLocation[1] << ")" << "\n";
          std::cout << "  Target initialized at (" << centerX << ", " << centerY << ")" << "\n";
          std::cout << "  Target optimized to (" << x << ", " << y << ")" << std::endl;

          // Write the mapping to the output file if requested.
          if (!writeMapsFile.empty()) {
            outMapFile << std::fixed << std::setprecision(8) << pose.targetID << "," << pose.frameIndex << "," << pose.cameraID << ","
              << expectedLocation[0] << "," << expectedLocation[1] << ","
              << x << "," << y << std::endl;
          }
        }
      }
    }
    // Close the output file if we opened it.
    if (outMapFile.is_open()) {
      outMapFile.close();
      std::cout << "Wrote mappings to " << writeMapsFile << std::endl;
    }

    // Perform the optimization to determine the camera models, including distortion.
    if (targetInfos.size() == 1) {
      // We have a single target, so we do a direct bag-of-mappings distortion model to make
      // all points line up at the single target's location (only works for a single depth).
      std::cout << "Using single-depth distortion model" << std::endl;

      // Just grab the bag of mappings for the first (and only) target ID.
      bags = perTargetBags[targetInfos[0].id];

    } else {
      // We have multiple targets, so do full estimation of position, orientation, and distortion
      // for each entry in cameraRenderInfos based on the ideal-camera FOV and the target locations.
      // Modify the CRI information in place and set the bags distortion for each camera.

      std::cout << "Using multiple-depth distortion model" << std::endl;

      // Construct a vector of bags of mappings per camera ID with an entry for each target in the
      // targetInfos vector.  Use that to optimize the camera parameters for that camera.
      for (auto& cri : cameraRenderInfos) {

        // Find the bags for this camera ID.
        std::vector<DistortionBagOfMappings::Bag> myBags;
        for (auto& target : targetInfos) {
          auto& bag = perTargetBags[target.id][cri.m_ID];
          myBags.push_back(bag);
        }

        // Use the bags to optimize the camera parameters for this camera and to construct its
        // bag-of-mappings distortion (maps from actual pixel location to location in the ideal
        // camera, which may be outside of the ideal camera FOV).
        /// @todo Remember to map FROM actual location TO ideal (expected) location.

        /// @todo

        std::cerr << "Error: Multiple targets not yet implemented." << std::endl;
        return 100;
      }
    }

    // Bring the positions back to the original camera position by subtracting the offsets we added above.
    for (auto& cri : cameraRenderInfos) {
      cri.m_positionMeters[0] -= gimbalInfo.cameraOffset[0];
      cri.m_positionMeters[1] -= gimbalInfo.cameraOffset[1];
      cri.m_positionMeters[2] -= gimbalInfo.cameraOffset[2];
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
          // "bagOfMappings", and a "parameters" field with a "map" field with the bag of mappings.
          // Fill this into the distortion field in the JSON structure.
          json jsonObject;
          jsonObject["type"] = "bagOfMappings";
          jsonObject["parameters"] = json::object();
          jsonObject["parameters"]["map"] = json::array();

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
          jsonObject["parameters"]["map"] = mappingJson;
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
