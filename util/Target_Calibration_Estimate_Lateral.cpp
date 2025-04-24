/*
 * Copyright (C) 2025: Arizona Board of Regents on Behalf of the University of Arizona
 */

/**
 * @file Target_Calibration_Estimate_Lateral.cpp
 * @brief Apache Strap-Down Pilotage configuration calibration program.
 *
* @author ReliaSolve.
* @date March 31st, 2025.
*/

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <filesystem>
#include <vector>
#include <set>
#include <cmath>
#include <CameraRenderInfo.h>
#include <ASDP_ImageSource.h>
#include <Calibration_Helpers.h>
#include <spot_tracker.h>
#include <nlohmann/json.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

using namespace asdp;
using namespace asdp::render;
using namespace asdp::render::calibration;
using json = nlohmann::json;

static std::string VERSION = "2.0.0";

void usage(std::string name)
{
  std::cerr << "Usage: " << name << " [options] camConfig.json targetConfig.json gimbalConfig.json threshold baseDirectory" << std::endl;
  std::cerr << "  camConfig.json                Camera configuration file." << std::endl;
  std::cerr << "  targetConfig.json             Target configuration file." << std::endl;
  std::cerr << "  gimbalConfig.json             Gimbal configuration file." << std::endl;
  std::cerr << "  threshold                     Threshold brightness (int value) for target center." << std::endl;
  std::cerr << "  baseDirectory                 Base directory for the data files (where target_1_poses.csv, target_lateral_1_images, etc. are found)." << std::endl;
  std::cerr << "  Options:" << std::endl;
  std::cerr << "    --help                      Print this information and quit." << std::endl;
  std::cerr << "  Writes targets_lateral_opt.json." << std::endl;
};

int main(int argc, char** argv)
{
  std::string camConfigFile, targetConfigFile, gimbalConfigFile, baseDirectory;
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
      targetBrightnessThreshold = std::stoi(argv[i]);
      break;
    case 4:
      baseDirectory = argv[i];
      break;
    default:
      usage(argv[0]);
      return 2;
    }
  }
  if (realParams != 5) {
    usage(argv[0]);
    return 2;
  }

  // Run inside a block so that the destructors will be called for all objects before we exit.
  {
    std::cout << "Target_Calibration_Estimate_Lateral version " << VERSION << std::endl;

    // Read the configuration files.
    std::vector<asdp::render::CameraRenderInfo> cameraRenderInfos;
    try {
      cameraRenderInfos = GetCameraRenderInfos(camConfigFile);
    }
    catch (...) {
      std::cerr << "Error: Unable to read camera configuration file: " << camConfigFile << std::endl;
      return 14;
    }
    std::cout << "Read camera configuration from " << camConfigFile << std::endl;

    if (!std::filesystem::exists(targetConfigFile)) {
      std::cerr << "Configuration file not found: " << targetConfigFile << std::endl;
      return 14;
    }
    std::ifstream configFile2(targetConfigFile);
    json targetConfig = json::parse(configFile2);
    std::cout << "Read target configuration from " << targetConfigFile << std::endl;

    GimbalInfo gimbalInfo;
    try {
      gimbalInfo = GetGimbalInfo(gimbalConfigFile);
    }
    catch (const std::exception& e) {
      std::cerr << "Error: Unable to read gimbal configuration file: " << gimbalConfigFile
        << ": " << e.what() << std::endl;
      return 14;
    }

    // Parse the target information for each target and read the information from
    // its target_N_poses.csv file.
    struct TargetInfo {
      int id;
      std::array<double, 3> position;
      std::vector<PoseInfo> poses;
    };
    std::vector<TargetInfo> targetInfos;
    for (const auto& target : targetConfig["targets"]) {
      try {
        TargetInfo info;
        info.id = target["id"];
        info.position = target["positionMeters"];

        // Read the poses from the target_N_poses.csv file in the root directory.
        std::string filename = baseDirectory + "/target_" + std::to_string(info.id) + "_poses.csv";
        std::cout << "  Reading poses for target " << info.id << " from " << filename << std::endl;
        info.poses = GetPoseInfos(filename);
        targetInfos.push_back(info);
      }
      catch (const std::exception& e) {
        std::cerr << "Error: Unable to parse target information: " << e.what() << std::endl;
        return 29;
      }
    }

    // For each target, produce an improved lateral position estimate.
    for (TargetInfo& target : targetInfos) {

      // Find the plane through the target location and its normal vector (pointing at the origin).
      glm::dvec3 pointInPlane(target.position[0], target.position[1], target.position[2]);
      glm::dvec3 targetNormal = -glm::normalize(pointInPlane);

      // For each pose, read the set of images and find the average over all images.  Find the target
      // closest to the center of the average image; localize the center of the target.
      // Then find the intersection of the ray from the camera through the image-space
      // target location with the plane through the 3D target.  Replace the 3D target location with
      // the average of these intersection locations.
      std::vector<glm::dvec3> targetLocations;
      for (auto const &pose : target.poses) {

        // Read the set of images associated with this pose and average them into a double-precision
        // floating-point array in a double_image object, which will be usable by the spot-tracker
        // library.  Start by reading the first one to get the size.
        std::cout << "Processing pose " << pose.frameIndex << " for target " << target.id << std::endl;
        std::string imageDirectory = baseDirectory + "/target_lateral_" + std::to_string(target.id) + "_images";
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
        } catch (const std::exception& e) {
          std::cerr << "Error: Unable to read PGM file" << filename << ": " << e.what() << std::endl;
          return 30;
        }
        for (index = 2; index <= pose.numFrames; ++index) {
          filename = imageDirectory + "/" + std::to_string(pose.frameIndex)
            + "_" + std::to_string(pose.cameraID) + "_" + std::to_string(index) + ".pgm";
          try {
            asdp::ImageSource::Image ppm(filename);
            if (ppm.getWidth() != width || ppm.getHeight() != height) {
              std::cerr << "Error: Image " << filename << " has different dimensions from the first image." << std::endl;
              return 30;
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
            return 30;
          }
        }
        double scale = 1/static_cast<double>(pose.numFrames);
        for (int y = 0; y < height; ++y) {
          for (int x = 0; x < width; ++x) {
            double value;
            if (avg->read_pixel(x, y, value)) {
              value *= scale;
              avg->write_pixel(x, y, value);
            }
          }
        }

        // Find the pixel nearest to the image center that is above the threshold brightness.
        int centerX = -1;
        int centerY = -1;
        double minSquaredDistance = 1e30;
        double maxVal = avg->read_pixel_nocheck(0, 0);
        for (int y = 0; y < height; ++y) {
          for (int x = 0; x < width; ++x) {
            maxVal = std::max(maxVal, avg->read_pixel_nocheck(x, y));
            if (avg->read_pixel_nocheck(x, y) >= targetBrightnessThreshold) {
              double squaredDistance = (width/2 - x) * (width/2 - x) + (height/2 - y) * (height/2 - y);
              if (squaredDistance < minSquaredDistance) {
                minSquaredDistance = squaredDistance;
                centerX = x;
                centerY = y;
              }
            }
          }
        }
        if (minSquaredDistance == 1e30) {
          std::cerr << "Error: No target found in pose " << pose.frameIndex << " for target " << target.id << std::endl;
          return 31;
        }
        std::cout << "  Target initialized at (" << centerX << ", " << centerY << ")" << std::endl;

        // Optimize a symmetric spot tracker starting at the specified location with radius 10 pixels.
        symmetric_spot_tracker_interp symmetrictracker(10);
        symmetrictracker.set_pixel_accuracy(0.01);
        double x, y;
        symmetrictracker.optimize_xy(*avg, 0, x, y, centerX, centerY);
        std::cout << "  Target optimized to (" << x << ", " << y << ")" << std::endl;

        // Find the CameraRenderInfo associated with this pose.
        CameraRenderInfo* cri = nullptr;
        for (auto& info : cameraRenderInfos) {
          if (info.m_ID == pose.cameraID) {
            cri = &info;
            break;
          }
        }
        if (cri == nullptr) {
          std::cerr << "Error: Camera ID " << pose.cameraID << " not found in camera configuration." << std::endl;
          return 32;
        }
        if (width != cri->m_resolutionPixels[0] || height != cri->m_resolutionPixels[1]) {
          std::cerr << "Error: Camera " << pose.cameraID << " resolution does not match image resolution." << std::endl;
          return 33;
        }

        // Find the intersection of the ray from the camera starting location through the image-space
        // target location with the plane through the 3D target.  First find the ray start, which is the
        // camera position. Then find the ray direction, which is the ray in camera space rotated by the
        // camera rotation.
        glm::dvec3 rayStartInWorld, rayDirectionInWorld;
        WorldSpaceRayNoDistortion(*cri, x, y, gimbalInfo.pitchFirst,
          pose.zRotationDegrees, pose.xRotationDegrees,
          rayStartInWorld, rayDirectionInWorld, true);

        // Compute the intersection of the ray with the plane.
        // The target normal points towards the origin and the rotated ray direction should point away.
        double dotProduct = glm::dot(targetNormal, rayDirectionInWorld);
        if (dotProduct == 0) {
          std::cerr << "Error: Ray is parallel to the plane for target " << target.id << std::endl;
          return 34;
        }
        double distance = glm::dot(pointInPlane - rayStartInWorld, targetNormal) / dotProduct;
        glm::dvec3 intersection = rayStartInWorld + distance * rayDirectionInWorld;

        std::cout << "  Intersection = " << intersection.x << " " << intersection.y << " " << intersection.z << std::endl;
        targetLocations.push_back(intersection);

      } // End of loop over poses.

      // Replace the 3D target location with the average of the intersection locations.
      if (targetLocations.size() > 0) {
        glm::dvec3 point(0, 0, 0);
        for (auto& location : targetLocations) {
          point += location;
        }
        point /= targetLocations.size();
        target.position[0] = point.x;
        target.position[1] = point.y;
        target.position[2] = point.z;
      } else {
        std::cerr << "Error: No target locations found for target " << target.id << std::endl;
        return 35;
      }

    } // End of loop over targets.

    // Write the optimized target positions to the specified JSON file in the root directory.
    std::string filename = baseDirectory + "/targets_lateral_opt.json";
    size_t t = 0;
    for (auto& target : targetConfig["targets"]) {
      // Find the info with the matching ID.
      auto const& info = targetInfos[t++];
      target["positionMeters"] = info.position;
    }
    std::cout << "Writing optimized target positions to " << filename << std::endl;
    std::ofstream outFile(filename);
    if (!outFile) {
      std::cerr << "Error: Unable to open output file " << filename << std::endl;
      return 50;
    }
    outFile << targetConfig.dump(2) << std::endl;
    outFile.close();

  } // End of block to ensure that all objects are destructed before we exit.

  return 0;
}
