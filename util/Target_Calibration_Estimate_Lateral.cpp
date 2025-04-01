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
#include <spot_tracker.h>
#include <nlohmann/json.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

using namespace asdp;
using namespace asdp::render;
using json = nlohmann::json;

static std::string VERSION = "1.0.0";

void usage(std::string name)
{
  std::cerr << "Usage: " << name << " [options] camConfig.json targetConfig.json threshold baseDirectory" << std::endl;
  std::cerr << "  camConfig.json                Camera configuration file." << std::endl;
  std::cerr << "  targetConfig.json             Target configuration file." << std::endl;
  std::cerr << "  threshold                     Threshold brightness (int value) for target center." << std::endl;
  std::cerr << "  baseDirectory                 Base directory for the data files (where target_1_poses.csv, target_lateral_1_images, etc. are found)." << std::endl;
  std::cerr << "  Options:" << std::endl;
  std::cerr << "    --help                      Print this information and quit." << std::endl;
  std::cerr << "  Writes targets_lateral_opt.json." << std::endl;
};

int main(int argc, char** argv)
{
  std::string camConfigFile, targetConfigFile, baseDirectory;
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
      targetBrightnessThreshold = std::stoi(argv[i]);
      break;
    case 3:
      baseDirectory = argv[i];
      break;
    default:
      usage(argv[0]);
      return 2;
    }
  }
  if (realParams != 4) {
    usage(argv[0]);
    return 2;
  }

  // Run inside a block so that the destructors will be called for all objects before we exit.
  {
    std::cout << "Target_Calibration_Estimate_Lateral version " << VERSION << std::endl;

    // Read the configuration files.
    if (!std::filesystem::exists(camConfigFile)) {
      std::cerr << "Configuration file not found: " << camConfigFile << std::endl;
      return 14;
    }
    std::ifstream configFile1(camConfigFile);
    json camConfig = json::parse(configFile1);
    std::cout << "Read camera configuration from " << camConfigFile << std::endl;

    if (!std::filesystem::exists(targetConfigFile)) {
      std::cerr << "Configuration file not found: " << targetConfigFile << std::endl;
      return 14;
    }
    std::ifstream configFile2(targetConfigFile);
    json targetConfig = json::parse(configFile2);
    std::cout << "Read target configuration from " << targetConfigFile << std::endl;

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
        std::cerr << "Error: Unknown distortion type: " << distortion["type"] << std::endl;
        return 17;
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
        }
        else if (vignette["type"] == nullptr) {
          // No vignette specified, so use the default.
        }
        else {
          std::cerr << "Error: Unknown vignette type: " << vignette["type"] << std::endl;
          return 18;
        }
      }
      catch (...) {
        // No vignette specified, so use the default.
      }

      asdp::render::CameraRenderInfo info(camera["id"],
        camera["positionMeters"], camera["orientationDegrees"],
        camera["resolutionPixels"], camera["fieldOfViewDegrees"],
        dist, vig, std::make_shared<asdp::render::ImageQueue>(), -1.0f);
      cameraRenderInfos.push_back(info);
    }

    // Parse the target information for each target and read the information from
    // its target_N_poses.csv file.
    struct poseInfo {
      int frameIndex;
      double zRotation;
      double xRotation;
      uint16_t cameraID;
      int numFrames;
    };
    struct TargetInfo {
      int id;
      std::array<double, 3> position;
      std::vector<poseInfo> poses;
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
        std::ifstream poseFile(filename);
        if (!poseFile) {
          std::cerr << "Error: Unable to open pose file " << filename << std::endl;
          return 20;
        }
        std::string line;
        std::getline(poseFile, line); // Skip the header line.
        while (std::getline(poseFile, line)) {
          poseInfo pose;
          std::istringstream ss(line);
          char comma;
          ss >> pose.frameIndex >> comma >> pose.zRotation >> comma >> pose.xRotation >> comma
            >> pose.cameraID >> comma >> pose.numFrames;
          info.poses.push_back(pose);
          std::cout << "    " << pose.frameIndex << " " << pose.zRotation << " " << pose.xRotation << " " << pose.cameraID << " " << pose.numFrames << std::endl;
        }
        poseFile.close();

        targetInfos.push_back(info);
      }
      catch (...) {
        std::cerr << "Error: Unable to parse target information." << std::endl;
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
        asdp::ImageSource::Image firstPPM(filename);
        int width = firstPPM.getWidth();
        int height = firstPPM.getHeight();
        double_image avg(0, width - 1, 0, height - 1);
        std::shared_ptr< std::vector<uint16_t> > data = firstPPM.getData();
        for (int y = 0; y < height; ++y) {
          for (int x = 0; x < width; ++x) {
            avg.write_pixel(x, y, (*data)[y * width + x]);
          }
        }
        for (index = 2; index <= pose.numFrames; ++index) {
          filename = imageDirectory + "/" + std::to_string(pose.frameIndex)
            + "_" + std::to_string(pose.cameraID) + "_" + std::to_string(index) + ".pgm";
          asdp::ImageSource::Image ppm(filename);
          if (ppm.getWidth() != width || ppm.getHeight() != height) {
            std::cerr << "Error: Image " << filename << " has different dimensions from the first image." << std::endl;
            return 30;
          }
          data = ppm.getData();
          for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
              double value;
              if (avg.read_pixel(x, y, value)) {
                value += (*data)[y * width + x];
                avg.write_pixel(x, y, value);
              }
            }
          }
        }
        double scale = 1/static_cast<double>(pose.numFrames);
        for (int y = 0; y < height; ++y) {
          for (int x = 0; x < width; ++x) {
            double value;
            if (avg.read_pixel(x, y, value)) {
              value *= scale;
              avg.write_pixel(x, y, value);
            }
          }
        }

        // Find the pixel nearest to the image center that is above the threshold brightness.
        int centerX = -1;
        int centerY = -1;
        double minSquaredDistance = 1e30;
        double maxVal = avg.read_pixel_nocheck(0, 0);
        for (int y = 0; y < height; ++y) {
          for (int x = 0; x < width; ++x) {
            maxVal = std::max(maxVal, avg.read_pixel_nocheck(x, y));
            if (avg.read_pixel_nocheck(x, y) >= targetBrightnessThreshold) {
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
        symmetrictracker.optimize_xy(avg, 0, x, y, centerX, centerY);
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

        // Find the intersection of the ray from the camera starting location through the image-space
        // target location with the plane through the 3D target.  First find the ray start, which is the
        // camera position. Then find the ray direction, which is the ray in camera space rotated by the
        // camera rotation.
        /// @todo Can't lookup by camera ID because the camera ID is not the same as the index in the array.
        glm::dvec3 rayStart;
        rayStart.x = cri->m_positionMeters[0];
        rayStart.y = cri->m_positionMeters[1];
        rayStart.z = cri->m_positionMeters[2];

        double normalizedX = x / (width - 1);
        double normalizedY = y / (height - 1);
        double halfWidth = atan(glm::radians(cri->m_fovDegrees[0] / 2.0));
        double halfHeight = atan(glm::radians(cri->m_fovDegrees[1] / 2.0));
        double xScaled = halfWidth * (2 * normalizedX - 1);
        double yScaled = halfHeight * (1 - 2 * normalizedY);    // Flip the Y axis to make it right handed
        // The pixel centers are half a pixel in from the edge, so we scale to make this happen.
        double xM = xScaled * (width - 1.0) / width;
        double yM = yScaled * (height - 1.0) / height;
        double zM = -1.0;
        // Convert from +X, +Y camera space pointing along -Z to helicopter space with +Z up and +Y forward.
        glm::dvec3 rayDirection = glm::normalize(glm::dvec3(xM, -zM, yM));
        std::cout << "  Camera-space ray direction = " << rayDirection.x << " " << rayDirection.y << " " << rayDirection.z << std::endl;

        // Rotate the ray direction by the camera orientation; X, then Y then Z.
        glm::dvec3 rayDirectionInBall;
        {
          glm::dquat rotationX = glm::angleAxis(glm::radians(cri->m_orientationDegrees[0]), glm::dvec3(1.0, 0.0, 0.0));
          glm::dquat rotationY = glm::angleAxis(glm::radians(cri->m_orientationDegrees[1]), glm::dvec3(0.0, 1.0, 0.0));
          glm::dquat rotationZ = glm::angleAxis(glm::radians(cri->m_orientationDegrees[2]), glm::dvec3(0.0, 0.0, 1.0));
          glm::dquat rotationTotal = rotationX * rotationY * rotationZ;
          rayDirectionInBall = rotationTotal * rayDirection;
        }
        std::cout << "  Ball-space ray direction = " << rayDirectionInBall.x << " " << rayDirectionInBall.y << " " << rayDirectionInBall.z << std::endl;

        // Okay, we now have the ray start and direction in the camera ball's coordinate system.
        // We must rotate both by the pose angle, around Z first and then around X.
        glm::dvec3 rayStartInWorld;
        glm::dvec3 rayDirectionInWorld;
        {
          glm::dquat rotationZ = glm::angleAxis(glm::radians(pose.zRotation), glm::dvec3(0.0, 0.0, 1.0));
          glm::dquat rotationX = glm::angleAxis(glm::radians(pose.xRotation), glm::dvec3(1.0, 0.0, 0.0));
          glm::dquat rotationTotal = rotationZ * rotationX;
          rayDirectionInWorld = rotationTotal * rayDirectionInBall;
          rayStartInWorld = rotationTotal * rayStart;
        }
        std::cout << "  World-space ray direction = " << rayDirectionInWorld.x << " " << rayDirectionInWorld.y << " " << rayDirectionInWorld.z << std::endl;
        std::cout << "  World-space ray start = " << rayStartInWorld.x << " " << rayStartInWorld.y << " " << rayStartInWorld.z << std::endl;

        // Compute the intersection of the ray with the plane.
        // The target normal points towards the origin and the rotated ray direction should point away.
        /// @todo Check this math
        double dotProduct = glm::dot(targetNormal, rayDirectionInWorld);
        if (dotProduct == 0) {
          std::cerr << "Error: Ray is parallel to the plane for target " << target.id << std::endl;
          return 33;
        }
        double distance = glm::dot(pointInPlane - rayStartInWorld, targetNormal) / dotProduct;
        glm::dvec3 intersection = rayStartInWorld + distance * rayDirectionInWorld;

        /// @todo
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
        return 34;
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
