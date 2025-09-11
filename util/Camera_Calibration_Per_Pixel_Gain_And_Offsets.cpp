/*
 * Copyright (C) 2025: Arizona Board of Regents on Behalf of the University of Arizona
 */

/**
 * @file Camera_Calibration_Per_Pixel_Gain_And_Offsets.cpp
 * @brief Apache Strap-Down Pilotage configuration calibration program.
 *
* @author ReliaSolve.
* @date September 8th, 2025.
*/

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <filesystem>
#include <vector>
#include <array>
#include <map>
#include <cmath>
#include <CameraRenderInfo.h>
#include <ASDP_ImageSource.h>
#include <Calibration_Helpers.h>
#include <nlohmann/json.hpp>
#include <base_camera_server.h>

using namespace asdp;
using namespace asdp::render;
using namespace asdp::render::calibration;
using json = nlohmann::json;

static std::string VERSION = "1.0.0";

static void usage(std::string name)
{
  std::cerr << "Usage: " << name << " [options] camConfig.json targetConfig.json gimbalConfig.json NUC.json baseDirectory outDirectory" << std::endl;
  std::cerr << "  camConfig.json                Camera configuration file." << std::endl;
  std::cerr << "  targetConfig.json             Target configuration file." << std::endl;
  std::cerr << "  gimbalConfig.json             Gimbal configuration file." << std::endl;
  std::cerr << "  NUC.json                      Non-uniformity correction calibration file." << std::endl;
  std::cerr << "  baseDirectory                 Base directory for the data files (where target_1_poses.csv, target_lateral_1_images, etc. are found)." << std::endl;
  std::cerr << "  outDirectory                  Output directory for the result files." << std::endl;
  std::cerr << "  Options:" << std::endl;
  std::cerr << "    --help                      Print this information and quit." << std::endl;
  std::cerr << "    --version                   Print the version number and quit." << std::endl;
  std::cerr << "  Writes NUC.json, raw calibration files, and cameras_opt.json into outDirectory." << std::endl;
};

inline void computeGainAndOffset(double lowTemp, double highTemp, double minVisibleTemp, double maxVisibleTemp,
  double lowValue, double highValue, double& gain, double& offset)
{
  //============================================================
  // First convert from values to Celsius temperature, with the conversion function
  // to be applied being degrees = original * gain + offset

  // Compute the gain as the slope of the line from minimum to maximum temperature, dividing the
  // temperature range by the pixel value range.
  double tempRange = highTemp - lowTemp;
  double pixelRange = highValue - lowValue;
  double gainC = (pixelRange != 0) ? (tempRange / pixelRange) : 1.0;

  // Find the pixel value relative to temperature zero based on the gain and the minimum temperature.
  // The offset is the negative of this times the gain (because the offset is in degrees, applied in
  // the space after the gain has been applied to the original value which included the offset).
  double zeroRelativeValue = (0 - lowTemp) / gainC + lowValue;
  double offsetC = -zeroRelativeValue * gainC;

  //============================================================
  // Now convert from Celsius temperature to the desired output range, such that the minimum visible
  // value maps to 0 and the maximum visible value maps to 65535. This will map the visible temperature
  // range to the full range of the 16-bit image.

  double gain16 = 65535.0 / (maxVisibleTemp - minVisibleTemp);
  gain = gainC * gain16;

  // The offset is the value that will make minVisibleTemp move to 0 in combination with the original
  // offset and gain.  We find the offset in Celsius space and then convert it to the 16-bit space.
  offset = (offsetC - minVisibleTemp) * gain16;
}

static bool isClose(double a, double b, double tol = 1e-6)
{
  return std::fabs(a - b) <= tol;
}

static bool testComputeGainAndOffset()
{
  // If the scales are all the same and there are no offsets, the gain should be 65535 / (highTemp)
  // with no offset.
  {
    double gain, offset;
    computeGainAndOffset(0.0, 50.0, 0.0, 50.0, 0.0, 50.0, gain, offset);
    if (!isClose(gain, 65535.0 / (50.0)) || !isClose(offset, 0.0)) {
      std::cerr << "Test 1 failed: gain=" << gain << " offset=" << offset << std::endl;
      return false;
    }
  }

  // If the scales are all the same and there is the same offset for the scales and none for the
  // values, the gain should be 65535 / (highTemp - lowTemp) and the offset should be 0.
  {
    double gain, offset;
    computeGainAndOffset(10.0, 60.0, 10.0, 60.0, 0.0, 50.0, gain, offset);
    if (!isClose(gain, 65535.0 / (60.0 - 10.0)) || !isClose(offset, 0.0 )) {
      std::cerr << "Test 2 failed: gain=" << gain << " offset=" << offset << std::endl;
      return false;
    }
  }

  // If the range is in centigrade and from 0-100 but there is a shift in the values by 10000, the
  // gain should be 65535 / 100 and the offset should be -10000 * gain.
  {
    double gain, offset;
    computeGainAndOffset(0.0, 100.0, 0.0, 100.0, 10000.0, 10100.0, gain, offset);
    if (!isClose(gain, 65535.0 / 100.0) || !isClose(offset, -10000.0 * gain)) {
      std::cerr << "Test 3 failed: gain=" << gain << " offset=" << offset << std::endl;
      return false;
    }
  }

  // If the range of temperatures measured is from 20-70 and the values are from that same range,
  // then the initial gain should be 1.0 and the offset should be -20.0 * gain.  If the final range
  // is 0-100, the gain should be 65535 / (50*2) and the offset should be 0.
  {
    double gain, offset;
    computeGainAndOffset(20.0, 70.0, 0.0, 100.0, 20.0, 70.0, gain, offset);
    if (!isClose(gain, 65535.0 / (50.0*2.0)) || !isClose(offset, 0.0)) {
      std::cerr << "Test 4 failed: gain=" << gain << " offset=" << offset << std::endl;
      return false;
    }
  }

  // If the measured temperatures are from freezing to boiling in Farenheit, and the values are
  // from 0 to 100 Celsius, the initial gain should be 5/9 and the offset should be -32 * gain.
  {
    double gain, offset;
    computeGainAndOffset(0.0, 100.0, 0.0, 100.0, 32.0, 212.0, gain, offset);
    if (!isClose(gain, (65535.0 / 100.0) * (5.0/9.0)) || !isClose(offset, -32.0 * (65535.0 / 100.0) * (5.0/9.0))) {
      std::cerr << "Test 5 failed: gain=" << gain << " offset=" << offset << std::endl;
      return false;
    }
  }

  return true;
}

int main(int argc, char** argv)
{
  // Test our gain and offset computation to make sure it works.
  if (!testComputeGainAndOffset()) {
    std::cerr << "Error: Gain and offset computation test failed." << std::endl;
    return 3;
  }

  std::string camConfigFile, targetConfigFile, gimbalConfigFile, NUCConfigFile, baseDirectory, outDirectory;
  size_t realParams = 0;          ///< The number of non-flag parameters we've seen.

  // Parse the command line arguments, with the first non-flag argument being the
  // name of the IP address to listen on.
  for (int i = 1; i < argc; ++i) {
    if (std::string("--help") == argv[i]) {
      usage(argv[0]);
      return 0;
    } else if (std::string("--version") == argv[i]) {
      std::cout << "Target_Calibration_Estimate_Lateral version " << VERSION << std::endl;
      return 0;
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
      NUCConfigFile = argv[i];
      break;
    case 4:
      baseDirectory = argv[i];
      break;
    case 5:
      outDirectory = argv[i];
      break;
    default:
      usage(argv[0]);
      return 2;
    }
  }
  if (realParams != 6) {
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
      return 4;
    }
    std::cout << "Read camera configuration from " << camConfigFile << std::endl;

    if (!std::filesystem::exists(targetConfigFile)) {
      std::cerr << "Configuration file not found: " << targetConfigFile << std::endl;
      return 5;
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
      return 6;
    }

    if (!std::filesystem::exists(NUCConfigFile)) {
      std::cerr << "Configuration file not found: " << NUCConfigFile << std::endl;
      return 7;
    }
    std::ifstream configFile3(NUCConfigFile);
    json NUCConfig = json::parse(configFile3);
    std::cout << "Read NUC configuration from " << NUCConfigFile << std::endl;

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
        try {
          info.poses = GetPoseInfos(filename);
        }
        catch (const std::exception& e) {
          std::cerr << "Error: Unable to read pose file: " << filename << ": " << e.what() << std::endl;
          return 28;
        }
        targetInfos.push_back(info);
      }
      catch (const std::exception& e) {
        std::cerr << "Error: Unable to parse target information: " << e.what() << std::endl;
        return 29;
      }
    }

    // For each target and pose/camera, produce an average floating-point image across all input images.
    std::array< std::map<uint16_t, std::shared_ptr<float_image> >, 2> lateralAvgImages; // [targetIndex][cameraID]
    int width = 0, height = 0;
    for (TargetInfo& target : targetInfos) {

      // For each pose, read the set of images and find the average over all images.
      std::map<uint16_t, std::shared_ptr<float_image> > avgImagePerCamera;
      for (auto const& pose : target.poses) {

        // Read the set of images associated with this pose and average them into a double-precision
        // floating-point array in a float_image object, which will be usable by the spot-tracker
        // library.  Start by reading the first one to get the size.
        std::cout << "Processing pose " << pose.frameIndex << " for target " << target.id << std::endl;
        std::string imageDirectory = baseDirectory + "/target_lateral_" + std::to_string(target.id) + "_images";
        int index = 1;
        std::string filename = imageDirectory + "/" + std::to_string(pose.frameIndex)
          + "_" + std::to_string(pose.cameraID) + "_" + std::to_string(index) + ".pgm";
        std::shared_ptr<float_image> avg;
        try {
          asdp::ImageSource::Image firstPGM(filename);
          width = firstPGM.getWidth();
          height = firstPGM.getHeight();
          avg = std::make_shared<float_image>(0, width - 1, 0, height - 1);
          std::shared_ptr< std::vector<uint16_t> > data = firstPGM.getData();
          for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
              avg->write_pixel(x, y, (*data)[y * width + x]);
            }
          }
        }
        catch (const std::exception& e) {
          std::cerr << "Error: Unable to read PGM file" << filename << ": " << e.what() << std::endl;
          return 30;
        }
        for (index = 2; index <= pose.numFrames; ++index) {
          filename = imageDirectory + "/" + std::to_string(pose.frameIndex)
            + "_" + std::to_string(pose.cameraID) + "_" + std::to_string(index) + ".pgm";
          try {
            asdp::ImageSource::Image pgm(filename);
            if (pgm.getWidth() != width || pgm.getHeight() != height) {
              std::cerr << "Error: Image " << filename << " has different dimensions from the first image." << std::endl;
              return 30;
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
            return 30;
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

        // Save the average image for this camera.
        avgImagePerCamera[pose.cameraID] = avg;
      } // End of loop over poses.

      // Save the average images for this target.
      lateralAvgImages[target.id - 1] = avgImagePerCamera;

    } // End of loop over targets.

    // Make a vector of all camera IDs.
    std::vector<uint16_t> cameraIDs;
    for (const auto& cri : cameraRenderInfos) {
      cameraIDs.push_back(cri.m_ID);
    }
    std::sort(cameraIDs.begin(), cameraIDs.end());

    // Make the per-pixel gain and offset images for each camera using the average images
    // corresponding to each target.  Write them to raw files in the output directory.
    for (auto id : cameraIDs) {

      // Create the gain and offset images for this camera.
      std::shared_ptr<float_image> gainImage, offsetImage;
      gainImage = std::make_shared<float_image>(0, width - 1, 0, height - 1);
      offsetImage = std::make_shared<float_image>(0, width - 1, 0, height - 1);

      // Get the lower-temperature and higher-temperature target images.
      std::shared_ptr<float_image> lowImage = lateralAvgImages[0][id];
      std::shared_ptr<float_image> highImage = lateralAvgImages[1][id];

      // Get the other parameters we need.
      float lowTemp = NUCConfig["coldBBRTemperature"];
      float highTemp = NUCConfig["hotBBRTemperature"];
      float minVisibleTemp = NUCConfig["minVisibleTemperature"];
      float maxVisibleTemp = NUCConfig["maxVisibleTemperature"];

      // Compute the gain and offset images using the averaged images for each target along with the
      // temperatures specified in the NUC configuration file.
#pragma omp parallel for
      for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
          double lowValue = lowImage->read_pixel_nocheck(x, y);
          double highValue = highImage->read_pixel_nocheck(x, y);

          double gain, offset;
          computeGainAndOffset(lowTemp, highTemp, minVisibleTemp, maxVisibleTemp,
            lowValue, highValue, gain, offset);

          gainImage->write_pixel(x, y, gain);
          offsetImage->write_pixel(x, y, offset);
        }
      }

      // Find the base filename for the gain and offset raw images.
      std::string gainFilename = "/camera_" + std::to_string(id) + "_gain.raw";
      std::string offsetFilename = "/camera_" + std::to_string(id) + "_offset.raw";

      // Write the gain and offset raw images to the output directory in the specified file names.
      // These are written as 32-bit floating-point raw files of width*height each.
      std::string gainFullFilename = outDirectory + gainFilename;
      std::cout << "  Writing gain image for camera " << id << " to " << gainFullFilename << std::endl;
      std::ofstream gainFile(gainFullFilename, std::ios::binary);
      if (!gainFile) {
        std::cerr << "Error: Unable to open output file " << gainFullFilename << std::endl;
        return 40;
      }
      for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
          double value = 0.0f;
          gainImage&& gainImage->read_pixel(x, y, value);
          float floatValue = static_cast<float>(value);
          gainFile.write(reinterpret_cast<const char*>(&value), sizeof(float));
        }
      }

      // Adjust the NUC data to add entries for the gain and offset raw images and the image resolutions.
      for (auto& camera : NUCConfig["cameras"]) {
        if (camera["id"] == id) {
          camera["gainFile"] = gainFilename;
          camera["offsetFile"] = offsetFilename;
          camera["imageWidth"] = width;
          camera["imageHeight"] = height;
          break;
        }
      }
    }

    // Write the modified NUC data to the specified JSON file in the output directory.
    std::string filename = outDirectory + "/NUC.json";
    size_t t = 0;
    std::cout << "Writing NUC information to " << filename << std::endl;
    std::ofstream outFile(filename);
    if (!outFile) {
      std::cerr << "Error: Unable to open output file " << filename << std::endl;
      return 50;
    }
    outFile << NUCConfig.dump(2) << std::endl;
    outFile.close();

    // Parse the JSON configuration file for the camera configuration directly, then replace
    // the gain and offset for each camera with 1 and 0.
    json cameraConfig;
    try {
      std::ifstream configFile(camConfigFile);
      cameraConfig = json::parse(configFile);
    } catch (const std::exception& e) {
      std::cerr << "Error: Unable to read camera configuration file: " << camConfigFile
        << ": " << e.what() << std::endl;
      return 200;
    }
    for (auto& camera : cameraConfig["cameras"]) {
      for (auto& cri : cameraRenderInfos) {
        camera["color"]["gain"] = 1.0;
        camera["color"]["offset"] = 0.0;
      }
    }

    // Write the optimized camera configuration to the specified JSON file in the output directory.
    filename = outDirectory + "/cameras_opt.json";
    std::cout << "Writing optimized camera configuration to " << filename << std::endl;
    std::ofstream outFile2(filename);
    if (!outFile2) {
      std::cerr << "Error: Unable to open output file " << filename << std::endl;
      return 51;
    }
    outFile2 << cameraConfig.dump(2) << std::endl;
    outFile2.close();

  } // End of block to ensure that all objects are destructed before we exit.

  return 0;
}
