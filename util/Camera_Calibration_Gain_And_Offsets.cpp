/*
 * Copyright (C) 2025: Arizona Board of Regents on Behalf of the University of Arizona
 */

/**
 * @file Camera_Calibration_Gain_And_Offsets.cpp
 * @brief Apache Strap-Down Pilotage configuration calibration program.
 *
* @author ReliaSolve.
* @date August 26th, 2025.
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

static std::string VERSION = "1.2.0";

void usage(std::string name)
{
  std::cerr << "Usage: " << name << " [options] camConfig.json poses.csv imageDirectory lowTemp highTemp minTemp outputConfig.json" << std::endl;
  std::cerr << "  camConfig.json                Camera configuration file." << std::endl;
  std::cerr << "  poses.csv                     CSV file with the poses of the camera." << std::endl;
  std::cerr << "  imageDirectory                Directory with the images." << std::endl;
  std::cerr << "  lowTemp                       Temperature of the cool blackbody." << std::endl;
  std::cerr << "  highTemp                      Temperature of the hot blackbody." << std::endl;
  std::cerr << "  minTemp                       Minimum temperature that will be visible to the ball." << std::endl;
  std::cerr << "  outputConfig.json             Output configuration file." << std::endl;
  std::cerr << "  Options:" << std::endl;
  std::cerr << "    --help                      Print this information and quit." << std::endl;
};

int main(int argc, char** argv)
{
  std::string camConfigFile, posesFile, imageDirectory, outputFile;
  double lowTemp, highTemp, minTemp;
  size_t realParams = 0;          ///< The number of non-flag parameters we've seen.

  // Parse the command line arguments, with the first non-flag argument being the
  // name of the IP address to listen on.
  for (int i = 1; i < argc; ++i) {
    if (std::string("--help") == argv[i]) {
      usage(argv[0]);
      return 0;
    } else switch (realParams++) {
    case 0:
      camConfigFile = argv[i];
      break;
    case 1:
      posesFile = argv[i];
      break;
    case 2:
      imageDirectory = argv[i];
      break;
    case 3:
      lowTemp = std::stod(argv[i]);
      break;
    case 4:
      highTemp = std::stod(argv[i]);
      break;
    case 5:
      minTemp = std::stod(argv[i]);
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
    std::cout << "Camera_Calibration_Gain_And_Offsets version " << VERSION << std::endl;

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

    // Loop over all the poses and associated images; there should be one pose for each camera.
    std::map<CameraRenderInfo*, double> hardwareOffsets; // The calculated hardware offset for each camera.
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

      // Make a median-filtered version of the average image to remove single stuck pixels.
      size_t pixelCount = width * height;
      std::shared_ptr< std::vector<double> > filteredBuffer = std::make_shared< std::vector<double> >(pixelCount);
      for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
          std::vector<double> neighbors;
          // Collect the 3x3 neighborhood.
          for (int dy = -1; dy <= 1; dy++) {
            for (int dx = -1; dx <= 1; dx++) {
              int nx = x + dx;
              int ny = y + dy;
              if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                double neighborPixel = avg->read_pixel_nocheck(nx, ny);
                neighbors.push_back(neighborPixel);
              }
            }
          }
          // Sort the neighbors and take the median.
          std::sort(neighbors.begin(), neighbors.end());
          double medianValue = neighbors[neighbors.size() / 2];
          double* pixel = &(*filteredBuffer)[y * width + x];
          *pixel = medianValue;
        }
      }

      // Find the minimum and maximum pixel values in the filtered image.
      double minPixel = 1e30;
      double maxPixel = -1e30;
      for (auto pixel : *filteredBuffer) {
        if (pixel < minPixel) {
          minPixel = pixel;
        }
        if (pixel > maxPixel) {
          maxPixel = pixel;
        }
      }

      // Compute the gain as the slope of the line from minimum to maximum temperature, dividing the
      // temperature range by the pixel value range.
      double tempRange = highTemp - lowTemp;
      double pixelRange = maxPixel - minPixel;
      double gain = (pixelRange != 0) ? (tempRange / pixelRange) : 1.0;

      // Find the pixel value that corresponds to the temperature zero based on the gain and the minimum temperature.
      // This is the offset that needs to be applied in the camera system.  This offset is the negative of this.
      double zeroValue = (0 - lowTemp) / gain + minPixel;
      double viewOffset = -zeroValue;

      // Find the pixel value that corresponds to the specified minimum temperature that will be visible to the ball.
      // This is the hardware offset that needs to be applied in the camera system to ensure that the specified
      // minimum temperature is visible to the ball.
      double minValue = (minTemp - lowTemp) / gain + minPixel;
      double hardwareOffset = -minValue;

      // Update the offset and gain for this camera and tell what we did.
      // Make a critical section to avoid thread contention during this time.
#pragma omp critical
      {
        std::cout << count << " / " << poseInfos.size() << " processed; pose " << pose.frameIndex
          << " for camera " << pose.cameraID << "\n";
        std::cout << "  Gain: " << gain << "\n";
        std::cout << "  View Offset: " << viewOffset << std::endl;
        std::cout << "  Hardware Offset: " << hardwareOffset << std::endl;
        cri->SetColorOffsetGain(viewOffset, gain);

        hardwareOffsets[cri] = hardwareOffset;

        count++;
      }
    }

    // Parse the JSON configuration file for the camera configuration directly, then replace
    // the offset and gain parameters for each camera with the calculated values
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
          float offset, gain;
          cri.GetColorOffsetGain(offset, gain);
          camera["color"]["gain"] = gain;
          camera["color"]["offset"] = offset;
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

    // Write the hardware offsets to standard output, sorting them by camera ID.
    std::vector<std::pair<int, double> > sortedOffsets;
    for (const auto& entry : hardwareOffsets) {
      sortedOffsets.push_back({ entry.first->m_ID, entry.second });
    }
    std::sort(sortedOffsets.begin(), sortedOffsets.end());
    std::cout << "Hardware Offsets (to be applied in the camera system):" << std::endl;
    for (const auto& entry : sortedOffsets) {
      std::cout << "  Camera ID " << entry.first << ", " << entry.second << std::endl;
    }

  } // End of block to ensure that all objects are destructed before we exit.

  return 0;
}
