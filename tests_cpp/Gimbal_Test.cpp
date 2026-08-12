/*
 * Copyright (C) 2025: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include <iostream>
#include <memory>
#include <string>
#include <Gimbal.h>
#include <Calibration_Helpers.h>

static void usage(const char* progName)
{
  std::cerr << "Usage: " << progName << " [--home] [--gimbalConfig <filename>] [--speed <degPerSec>] [--accel <degPerSec2>] [--moveTo <yawDeg> <pitchDeg>]" << std::endl;
  std::cerr << "       --gimbalConfig <string>: The gimbal configuration file name (default gimbal.json)." << std::endl;
  std::cerr << "       --home: Move to home position." << std::endl;
  std::cerr << "       --moveTo <yawDeg> <pitchDeg>: Move to the specified yaw and pitch angles." << std::endl;
  std::cerr << "       --help: Print this information and quit." << std::endl;
}

int main(int argc, char** argv)
{
  std::string gimbalConfigFile = "gimbal.json";
  double yawDegrees = 0;
  double pitchDegrees = 0;
  bool home = false;
  bool moveTo = false;

  size_t realParams = 0;
  for (int i = 1; i < argc; ++i) {
    if (std::string("--home") == argv[i]) {
      home = true;
    }
    else if (std::string("--gimbalConfig") == argv[i]) {
      if (++i >= argc) {
        std::cerr << "Error: Missing gimbal configuration file name." << std::endl;
        return 1;
      }
      gimbalConfigFile = argv[i];
    }
    else if (std::string("--help") == argv[i]) {
      usage(argv[0]);
      return 0;
    }
    else if (std::string("--moveTo") == argv[i]) {
      moveTo = true;
      if (++i >= argc) {
        usage(argv[0]);
        return 1;
      }
      yawDegrees = std::stod(argv[i]);
      if (++i >= argc) {
        usage(argv[0]);
        return 1;
      }
      pitchDegrees = std::stod(argv[i]);
    }
    else if (argv[i][0] == '-') {
      usage(argv[0]);
      return 1;
    }
    else switch (++realParams) {
      case 1:
      default:
        usage(argv[0]);
        return 1;
    }
  }
  if (realParams != 0) {
    usage(argv[0]);
    return 1;
  }

  // Put the code into a block so that objects will be go out of scope before we exit.
  {
    // Read the gimbal configuration file.
    asdp::render::calibration::GimbalInfo gimbalInfo;
    try {
      gimbalInfo = asdp::render::calibration::GetGimbalInfo(gimbalConfigFile);
    }
    catch (const std::exception& e) {
      std::cerr << "Error: Unable to read gimbal configuration file: " << e.what() << std::endl;
      return 2;
    }

    // Open the gimbal on the specified COM port, setting its speed and acceleration if provided.
    // Then verify the gimbal is connected and operational.
    std::shared_ptr<Gimbal> gimbal;
    std::cout << "Opening gimbal on " << gimbalInfo.comPort << std::endl;
    try {
      gimbal = ConstructGimbal(gimbalInfo);
    }
    catch (const std::exception& e) {
      std::cerr << "Error: Unable to construct gimbal: " << e.what() << std::endl;
      return 3;
    }    
    std::cout << "Getting status from gimbal" << std::endl;
    if (!gimbal->Status()) {
      std::cerr << "Gimbal not connected or not operational." << std::endl;
      return 10;
    }
    std::cout << "Gimbal is operational" << std::endl;

    // If we've been asked to home the gimbal, do so.
    if (home) {
      std::cout << "Homing gimbal" << std::endl;
      try {
        gimbal->Home();
      }
      catch (const std::exception& e) {
        std::cerr << "Failed to home the gimbal: " << e.what() << std::endl;
        return 20;
      }
    }

    // If we've been asked to move to a specific position, do so.
    if (moveTo) {
      std::cout << "Moving gimbal to yaw: " << yawDegrees << ", pitch: " << pitchDegrees << std::endl;
      try {
        gimbal->MoveAbsolute(yawDegrees, pitchDegrees);
      }
      catch (const std::exception& e) {
        std::cerr << "Failed to move the gimbal to the specified position: " << e.what() << std::endl;
        return 30;
      }
    }
  }

  // Done
  return 0;
}
