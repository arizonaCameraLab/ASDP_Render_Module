/*
 * Copyright (C) 2025: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include <iostream>
#include <memory>
#include <string>
#include <Gimbal.h>

static void usage(const char* progName)
{
  std::cerr << "Usage: " << progName << " [--home] [--speed <degPerSec>] [--accel <degPerSec2>] [--moveTo <yawDeg> <pitchDeg>] COM_DEVICE" << std::endl;
  std::cerr << "       COM_DEVICE: The COM port device name (e.g., /dev/ttyUSB0 or COM3)." << std::endl;
  std::cerr << "       --home: Move to home position." << std::endl;
  std::cerr << "       --speed <degPerSec>: Set the speed in degrees/second." << std::endl;
  std::cerr << "       --accel <degPerSec2>: Set the acceleration in degrees/second^2." << std::endl;
  std::cerr << "       --moveTo <yawDeg> <pitchDeg>: Move to the specified yaw and pitch angles." << std::endl;
}

int main(int argc, char** argv)
{
  std::string comDevice;
  double maxVelocityDeg = -1;
  double maxAccelDeg = -1;
  double yawDegrees = 0;
  double pitchDegrees = 0;
  bool home = false;
  bool moveTo = false;

  size_t realParams = 0;
  for (int i = 1; i < argc; ++i) {
    if (std::string("--home") == argv[i]) {
      home = true;
    }
    else if (std::string("--speed") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 1;
      }
      maxVelocityDeg = std::stod(argv[i]);
    }
    else if (std::string("--accel") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 1;
      }
      maxAccelDeg = std::stod(argv[i]);
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
        comDevice = argv[i];
        break;
      default:
        usage(argv[0]);
        return 1;
    }
  }
  if (realParams != 1) {
    usage(argv[0]);
    return 1;
  }

  // Put the code into a block so that objects will be go out of scope before we exit.
  {
    // Open the gimbal on the specified COM port, setting its speed and acceleration if provided.
    // Then verify the gimbal is connected and operational.
    std::shared_ptr<Gimbal> gimbal = std::make_shared<GimbalZaber_X_G_RST>(comDevice, 1, maxVelocityDeg, maxAccelDeg);
    if (!gimbal->Status()) {
      std::cerr << "Gimbal not connected or not operational." << std::endl;
      return 10;
    }

    // If we've been asked to home the gimbal, do so.
    if (home) {
      if (!gimbal->Home()) {
        std::cerr << "Failed to home the gimbal." << std::endl;
        return 20;
      }
    }

    // If we've been asked to move to a specific position, do so.
    if (moveTo) {
      if (!gimbal->MoveAbsolute(yawDegrees, pitchDegrees)) {
        std::cerr << "Failed to move the gimbal to the specified position." << std::endl;
        return 30;
      }
    }
  }

  // Done
  return 0;
}
