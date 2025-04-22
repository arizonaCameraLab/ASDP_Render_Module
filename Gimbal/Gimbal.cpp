/*
 * Copyright (C) 2025: Arizona Board of Regents on Behalf of the University of Arizona
 */

 /**
  * @file Gimbal.cpp
  * @brief Classes to control orientation gimbals.
  *
 * @author ReliaSolve.
 * @date April 8th, 2025.
 */

#include "Gimbal.h"
#include <vrpn_Serial.h>
#include <cstdint>
#include <vector>
#include <iostream>

bool GimbalFake::MoveAbsolute(double yawDegrees, double pitchDegrees)
{
  std::cout << "GimbalFake: Move to (" << yawDegrees << ", " << pitchDegrees << ")" << std::endl;
  return true;
}

class GimbalZaber_X_G_RST::GimbalZaber_X_G_RST_Impl
{
public:
  int deviceID = -1;    ///< The device ID of the gimbal we're using.
  int commPort = -1;    ///< Theserial port index we're using.
  //bool status = false;  ///< The status of the gimbal, true = good, false = broken.

  // The microstep size is at the default resolution of 1/64th step is 0.00015625 degrees,
  // from the user manual.
  // The range of of valid values for resolution is 1 to 256, from the API manual.
  double microstepSize = 0.00015625;
  double maxSpeed = 24.0; ///< The maximum speed in degrees/second, from the user manual.

  int speed = 0;          ///< The speed to move at in degrees/second.
  int accel = 0;          ///< The acceleration to move at in degrees/second^2.

  // Helper functions.

  /// @brief Send a command to the gimbal without waiting for a response.
  /// @param axis The axis number on the device to send the command to.
  /// @param cmd The command to send, including all parameters after spaces.
  /// @return True if the command was sent successfully, false otherwise.
  bool sendCommand(uint8_t axis, std::string cmd);

  struct Response {
    int deviceID = -1;    ///< The device ID of the sender
    int scope = -1;       ///< The scope of the response (0 = device, 1 = axis 1, 2 = axis 2)
    bool success = false; ///< True if the command was successful, false otherwise
    bool idle = false;    ///< True if the device is idle, false if it is busy
    bool faults = true;   ///< True if the device has faults, false otherwise
    std::string data;     ///< The return data, variable by command
  };

  /// @brief Parse a response from the gimbal.
  /// @param buffer The buffer containing the response data.  It is guaranteed by the caller
  /// to start with the @ sign and end with carriage-return, line-feed.
  /// @param out The response structure to fill in with the parsed data.
  /// @return True if the response was parsed successfully, false otherwise.
  bool parseResponse(uint8_t* buffer, Response &out);

  /// @brief Await a response from some device.
  /// @param timeout The maximum amount of time to wait for a response.  A null pointer will
  /// cause it to wait forever, a pointer to a zero time will return immediately.
  /// @return A vector of structures containing the responses from all devices.  If there was
  /// not a complete response, the vector will be empty.
  std::vector<Response> getResponses(struct timeval *timeout);

  unsigned valueForLocation(double degrees) {
    // The conversion from counts to real-world position is as follows (from the API documentation):
    // position = data * microstepSize;
    // Therefore, data = position / microstepSize;
    return degrees / microstepSize;
  }

  unsigned valueForSpeed(double degreesPerSecond)   {
    // The conversion from counts to real-world velocity is as follows (from the API documentation):
    // velocity = data * microstepSize / 1.6384;
    // Therefore, data = velocity * 1.6384 / microstepSize;
    return degreesPerSecond * 1.6384 / microstepSize;
  }

  unsigned valueForAccel(double degreesPerSecondSquared) {
    // The conversion from counts to real-world acceleration is as follows (from the API documentation):
    // acceleration = data * microstepSize * 10000 / 1.6384;
    // Therefore, data = acceleration * 1.6384 / (microstepSize * 10000);
    return degreesPerSecondSquared * 1.6384 / (microstepSize * 10000);
  }

  // These are not strictly needed because we can set the speed and acceleration for each move command.
  bool setSpeed(double degreesPerSecond);
  bool setAccel(double degreesPerSecondSquared);

  /// @brief Get the status for a particular axis.
  /// @param axis The axis number to get the status for.
  /// @param out A response structure to fill in with the status data.
  /// @return True if things are working well, false if there are problems or the wrong axis responds.
  bool getStatus(uint8_t axis, Response& out);

  /// @brief Move the gimbal to the home position.
  /// @details The home position is defined as the position where the gimbal is on its limit switches.
  /// @return True if the move was successful, false otherwise.
  bool home();

  /// @brief Move at the velocity and acceleration defined in the constructor to this absolute location.
  bool moveAbsolute(double yawDegrees, double pitchDegrees);
};

bool GimbalZaber_X_G_RST::GimbalZaber_X_G_RST_Impl::sendCommand(uint8_t axis, std::string cmd)
{
  if (commPort == -1) {
    return false;
  }

  // Construct the command string.
  std::string command = "/" + std::to_string(deviceID) + " " + std::to_string(axis) + " " + cmd + "\n";

  // Send the command to the gimbal.
  int result = vrpn_write_characters(commPort, (unsigned char*)(command.c_str()), command.length());
  return result == command.length();
}

bool GimbalZaber_X_G_RST::GimbalZaber_X_G_RST_Impl::parseResponse(uint8_t* buffer, Response& out)
{
  // Clear the response to its default values.
  out = Response();

  // The first character should be the @ sign, which indicates the start of a response.
  // We then have the following space-separated values:
  // Device ID (2 bytes), Axis number (1 byte), reply flag (2 bytes, 'OK' if good), status ('IDLE' or 'BUSY'),
  // warning flag (2 bytes, '--' if okay), data (often '0', varies by command).

  // Verify that the places where we expect spaces are spaces.
  if (buffer[3] != ' ' || buffer[5] != ' ' || buffer[8] != ' ' || buffer[11] != ' ' ||
      buffer[15] != ' ' || buffer[18] != ' ') {
    return false;
  }

  // Parse each field of the response.
  char deviceStr[3] = {};
  deviceStr[0] = buffer[1];
  deviceStr[1] = buffer[2];
  out.deviceID = atoi(deviceStr);

  char axisStr[2] = {};
  axisStr[0] = buffer[4];
  out.scope = atoi(axisStr);

  if (buffer[6] == 'O' && buffer[7] == 'K') {
    out.success = true;
  }

  char statusStr[5] = {};
  statusStr[0] = buffer[9];
  statusStr[1] = buffer[10];
  statusStr[2] = buffer[11];
  statusStr[3] = buffer[12];
  out.idle = std::string(statusStr) == "IDLE";

  if (buffer[14] == '-' && buffer[15] == '-') {
    out.faults = false;
  } else {
    out.faults = true;
  }

  uint8_t* status = buffer + 19;
  while (*status != '\r') {
    out.data += *status;
  }

  return true;
}

std::vector<GimbalZaber_X_G_RST::GimbalZaber_X_G_RST_Impl::Response>
  GimbalZaber_X_G_RST::GimbalZaber_X_G_RST_Impl::getResponses(struct timeval *timeout)
{
  std::vector<Response> responses;

  // Allocate a buffer that is large enough to hold the longest response and then try to
  // read that many characters.
  std::vector<uint8_t> buffer(1024);
  int ret = vrpn_read_available_characters(commPort, buffer.data(), buffer.size(), timeout);

  // If we got nothing, return an empty response.
  if (ret <= 1) {
    return responses;
  }

  // Start parsing responses from the buffer.  If we start in the middle of a response, insert a
  // bad (default) response.
  size_t i = 0;
  if (buffer[0] != '@') {
    responses.push_back(Response());
    while (++i < buffer.size() && buffer[i] != '@') {
      // Skip over the bad data.
    }
  }

  // We are now at the start of the next response in the buffer, if there is one there.
  // Parse until we run out of them.
  while (i < buffer.size() && buffer[i] == '@') {
    // Find the end of the response, which is marked by carriage-return/newline.
    // If we don't find this, then the response is incomplete and we insert a
    // broken response.
    size_t j = i + 2;
    while (j < buffer.size() && (buffer[j-1] != '\r' || buffer[j] != '\n')) {
      ++j;
    }
    if (j >= buffer.size()) {
      responses.push_back(Response());
      break;
    }

    // Try to parse the response. If we fail, insert a broken response and we're done.
    Response res;
    if (parseResponse(buffer.data() + i, res)) {
      responses.push_back(res);
    } else {
      responses.push_back(Response());
      break;
    }

    // Move to the next response.
    i = j + 1;
  }

  return responses;
}

bool GimbalZaber_X_G_RST::GimbalZaber_X_G_RST_Impl::setSpeed(double degreesPerSecond)
{
  if (degreesPerSecond <= 0 || degreesPerSecond > maxSpeed) {
    return false;
  }

  unsigned data = valueForSpeed(degreesPerSecond);

  for (int axis = 1; axis <= 2; axis++) {
    // Set the speed on both axes on the device, awaiting a response from each axis.
    if (!sendCommand(axis, "set maxspeed " + std::to_string(data))) {
      return false;
    }
    struct timeval timeout = { 0, 500000 };
    std::vector<Response> responses = getResponses(&timeout);
    if (responses.size() != 1) {
      return false;
    }
    Response const& r = responses[0];
    if (r.deviceID != deviceID || r.scope != axis || !r.success || r.faults) {
      return false;
    }
  }

  return true;
}

bool GimbalZaber_X_G_RST::GimbalZaber_X_G_RST_Impl::setAccel(double degreesPerSecondSquared)
{
  if (degreesPerSecondSquared <= 0) {
    return false;
  }

  unsigned data = valueForAccel(degreesPerSecondSquared);

  for (int axis = 1; axis <= 2; axis++) {
    // Set the speed on both axes on the device, awaiting a response within 10ms from each axis.
    if (!sendCommand(axis, "set maxspeed " + std::to_string(data))) {
      return false;
    }
    struct timeval timeout = { 0, 10000 };
    std::vector<Response> responses = getResponses(&timeout);
    if (responses.size() != 1) {
      return false;
    }
    Response const& r = responses[0];
    if (r.deviceID != deviceID || r.scope != axis || !r.success || r.faults) {
      return false;
    }
  }

  return true;
}

bool GimbalZaber_X_G_RST::GimbalZaber_X_G_RST_Impl::getStatus(uint8_t axis, Response& out)
{
  if (commPort == -1) {
    return false;
  }

  // Send the status command to the axis.
  if (!sendCommand(axis, "get maxspeed")) {
    return false;
  }

  // Wait for a response from the axis.
  struct timeval timeout = { 0, 10000 };
  std::vector<Response> responses = getResponses(&timeout);
  if (responses.size() != 1) {
    return false;
  }
  Response const& r = responses[0];
  if (r.deviceID != deviceID || r.scope != axis || !r.success || r.faults) {
    return false;
  }
  out = r;
  return true;
}

bool GimbalZaber_X_G_RST::GimbalZaber_X_G_RST_Impl::home()
{
  // Send the home command to each axis, awaiting a response within 10ms from each axis.
  for (int axis = 1; axis <= 2; axis++) {
    if (!sendCommand(axis, "home")) {
      return false;
    }
    struct timeval timeout = { 0, 10000 };
    std::vector<Response> responses = getResponses(&timeout);
    if (responses.size() != 1) {
      return false;
    }
    Response const& r = responses[0];
    if (r.deviceID != deviceID || r.scope != axis || !r.success || r.faults) {
      return false;
    }
  }
  // Wait for both axes to come to a halt.
  bool axisRunning;
  do {
    axisRunning = false;
    for (int axis = 1; axis <= 2; axis++) {
      Response status;
      if (!getStatus(axis, status)) {
        return false;
      }
      if (!status.idle) {
        axisRunning = true;
      }
    }
  } while (axisRunning);
  return true;
}

bool GimbalZaber_X_G_RST::GimbalZaber_X_G_RST_Impl::moveAbsolute(double yawDegrees, double pitchDegrees)
{
  // Send the motion command to each axis, scaling the physical units to device units.
  std::vector<double> axes = { yawDegrees, pitchDegrees };
  for (int axis = 1; axis <= 2; axis++) {
    unsigned locationData = valueForLocation(axes[axis - 1]);
    unsigned speedData = valueForSpeed(speed);
    unsigned accelData = valueForAccel(accel);
    // Send move on both axes on the device, awaiting a response within 10ms from each axis.
    if (!sendCommand(axis, "move abs " + std::to_string(locationData))) {
      return false;
    }
    struct timeval timeout = { 0, 10000 };
    std::vector<Response> responses = getResponses(&timeout);
    if (responses.size() != 1) {
      return false;
    }
    Response const& r = responses[0];
    if (r.deviceID != deviceID || r.scope != axis || !r.success || r.faults) {
      return false;
    }
  }

  // Wait for both axes to come to a halt.
  bool axisRunning;
  do {
    axisRunning = false;
    for (int axis = 1; axis <= 2; axis++) {
      Response status;
      if (!getStatus(axis, status)) {
        return false;
      }
      if (!status.idle) {
        axisRunning = true;
      }
    }
  } while (axisRunning);

  return true;
}

GimbalZaber_X_G_RST::GimbalZaber_X_G_RST(std::string comPortName, int deviceID,
    double maxVelocityDeg, double maxAccelDeg)
  : m_impl(new GimbalZaber_X_G_RST_Impl())
{
  // Open the serial port using the defaults of 8 bits, no parity, 1 start and stop bits with no
  // RTS (hardware) flow control.
  m_impl->commPort = vrpn_open_commport(comPortName.c_str(), 115200);
  if (m_impl->commPort != -1) {
    // Set the device parameters
    m_impl->deviceID = deviceID;
    m_impl->speed = maxVelocityDeg;
    m_impl->accel = maxAccelDeg;

    // Gobble up any full or partial responses that have come from the device.
    std::vector<GimbalZaber_X_G_RST_Impl::Response> responses;
    do {
      struct timeval timeout = { 0, 0 };
      responses = m_impl->getResponses(&timeout);
    } while (responses.size() > 0);

    // If we've been asked to set the speed, do so for both axes.
    if (maxVelocityDeg > 0) {
      m_impl->setSpeed(maxVelocityDeg);
    }

    // If we've been asked to set the acceleration, do so for both axes.
    if (maxAccelDeg > 0) {
      m_impl->setAccel(maxAccelDeg);
    }

  } 
}

bool GimbalZaber_X_G_RST::Status()
{
  if (!m_impl) {
    return false;
  }

  // Query the status of the device and verify that none of the error or warning
  // fields are filled in.
  GimbalZaber_X_G_RST_Impl::Response status;
  for (int axis = 1; axis <= 2; axis++) {
    if (!m_impl->getStatus(axis, status)) {
      return false;
    }
  }
  return true;
}

GimbalZaber_X_G_RST::~GimbalZaber_X_G_RST()
{
  if (m_impl) {
    if (m_impl->commPort != -1) {
      vrpn_close_commport(m_impl->commPort);
    }
  }
}

bool GimbalZaber_X_G_RST::Home()
{
  if (!m_impl) {
    return false;
  }
  // Move the gimbal to the specified absolute position.
  return m_impl->home();
}

bool GimbalZaber_X_G_RST::MoveAbsolute(double yawDegrees, double pitchDegrees)
{
  if (!m_impl) {
    return false;
  }
  // Move the gimbal to the specified absolute position.
  return m_impl->moveAbsolute(yawDegrees, pitchDegrees);
}
