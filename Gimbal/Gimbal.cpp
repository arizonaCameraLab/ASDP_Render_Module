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
#include <thread>
#include <cmath>

void GimbalFake::Home()
{
  std::cout << "GimbalFake: Home" << std::endl;
}

void GimbalFake::MoveAbsolute(double yawDegrees, double pitchDegrees)
{
  std::cout << "GimbalFake: Move to (" << yawDegrees << ", " << pitchDegrees << ")" << std::endl;
}

class GimbalZaber_X_G_RST::GimbalZaber_X_G_RST_Impl
{
public:
  int commPort = -1;    ///< The serial port index we're using.
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
  /// @param device The device number to send the command to.
  /// @param cmd The command to send, including all parameters after spaces.
  /// @return True if the command was sent successfully, false otherwise.
  bool sendCommand(uint8_t device, std::string cmd);

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

  int valueForLocation(double degrees) {
    // The conversion from counts to real-world position is as follows (from the API documentation):
    // position = data * microstepSize;
    // Therefore, data = position / microstepSize;
    // The direction of motion is the opposite of the direction of the gimbal, so we flip the sign.
    return -degrees / microstepSize;
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

  /// @brief Get the status for a particular device.
  /// @param device The device number to get the status for.
  /// @param out A response structure to fill in with the status data.
  /// @return True if things are working well, false if there are problems or the wrong device responds.
  bool getStatus(uint8_t device, Response& out);

  /// @brief Move the gimbal to the home position.
  /// @details The home position is defined as the position where the gimbal is on its limit switches.
  /// @return Empty string if successful, error message otherwise.
  std::string home();

  /// @brief Move at the velocity and acceleration defined in the constructor to this absolute location.
  /// @param yawDegrees The yaw angle in degrees.
  /// @param pitchDegrees The pitch angle in degrees.
  /// @return Empty string if successful, error message otherwise.
  std::string moveAbsolute(double yawDegrees, double pitchDegrees);
};

bool GimbalZaber_X_G_RST::GimbalZaber_X_G_RST_Impl::sendCommand(uint8_t device, std::string cmd)
{
  if (commPort == -1) {
    return false;
  }

  // Construct the command string.
  int axis = 1;
  std::string command = "/" + std::to_string(device) + " " + std::to_string(axis) + " " + cmd + "\n";

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
  if (buffer[3] != ' ' || buffer[5] != ' ' || buffer[8] != ' ' || buffer[13] != ' ' ||
      buffer[16] != ' ') {
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

  // The return code WR means that there is no reference position.
  if (buffer[14] == '-' && buffer[15] == '-') {
    out.faults = false;
  } else {
    out.faults = true;
  }

  uint8_t* status = buffer + 17;
  while (*status != '\r') {
    out.data += *(status++);
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
  if (ret <= 0) {
    return responses;
  }

  // Start parsing responses from the buffer.  If we start in the middle of a response, insert a
  // bad (default) response.
  size_t i = 0;
  if (buffer[0] != '@') {
    // Skip over info and alert messages, but push a bad response if we get something else.
    if (buffer[0] != '!' && buffer[0] != '#') {
      responses.push_back(Response());
    }
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

  for (int device = 1; device <= 2; device++) {
    // Set the speed on both devices, awaiting a response from each device.
    if (!sendCommand(device, "set maxspeed " + std::to_string(data))) {
      return false;
    }
    struct timeval timeout = { 0, 100000 };
    std::vector<Response> responses = getResponses(&timeout);
    if (responses.size() != 1) {
      return false;
    }
    Response const& r = responses[0];
    if (r.deviceID != device || r.scope != 1 || !r.success || r.faults) {
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

  for (int device = 1; device <= 2; device++) {
    // Set the speed on both axes on the device, awaiting a response within 10ms from each device.
    if (!sendCommand(device, "set accel " + std::to_string(data))) {
      return false;
    }
    struct timeval timeout = { 0, 100000 };
    std::vector<Response> responses = getResponses(&timeout);
    if (responses.size() != 1) {
      return false;
    }
    Response const& r = responses[0];
    if (r.deviceID != device || r.scope != 1 || !r.success || r.faults) {
      return false;
    }
  }

  return true;
}

bool GimbalZaber_X_G_RST::GimbalZaber_X_G_RST_Impl::getStatus(uint8_t device, Response& out)
{
  if (commPort == -1) {
    return false;
  }

  // Send the status command to the device.
  if (!sendCommand(device, "get maxspeed")) {
    return false;
  }

  // Wait for a response from the device.
  struct timeval timeout = { 0, 100000 };
  std::vector<Response> responses = getResponses(&timeout);
  if (responses.size() != 1) {
    return false;
  }
  Response const& r = responses[0];
  if (r.deviceID != device || r.scope != 1 || !r.success) {
    return false;
  }
  out = r;
  return true;
}

std::string GimbalZaber_X_G_RST::GimbalZaber_X_G_RST_Impl::home()
{
  // Send the home command to each device, awaiting a response within 10ms from each device.
  for (int device = 1; device <= 2; device++) {
    if (!sendCommand(device, "home")) {
      return "Could not send home command";
    }
    struct timeval timeout = { 0, 100000 };
    std::vector<Response> responses = getResponses(&timeout);
    if (responses.size() != 1) {
      return "No response to home command";
    }
    Response const& r = responses[0];
    if (r.deviceID != device || r.scope != 1 || !r.success) {
      return "Failure reported after home command";
    }
  }
  // Wait for both axes to come to a halt.
  bool deviceRunning;
  do {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    deviceRunning = false;
    for (int device = 1; device <= 2; device++) {
      Response status;
      if (!getStatus(device, status)) {
        return "Bad status after home command, device " + std::to_string(device);
      }
      if (!status.idle) {
        deviceRunning = true;
      }
    }
  } while (deviceRunning);
  return "";
}

std::string GimbalZaber_X_G_RST::GimbalZaber_X_G_RST_Impl::moveAbsolute(double yawDegrees, double pitchDegrees)
{
  // Send the motion command to each device, scaling the physical units to device units.
  std::vector<double> axes = { yawDegrees, pitchDegrees };
  for (int device = 1; device <= 2; device++) {
    int locationData = valueForLocation(axes[device - 1]);
    unsigned speedData = valueForSpeed(speed);
    unsigned accelData = valueForAccel(accel);
    // Send move on both axes on the device, awaiting a response within 10ms from each device.
    if (!sendCommand(device, "move abs " + std::to_string(locationData))) {
      return "Could not send move command";
    }
    struct timeval timeout = { 0, 100000 };
    std::vector<Response> responses = getResponses(&timeout);
    if (responses.size() != 1) {
      return "No response to move command";
    }
    Response const& r = responses[0];
    if (r.deviceID != device || r.scope != 1 || !r.success || r.faults) {
      return "Failure reported after move command";
    }
  }

  // Wait for both axes to come to a halt.
  bool deviceRunning;
  do {
    deviceRunning = false;
    for (int device = 1; device <= 2; device++) {
      Response status;
      if (!getStatus(device, status)) {
        return "Bad status after move command";
      }
      if (!status.idle) {
        deviceRunning = true;
      }
    }
  } while (deviceRunning);

  return "";
}

GimbalZaber_X_G_RST::GimbalZaber_X_G_RST(std::string comPortName, int deviceID,
    double maxVelocityDeg, double maxAccelDeg)
  : m_impl(new GimbalZaber_X_G_RST_Impl())
{
  // Open the serial port using the defaults of 8 bits, no parity, 1 start and stop bits with no
  // RTS (hardware) flow control.
  m_impl->commPort = vrpn_open_commport(comPortName.c_str(), 115200);
  if (m_impl->commPort == -1) {
    throw std::runtime_error("Unable to open COM port " + comPortName);
  }
  if (m_impl->commPort != -1) {
    // Set the device parameters
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
  for (int device = 1; device <= 2; device++) {
    if (!m_impl->getStatus(device, status)) {
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

void GimbalZaber_X_G_RST::Home()
{
  if (!m_impl) {
    throw std::runtime_error("No implementation defined");
  }
  // Move the gimbal to the specified absolute position.
  std::string res = m_impl->home();
  if (!res.empty()) {
    throw std::runtime_error("Error homing gimbal: " + res);
  }
}

void GimbalZaber_X_G_RST::MoveAbsolute(double yawDegrees, double pitchDegrees)
{
  if (!m_impl) {
    throw std::runtime_error("No implementation defined");
  }
  // Move the gimbal to the specified absolute position.
  std::string res = m_impl->moveAbsolute(yawDegrees, pitchDegrees);
  if (!res.empty()) {
    throw std::runtime_error("Error moving gimbal: " + res);
  }
}

class Gimbal_iOptron::Gimbal_iOptron_Impl
{
public:
  int commPort = -1;    ///< The serial port index we're using.

  /// @brief Send a command to the gimbal without waiting for a response.
  /// @param cmd The command to send, including all parameters after spaces.
  /// @return True if the command was sent successfully, false otherwise.
  bool sendCommand(std::string cmd);

  /// @brief Get a response from the gimbal.
  /// @param timeout The maximum amount of time to wait for a response.  A null pointer will
  /// wait forever.
  /// @param numChars The number of characters to read from the gimbal.
  /// @return The response from the gimbal, or an empty string if there was no response.
  std::string getResponse(struct timeval* timeout, size_t numChars);

  /// @brief Send a command to the gimbal and check the response.  Close comms on failure.
  /// @param cmd The command to send, including all parameters after spaces.
  /// @param response The expected response from the gimbal.
  /// @return True if the command was sent successfully and the response was as expected, false otherwise.
  bool sendCommandCheckReponseAndFail(std::string cmd, std::string response);

  /// @brief Wait until the gimbal stops slewing.
  /// @param maxTiltDegrees The maximum tilt in degrees to allow before stopping the gimbal to avoid crashing.
  /// Normal range should be around 80 degrees, but the home command can allow unlimited.  This must be
  /// less than 90 degrees for actual motion cases because the tests will otherwise overlap and it will never stop.
  /// When it was 88 degrees, we sometimes moved over the full 4-degree range without detecting it because of slow
  /// responses from the unit.
  /// @param timeout The maximum amount of time to wait for a response.
  /// @return Empty string on succes, error message on failure.
  std::string waitForSlewStop(double maxTiltDegrees, std::chrono::milliseconds timeout = std::chrono::milliseconds(60000));

  /// @brief Reset the time on the gimbal to one that has the Earth aligned with Celestial coordinates.
  /// @details This selects a time when RA is 0, which is not the beginning of the epoch (because the
  /// Earth points in a different direction then) but is within a day later than that.
  /// @return Empty string on success, error message on failure.
  std::string resetTime();
};

bool Gimbal_iOptron::Gimbal_iOptron_Impl::sendCommand(std::string cmd)
{
  if (commPort == -1) {
    return false;
  }

  // Send the command to the gimbal.
  int result = vrpn_write_characters(commPort, (unsigned char*)(cmd.c_str()), cmd.length());
  return result == cmd.length();
}

std::string Gimbal_iOptron::Gimbal_iOptron_Impl::getResponse(struct timeval* timeout, size_t numChars)
{
  std::string response;
  if (commPort == -1) {
    return response;
  }

  // Allocate a buffer that is large enough to hold the longest response and then try to
  // read that many characters.
  std::vector<uint8_t> buffer(numChars);
  int ret = vrpn_read_available_characters(commPort, buffer.data(), buffer.size(), timeout);
  // If we got nothing, return an empty response.
  if (ret < 1) {
    return response;
  }

  // Copy the response into a string and return it.
  response.assign(buffer.begin(), buffer.begin() + ret);
  return response;
}

bool Gimbal_iOptron::Gimbal_iOptron_Impl::sendCommandCheckReponseAndFail(
  std::string cmd, std::string response)
{
  if (commPort == -1) {
    return false;
  }
  // Send the command to the gimbal.
  if (!sendCommand(cmd)) {
    return false;
  }
  // Wait for a response from the gimbal.
  struct timeval timeout = { 1, 0 };
  std::string resp = getResponse(&timeout, response.length());
  if (resp != response) {
    vrpn_close_commport(commPort);
    commPort = -1;
    return false;
  }
  return true;
}

std::string Gimbal_iOptron::Gimbal_iOptron_Impl::waitForSlewStop(double maxRADegrees, std::chrono::milliseconds timeout)
{
  if (commPort == -1) {
    return "commPort not initialized";
  }
  std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
  std::chrono::steady_clock::time_point end = start + timeout;
  while (std::chrono::steady_clock::now() <= end) {

    // Sleep to avoid flooding the gimbal with commands.
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Send the status command to the gimbal.
    if (!sendCommand(":GLS#")) {
      return "Could not send status request";
    }

    // Wait for a response from the gimbal.
    std::string example = "sTTTTTTTTTTTTTTTTnnnnnn#";
    struct timeval tv = { 0, 100000 };
    std::string resp = getResponse(&tv, example.size());
    if (resp.size() != example.size()) {
      // The unit sometimes sends a response that is shorter than expected or no response
      // while it is homing or slewing; ignore this.
      continue;
    }

    // See if slewing has stopped.  The 19th character is '2' if slewing is in progress, and '0' if it is not.
    if (resp[18] != '2') {
      // The gimbal is no longer slewing, so we can return.
      return "";
    }

    // Get the right ascension and declination. Make sure that we're not tipping over maxRADegrees degrees up
    // or down to avoid crashing the ball into the tripod.
    if (!sendCommand(":GEP#")) {
      return "Could not send position request";
    }

    example = "sTTTTTTTTTTTTTTTTTTT#";
    tv = { 0, 100000 };
    resp = getResponse(&tv, example.size());
    if (resp.size() != example.size()) {
      // The unit sometimes sends a response that is shorter than expected or no response
      // while it is homing or slewing; ignore this.
      continue;
    }

    // The valid range in degrees is from 0 to maxRADegrees and then from 360 down to 360-maxRADegrees.
    int RA = std::stoi(resp.substr(9, 9));
    double RAdeg = RA / (3600.0 * 100);
    if ((RAdeg > maxRADegrees) && (RAdeg < 360 - maxRADegrees)) {
      // If we're within the specified range of 180 degrees, then that is okay for some situations.
      if ((RAdeg < 180 - maxRADegrees) || (RAdeg > 180 + maxRADegrees)) {
        if (!sendCommandCheckReponseAndFail(":Q#", "1")) {
          if (!sendCommandCheckReponseAndFail(":Q#", "1")) {
            return "Could not send stop command after bypassing limits -- expect crash!";
          }
        }
        return "Gimbal is slewing to an unsafe position: " + std::to_string(RAdeg) + " degrees, stopped it from moving further.";
      }
    }
  }
  return "Timed out waiting for gimbal to stop slewing.";
}

std::string Gimbal_iOptron::Gimbal_iOptron_Impl::resetTime()
{
  // When the time is set to zero, we get an RA of 41:54.5, so we need to set the
  // time to 24 hours minus that, converted to milliseconds.
  // BUT when we do that, the time is still off by a little over 3 minutes, so we
  // use 44 minutes.  Then it is off by about 49 seconds, so we 54.5+49 = 1:43.5,
  // or 45 minutes, 43.5 seconds.  Then it is off by ~0.3 seconds, varying from run
  // to run depending on the time to run homing, so we tweak to 43.4.
  double minutes = 24 * 60 - (45 + (43.4 / 60.0));
  size_t milliseconds = static_cast<size_t>(minutes * 60 * 1000);
  std::string timeString = std::to_string(milliseconds);
  while (timeString.length() < 13) {
    timeString = "0" + timeString;
  }
  if (!sendCommandCheckReponseAndFail(":SUT" + timeString + "#", "1")) {
    return "Unable to send set time command";
  }

  return "";
}

Gimbal_iOptron::Gimbal_iOptron(std::string comPortName, std::string mountInfoResponse)
  : m_impl(new Gimbal_iOptron_Impl())
{
  // Retry opening the COM port a couple of times in case it is busy.
  bool success = false;
  for (int i = 0; i < 3 && !success; ++i) {
    // Open the serial port using the defaults of 8 bits, no parity, 1 start and stop bits with no
    // RTS (hardware) flow control.
    m_impl->commPort = vrpn_open_commport(comPortName.c_str(), 115200);
    if (m_impl->commPort == -1) {
      throw std::runtime_error("Unable to open COM port " + comPortName);
    }

    // Gobble up any full or partial responses that have come from the device.
    struct timeval timeout = { 0, 10000 };
    std::string gobble = m_impl->getResponse(&timeout, 10000);

    // Send a command to get the mount number.  Wait for a response and verify that
    // it matches what we expect for a CEM40.
    if (!m_impl->sendCommandCheckReponseAndFail(":MountInfo#", mountInfoResponse)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }
    success = true;
  }
  if (!success) {
    throw std::runtime_error("Unable to send MountInfo command");
  }

  // Send a command to set the maximum slew rate.
  if (!m_impl->sendCommandCheckReponseAndFail(":MSR9#", "1")) {
    throw std::runtime_error("Unable to send SlewRate command");
  }

  // Set the altitude limit to as low as possible.
  if (!m_impl->sendCommandCheckReponseAndFail(":SAL-89#", "1")) {
    throw std::runtime_error("Unable to send altitude limit command");
  }

  // Send a command to get the azimuth limit to be sure it worked.
  if (!m_impl->sendCommandCheckReponseAndFail(":GAL#", "-89#")) {
    throw std::runtime_error("Unable to send request azimuth limit command");
  }

  // Send a command to set the meridian treatment; flip at 15 degrees past.
  // This avoids a situation where the mount tries to take the long way around
  // when crossing the meridian, which can cause it to crash into the tripod.
  if (!m_impl->sendCommandCheckReponseAndFail(":SMT115#", "1")) {
    throw std::runtime_error("Unable to send meridian-treatment command");
  }

  // Disable tracking.
  if (!m_impl->sendCommandCheckReponseAndFail(":ST0#", "1")) {
    throw std::runtime_error("Unable to send stop-tracking command");
  }

  // Set the longitude to 0 degrees.
  if (!m_impl->sendCommandCheckReponseAndFail(":SLO+0000000#", "1")) {
    throw std::runtime_error("Unable to send set longitude command");
  }

  // Set the latitude to 0 degrees (equator).
  int lat = 0;
  int latTicks = static_cast<size_t>(lat * 3600.0) * 100;
  std::string latString = std::to_string(std::abs(latTicks));
  while (latString.length() < 8) {
    latString = "0" + latString;
  }
  if (!m_impl->sendCommandCheckReponseAndFail(":SLA+" + latString + "#", "1")) {
    throw std::runtime_error("Unable to send set latitude command");
  }

  // Set the offset from UTC to 0.
  if (!m_impl->sendCommandCheckReponseAndFail(":SG+000#", "1")) {
    throw std::runtime_error("Unable to send set UTC command");
  }

  // Set Daylight Savings Time to 0.
  if (!m_impl->sendCommandCheckReponseAndFail(":SDS0#", "1")) {
    throw std::runtime_error("Unable to send set DST command");
  }

  // Reset time to align the Earth with celestial coordinates, so RA=0.
  std::string ret = m_impl->resetTime();
  if (ret.size()) {
    throw std::runtime_error("Unable to reset time: " + ret);
  }
}

Gimbal_iOptron::~Gimbal_iOptron()
{
  if (m_impl && m_impl->commPort != -1) {
    vrpn_close_commport(m_impl->commPort);
  }
}

bool Gimbal_iOptron::Status()
{
  if (!m_impl || m_impl->commPort == -1) {
    return false;
  }
  return true;
}

void Gimbal_iOptron::Home()
{
  if (!m_impl || m_impl->commPort == -1) {
    throw std::runtime_error("No connection");
  }

  // Reset time to align the Earth with celestial coordinates, so RA=0.
  std::string ret = m_impl->resetTime();
  if (ret.size()) {
    throw std::runtime_error("Unable to reset time: " + ret);
  }
  // Send the home command to the gimbal.
  if (!m_impl->sendCommandCheckReponseAndFail(":MSH#", "1")) {
    throw std::runtime_error("Could not send home command");
  }

  // Wait for the gimbal to finish moving.
  ret = m_impl->waitForSlewStop(1000, std::chrono::milliseconds(60000));
  if (ret != "") {
    throw std::runtime_error("Timed out waiting for gimbal to finish moving: " + ret);
  }
}

void Gimbal_iOptron::MoveAbsolute(double yawDegrees, double pitchDegrees)
{
  if (!m_impl || m_impl->commPort == -1) {
    throw std::runtime_error("No connection");
  }

  double yawAdjusted = yawDegrees;
  double pitchAdjusted = pitchDegrees;

  // Ensure that we don't try to hit the rails.
  if (yawAdjusted > 175) {
    throw std::runtime_error("Yaw too large; limit is 175, value is " + std::to_string(yawAdjusted));
  }
  if (yawAdjusted < -175) {
    throw std::runtime_error("Yaw too small; limit is -175, value is " + std::to_string(yawAdjusted));
  }

  // When we're in the Northern hemisphere, the home yaw is +90 degrees and negative moves
  // to the right.  When in the Southern, the home is -90 degrees and negative moves to the left.
  // Determine which hemisphere we want to be in by checking the sign of the yaw.
  std::string hemisphere;
  if (yawAdjusted >= 0) {
    hemisphere = "0"; // Southern
    yawAdjusted = -90 + yawAdjusted;
    // Pitch is backwards in this hemisphere, so we invert it here.
    pitchAdjusted = -pitchAdjusted;
  }
  else {
    hemisphere = "1"; // Northern
    yawAdjusted = 90 + yawAdjusted;
  }

  // The pitch range is 0-360, so negative values have 360 added to them
  if (pitchAdjusted < 0) {
    pitchAdjusted += 360;
  }

  // If our adjusted yaw (declination) and the previous are both at or above 80 degrees in magnitude, first
  // command a move to the same sign but at 70 degrees magnitude to avoid instability in the
  // iOptron mount's shortest-path algorithm that can make it take the long way around,
  // crashing the ball into the tripod.
  if (fabs(yawAdjusted) >= 80 && fabs(m_lastYawDegrees) >= 80) {
    double clampedYaw = yawAdjusted * (70 / fabs(yawAdjusted));
    MoveAbsoluteRaw(clampedYaw, pitchAdjusted, hemisphere);
  }

  // Perform the move to the final state.
  MoveAbsoluteRaw(yawAdjusted, pitchAdjusted, hemisphere);

  // Remember our last commanded move.
  m_lastYawDegrees = yawAdjusted;
  m_lastPitchDegrees = pitchAdjusted;
}

void Gimbal_iOptron::MoveAbsoluteRaw(double yawAdjusted, double pitchAdjusted, std::string hemisphere)
{
  if (!m_impl->sendCommandCheckReponseAndFail(":SHE" + hemisphere + "#", "1")) {
    throw std::runtime_error("Unable to send hemisphere command");
  }

  // Set the declination to be slewed to.  This value is in units of 0.01 arc-seconds, so we
  // convert from degrees to arc-seconds and then multiply by 100.  We then put this into
  // an 8-character (sign then digits padded with 0 to the left to 8 digits long) string.
  int yawArcSeconds = static_cast<size_t>(yawAdjusted * 3600.0);
  int yawTicks = yawArcSeconds * 100;
  std::string yawString = std::to_string(std::abs(yawTicks));
  while (yawString.length() < 8) {
    yawString = "0" + yawString;
  }
  std::string sign = "+";
  if (yawAdjusted < 0) {
    sign = "-";
  }
  std::string cmd = ":Sd" + sign + yawString + "#";
  if (!m_impl->sendCommandCheckReponseAndFail(cmd, "1")) {
    throw std::runtime_error("Unable to send declination command");
  }

  // Set the right ascension to be slewed to.  This value is in units of 0.01 arc-seconds, so we
  // convert from degrees to arc-seconds and then multiply by 100.  We then put this into
  // an 9-character (padded with 0 to the left) string.
  int pitchArcSeconds = static_cast<size_t>((pitchAdjusted) * 3600.0);
  int pitchTicks = pitchArcSeconds * 100;
  std::string pitchString = std::to_string(std::abs(pitchTicks));
  while (pitchString.length() < 9) {
    pitchString = "0" + pitchString;
  }
  cmd = ":SRA" + pitchString + "#";
  if (!m_impl->sendCommandCheckReponseAndFail(cmd, "1")) {
    throw std::runtime_error("Unable to send right ascension command");
  }

  // Reset time to align the Earth with celestial coordinates, so RA=0.
  std::string ret = m_impl->resetTime();
  if (ret.size()) {
    throw std::runtime_error("Unable to reset time: " + ret);
  }

  // Slew to the requested location, in the normal (counterweight down) configuration.
  if (!m_impl->sendCommandCheckReponseAndFail(":MS1#", "1")) {
    throw std::runtime_error("Unable to send slew command");
  }

  // Wait for the gimbal to finish moving.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  constexpr double maxTiltDegrees = 80;
  ret = m_impl->waitForSlewStop(maxTiltDegrees, std::chrono::milliseconds(60000));
  if (ret != "") {
    if (ret.rfind("Gimbal is slewing to an unsafe position", 0) == 0) {
      // If this caused a slew to an unsafe position, find out which side of the pier we are on and issue
      // a direct-motion command to move RA towards making the counterweight down.

      std::string r2;
      if (!m_impl->sendCommand(":GEP#")) {
        throw std::runtime_error("Could not send pose request when slewed into an unsafe position");
      }
      std::string example = "sTTTTTTTTTTTTTTTTnnnnnn#";
      struct timeval tv = { 0, 100000 };
      std::string resp = m_impl->getResponse(&tv, example.size());
      if (resp.size() != example.size()) {
        // The unit sometimes sends a response that is shorter than expected or no response
        // while it is homing or slewing; ignore this.
        throw std::runtime_error("Could not get response to pose request when slewed into an unsafe position");
      }
      bool pierWest = (resp[18] != '0');
      std::string cmd = pierWest ? ":Me99999#" : ":Mw99999#";
      // This motion command gets no response
      if (!m_impl->sendCommand(cmd)) {
        throw std::runtime_error("When slewed into an unsafe position, could not send command " + cmd);
      }

      // Keep moving until we get into the safe zone or we time out.
      auto start = std::chrono::steady_clock::now();
      do {
        // Get the right ascension and declination. See if we've escaped the danger zone.
        if (!m_impl->sendCommand(":GEP#")) {
          throw std::runtime_error("When slewed into an unsafe position, could not send read position command");
        }
        example = "sTTTTTTTTTTTTTTTTTTT#";
        tv = { 0, 100000 };
        resp = m_impl->getResponse(&tv, example.size());
        if (resp.size() != example.size()) {
          // The unit sometimes sends a response that is shorter than expected or no response
          // while it is homing or slewing; ignore this.
          continue;
        }

        // The valid range in degrees is within maxRADegrees of 0, 180, or 360.
        int RA = std::stoi(resp.substr(9, 9));
        double RAdeg = RA / (3600.0 * 100);
        if ( (std::abs(RAdeg - 0) < maxTiltDegrees) || (std::abs(RAdeg - 180) < maxTiltDegrees) ||
             (std::abs(RAdeg - 360) < maxTiltDegrees)) {
          // Send a stop command
          if (!m_impl->sendCommandCheckReponseAndFail(":Q#", "1")) {
            if (!m_impl->sendCommandCheckReponseAndFail(":Q#", "1")) {
              throw std::runtime_error("When slewed into an unsafe position, could not send stop command after bypassing limits -- expect crash!");
            }
          }
          break;
        }

        auto now = std::chrono::steady_clock::now();
        if ((now - start) >= std::chrono::seconds(20)) {
          // Send a stop command
          if (!m_impl->sendCommandCheckReponseAndFail(":Q#", "1")) {
            if (!m_impl->sendCommandCheckReponseAndFail(":Q#", "1")) {
              throw std::runtime_error("When slewed into an unsafe position, could not send stop command after bypassing limits -- expect crash!");
            }
          }
          throw std::runtime_error("When slewed into an unsafe position, timed out trying to escape");
        }
      } while (true);

      // Make a move to (0,0) and then to the original request location
      try {
        MoveAbsoluteRaw(0, 0, hemisphere);
        MoveAbsoluteRaw(yawAdjusted, pitchAdjusted, hemisphere);
      } catch (const std::exception& e) {
        throw std::runtime_error(std::string("Error during adjustment move after slew to unsafe location: ") + e.what());
      }

    } else {
      throw std::runtime_error("Error waiting for gimbal to finish moving: " + ret);
    }
  }

  // Disable tracking, which the slew command re-enables every time.
  if (!m_impl->sendCommandCheckReponseAndFail(":ST0#", "1")) {
    throw std::runtime_error("Unable to send stop-tracking command");
  }
}

Gimbal_iOptron_CEM40::Gimbal_iOptron_CEM40(std::string comPortName)
  : Gimbal_iOptron(comPortName, "0040")
{
}

Gimbal_iOptron_CEM70::Gimbal_iOptron_CEM70(std::string comPortName)
  : Gimbal_iOptron(comPortName, "0070")
{
}
