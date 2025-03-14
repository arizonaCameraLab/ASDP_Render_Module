/*
 * Copyright (C) 2025: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include <iostream>
#include <chrono>
#include <string>
#include <vector>
#include <array>
#include <cmath>
#include <ASDP_Core_API.h>

static void usage(const char* progName)
{
  std::cerr << "Usage: " << progName << " <NIC name>" << std::endl;
}

static void encodeBigEndian(uint32_t const *value, uint8_t* buffer)
{
  buffer[0] = static_cast<uint8_t>((*value >> 24) & 0xFF);
  buffer[1] = static_cast<uint8_t>((*value >> 16) & 0xFF);
  buffer[2] = static_cast<uint8_t>((*value >> 8) & 0xFF);
  buffer[3] = static_cast<uint8_t>(*value & 0xFF);
}

static std::vector<uint8_t> encodeMessage(std::array<float, 3> const &RPA, uint32_t timeMilli, bool valid)
{
  std::vector<uint8_t> message(15 * sizeof(float) + 2 * sizeof(uint32_t) + 3, 0);
  for (size_t i = 0; i < 3; i++) {
    encodeBigEndian(reinterpret_cast<uint32_t const*>(&RPA[i]), &message[(3 + i) * sizeof(float)]);
  }
  encodeBigEndian(&timeMilli, &message[9 * sizeof(float) + 2]);
  message[12 * sizeof(float) + 2 * sizeof(uint32_t) + 2] = valid ? 1 : 0;
  return message;
}

int main(int argc, char** argv)
{
  std::string xSightNICName;
  if (argc != 2) {
    usage(argv[0]);
    return 1;
  }
  xSightNICName = argv[1];

  const uint16_t xSightPort = 5535;
  const std::string xSightMulticastAddress = "224.0.0.50";
  std::cout << "Sending to " << xSightMulticastAddress << " on port " << xSightPort << " on NIC " << xSightNICName << std::endl;
  {
    // Open a SenderUDP to write to the XSight multicast address on the specified NIC.
    asdp::SenderUDP sender(xSightNICName, xSightPort, false, xSightNICName, xSightMulticastAddress);
    asdp::Status status = sender.GetConstructorStatus();
    if (status != asdp::Status::OKAY) {
      std::cerr << "Failed to construct the SenderUDP object: " << asdp::ErrorMessage(status) << std::endl;
      return 2;
    }

    auto startTime = std::chrono::high_resolution_clock::now();
    size_t numSteps = 0;
    while (true) {
      // Wait until we pass another 20 milliseconds since start time.  Do this using a busy wait so that
      // we are very accurate in our timing.
      auto expectedTime = startTime + std::chrono::milliseconds(20 * ++numSteps);
      while (std::chrono::high_resolution_clock::now() < expectedTime) {
        // Do nothing.
      }

      // Compute the number of since we started running and then store it into an integer number
      // and fractions.
      auto currentTime = std::chrono::high_resolution_clock::now();
      auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - startTime);
      uint32_t seconds = static_cast<uint32_t>(elapsedTime.count() / 1000);
      float fraction = static_cast<uint32_t>(elapsedTime.count() % 1000) / 1000.0f;

      // Use the time modulo 3 to determine which angle to adjust and the time multiplied by 2 pi
      // to determine the phase and then multiply by 10 to shift by 10 degrees maximum.
      int axis = seconds % 3;
      float fracOfCircle = fraction * 2.0f * 3.14159f;
      float angle = 10 * sin(fracOfCircle);

      // Create the message to send.
      std::array<float, 3> RPA = { 0.0f, 0.0f, 0.0f };
      RPA[axis] = angle;
      uint32_t timeMilli = seconds * 1000 + static_cast<uint32_t>(fraction * 1000);
      bool valid = true;
      std::vector<uint8_t> message = encodeMessage(RPA, timeMilli, valid);

      // Send the message.
      status = sender.Send(message.data(), message.size());
      if (status != asdp::Status::OKAY) {
        std::cerr << "Failed to send the message: " << asdp::ErrorMessage(status) << std::endl;
        return 3;
      }
    }

  }

  // Done
  return 0;
}
