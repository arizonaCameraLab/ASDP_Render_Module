/*
 * Copyright (C) 2026: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include <iostream>
#include <chrono>
#include <string>
#include <vector>
#include <array>
#include <cmath>
#include <iomanip>
#include <ASDP_Core_API.h>

static void usage(const char* progName)
{
  std::cerr << "Usage: " << progName << " <NIC name> <port>" << std::endl;
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
  std::vector<uint8_t> message(1 + 15 * sizeof(float) + 2 * sizeof(uint32_t) + 3, 0);
  message[0] = 7;
  for (size_t i = 0; i < 3; i++) {
    encodeBigEndian(reinterpret_cast<uint32_t const*>(&RPA[i]), &message[1 + (3 + i) * sizeof(float)]);
  }
  encodeBigEndian(&timeMilli, &message[1 + 9 * sizeof(float) + 2]);
  message[1 + 12 * sizeof(float) + 2 * sizeof(uint32_t) + 2] = valid ? 1 : 0;
  return message;
}

int main(int argc, char** argv)
{
  std::string xSightNICName;
  if (argc != 3) {
    usage(argv[0]);
    return 1;
  }
  xSightNICName = argv[1];
  uint16_t xSightPort = static_cast<uint16_t>(std::stoi(argv[2]));

  const std::string xSightMulticastAddress = "224.0.0.50";
  std::cout << "Receiving from " << xSightMulticastAddress << " on port " << xSightPort << " on NIC " << xSightNICName << std::endl;
  {
    // Open a ReceiverUDP to read from the XSight multicast address on the specified NIC.
    asdp::ReceiverUDP receiver(xSightNICName, xSightPort, 9000, true, xSightMulticastAddress);
    asdp::Status status = receiver.GetConstructorStatus();
    if (status != asdp::Status::OKAY) {
      std::cerr << "Failed to construct the ReceiverUDP object: " << asdp::ErrorMessage(status) << std::endl;
      return 2;
    }

    auto startTime = std::chrono::high_resolution_clock::now();
    size_t numSteps = 0;

    uint8_t buffer[9000];
    while (true) {
      // Receive a message.
      size_t bufferSize = sizeof(buffer);
      status = receiver.ReceiveBuffer(buffer, bufferSize);
      if (status != asdp::Status::OKAY) {
        std::cerr << "Failed to receive a message: " << asdp::ErrorMessage(status) << std::endl;
        return 3;
      }
      std::cout << "Received message of size " << bufferSize << " bytes:";

      // Print the hexadecimal representation of the message, sending at most 32 bytes per line.
      for (size_t i = 0; i < bufferSize; i++) {
        if (i % 32 == 0) {
          std::cout << std::endl;
        }
        std::cout << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
          << static_cast<int>(buffer[i]) << " ";
      }
      std::cout << std::dec << std::endl;
    }
  }

  // Done
  return 0;
}
