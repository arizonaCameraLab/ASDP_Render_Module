/*
 * Copyright (C) 2025: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <filesystem>
#include <stdio.h>
#include <CameraRenderInfo.h>
#include <Calibration_Helpers.h>
#include <ASDP_StreamPacketSortedQueue.h>
#include <Gimbal.h>
#include <ASDP_Core_API.h>

using namespace asdp;

static void usage(const char* progName)
{
  std::cerr << "Usage: " << progName << " [options] NIC SERIAL POSES.csv OUTDIR" << std::endl;
  std::cerr << "       NIC: The network interface card (NIC) IP address for the camera (e.g., 10.10.10.32)." << std::endl;
  std::cerr << "       SERIAL: The serial number of the camera." << std::endl;
  std::cerr << "       POSES.csv: Poses to take images at." << std::endl;
  std::cerr << "       OUTDIR: The output directory for the calibration data (probably named cameras_images)." << std::endl;
  std::cerr << "       Options:" << std::endl;
  std::cerr << "         --home: Move to home position." << std::endl;
  std::cerr << "         --gimbalConfig <string>: The gimbal configuration file name (default gimbal.json)." << std::endl;
  std::cerr << "         --help: Print this information and quit." << std::endl;
}

static std::shared_ptr<Message> WaitForMessageType(std::shared_ptr<Receiver> receiver, MessageID type, float seconds)
{
  std::shared_ptr<Message> empty;   ///< We return this on failure.
  std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
  do {
    std::shared_ptr<StreamPacket> response;
    size_t offset = 0;
    Status status = receiver->ReceiveStreamPacket(0, response, offset);
    if ((status != OKAY) && (status != TIMEOUT)) {
      return empty;
    }
    if (response != nullptr) {
      std::shared_ptr<Message> message;
      status = response->GetNextMessage(message);
      if (status != OKAY) {
        return empty;
      }
      while (message != nullptr) {
        MessageID messageType;
        status = message->GetType(messageType);
        if (status != OKAY) {
          return empty;
        }
        if (messageType == type) {
          // Worked!
          return message;
        }
        status = response->GetNextMessage(message);
        if (status != OKAY) {
          return empty;
        }
      }
    }
  } while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() <= seconds);

  return empty;
}

static bool isBigEndian() {
  union {
    uint32_t i;
    char c[4];
  } testUnion = { 0x01020304 };

  return testUnion.c[0] == 1;
}

static void fixEndian(uint8_t* data, size_t size) {
  if (!isBigEndian()) {
    uint16_t* data16 = reinterpret_cast<uint16_t*>(data);
    for (size_t i = 0; i < size / 2; i++) {
      uint16_t& value = data16[i];
      value = (value >> 8) | (value << 8);
    }
  }
}


int main(int argc, char** argv)
{
  std::string gimbalConfigFile = "gimbal.json";
  std::string nic;
  unsigned int serial = 0;
  std::string posesFile;
  std::string outDir;
  bool home = false;

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
    else if (argv[i][0] == '-') {
      usage(argv[0]);
      return 1;
    }
    else switch (++realParams) {
      case 1:
        nic = argv[i];
        break;
      case 2:
        serial = std::atoi(argv[i]);
        break;
      case 3:
        posesFile = argv[i];
        break;
      case 4:
        outDir = argv[i];
        break;
      default:
        usage(argv[0]);
        return 1;
    }
  }
  if (realParams != 4) {
    usage(argv[0]);
    return 1;
  }

  // Put the code into a block so that objects will be go out of scope before we exit.
  {
    //=============================================================================================
    // Read the gimbal configuration file.
    asdp::render::calibration::GimbalInfo gimbalInfo;
    try {
      gimbalInfo = asdp::render::calibration::GetGimbalInfo(gimbalConfigFile);
    }
    catch (const std::exception& e) {
      std::cerr << "Error: Unable to read gimbal configuration file: " << e.what() << std::endl;
      return 10;
    }

    //=============================================================================================
    // Read the poses file and verify that the poses are all within bounds

    std::vector<asdp::render::calibration::PoseInfo> poseInfos;
    try {
      poseInfos = asdp::render::calibration::GetPoseInfos(posesFile);
    }
    catch (const std::exception& e) {
      std::cerr << "Error: Unable to read pose file: " << e.what() << std::endl;
      return 11;
    }
    if (poseInfos.empty()) {
      std::cerr << "Error: No poses found in the file." << std::endl;
      return 12;
    }
    std::cout << "Read " << poseInfos.size() << " poses from " << posesFile << std::endl;

    for (const auto& pose : poseInfos) {
      if (pose.zRotationDegrees < gimbalInfo.minYawDegrees ||
          pose.zRotationDegrees > gimbalInfo.maxYawDegrees ||
          pose.xRotationDegrees < gimbalInfo.minPitchDegrees ||
          pose.xRotationDegrees > gimbalInfo.maxPitchDegrees) {
        std::cerr << "Error: Pose out of bounds: " << pose.zRotationDegrees << ", " << pose.xRotationDegrees << std::endl;
        return 13;
      }
    }

    //=============================================================================================
    // Create the output directory, not failing if it already exists.

    if (!std::filesystem::exists(outDir)) {
      if (!std::filesystem::create_directories(outDir)) {
        std::cerr << "Error: Unable to create output directory: " << outDir << std::endl;
        return 14;
      }
    }

    //=============================================================================================
    // Connect to the specified camera on the specified NIC and configure it.

    std::shared_ptr<CoreClient> client = std::make_shared<CoreClient>(nic);
    if (client->GetConstructorStatus() != OKAY) {
      std::cerr << "Failed to open client: " << ErrorMessage(client->GetConstructorStatus()) << std::endl;
      return 20;
    }

    // Wait for up to two seconds to allow servers to send Discovery messages.
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    std::vector<std::string> servers;
    Status threadStatus;
    Status status;
    do {
      status = client->GetDiscoveryThreadStatus(threadStatus);
      if (status != OKAY) {
        std::cerr << "Failed to get discovery thread status: " << ErrorMessage(status) << std::endl;
        return 21;
      }
      if (threadStatus != OKAY) {
        std::cerr << "Discovery thread status: " << ErrorMessage(threadStatus) << std::endl;
        return 22;
      }
      status = client->IdentifiedServers(servers);
      if (status != OKAY) {
        std::cerr << "Failed to get identified servers: " << ErrorMessage(status) << std::endl;
        return 23;
      }
      if (!servers.empty()) { break; }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() <= 2.0);
    if (servers.empty()) {
      std::cerr << "No servers found; be sure to run the server first." << std::endl;
      return 24;
    }

    // Connect to each server until we find one that matches the serial number we're looking for.
    uint32_t serialNumber = 0;
    for (const std::string& server : servers) {
      uint16_t major, minor, patch;
      status = client->ConnectToServer(server, major, minor, patch);
      if (status != OKAY) {
        std::cerr << "Failed to connect to server: " << ErrorMessage(status) << std::endl;
        return 25;
      }
      status = client->GetServerSerialNumber(serialNumber);
      if (status != OKAY) {
        std::cerr << "Failed to get server serial number: " << ErrorMessage(status) << std::endl;
        return 26;
      }
      if (serialNumber == serial) {
        break; // Found the right server.
      }
    }
    if (serialNumber != serial) {
      std::cerr << "Error: Unable to find server with serial number " << serial << std::endl;
      return 27;
    }
    std::cout << "Connected to server " << serialNumber << " on " << nic << std::endl;

    // Get the main stream receiver
    std::shared_ptr<Receiver> receiver;
    status = client->GetMainStreamReceiver(receiver);
    if (status != OKAY) {
      std::cerr << "Failed to get main stream receiver: " << ErrorMessage(status) << std::endl;
      return 28;
    }

    // Ensure that we get a state message from the server within a reasonable time.
    // Record information about the cameras that were found.
    std::vector<CameraInfo> cameras;
    {
      std::shared_ptr<Message> msg = WaitForMessageType(receiver, STATE, 5.0);
      if (msg == nullptr) {
        std::cerr << "Did not get state message." << std::endl;
        return 29;
      }
      MessageState state(*msg);
      if (state.GetConstructorStatus() != OKAY) {
        std::cerr << "Failed to construct state message: " << ErrorMessage(state.GetConstructorStatus()) << std::endl;
        return 30;
      }
      status = state.GetCameras(cameras);
      std::cout << "Found " << cameras.size() << " cameras" << std::endl;
      if (cameras.size() == 0) {
        return 31;
      }
    }

    // Request triggering on the cameras at their maximum rates from their associated ID.
    for (size_t i = 0; i < cameras.size(); ++i) {
      uint32_t camID = i + 1;
      CameraInfo& camera = cameras[i];

      TriggerInfo ti;
      ti.ID = camera.trigger;
      ti.mode = 1;
      ti.period = camera.minTriggerPeriod;
      ti.offset = 0;
      ti.trackingFactor = 0.005;
      status = client->SendCommandPacket(CommandPacketConfigureTrigger(ti));
      if (status != OKAY) {
        std::cerr << "Failed to configure trigger: " << ErrorMessage(status) << std::endl;
        return 32;
      }
      std::cout << std::setprecision(10) << "  Configured trigger for camera " << camID << " with period " << ti.period << " seconds" << std::endl;
    }

    //=============================================================================================
    // Gimbal initialization

    // Open the gimbal on the specified COM port, setting its speed and acceleration if provided.
    // Then verify the gimbal is connected and operational.
    std::shared_ptr<Gimbal> gimbal;
    try {
      gimbal = ConstructGimbal(gimbalInfo);
    }
    catch (const std::exception& e) {
      std::cerr << "Error: Unable to construct gimbal: " << e.what() << std::endl;
      return 100;
    }
    if (!gimbal->Status()) {
      std::cerr << "Gimbal not connected or not operational." << std::endl;
      return 101;
    }

    // If we've been asked to home the gimbal, do so.
    if (home) {
      if (!gimbal->Home()) {
        std::cerr << "Failed to home the gimbal." << std::endl;
        return 102;
      }
    }

    //=============================================================================================
    // For each new frame index, move the pose to the specified location. For each pose, take the
    // specified number of images using the specified camera and write them to the output directory.

    int lastFrameIndex = -1;
    size_t width = cameras[0].width;
    size_t height = cameras[0].height;
    size_t frameSize = width * height * sizeof(uint16_t);
    for (auto const& pose : poseInfos) {

      //===================================================================================
      // Move if we have a new index (the first one in the file is 1).
      if (pose.frameIndex != lastFrameIndex) {
        // Move the gimbal to the new pose, waiting until it arrives.
        if (!gimbal->MoveAbsolute(pose.zRotationDegrees, pose.xRotationDegrees)) {
          std::cerr << "Failed to move gimbal to pose: " << pose.zRotationDegrees << ", " << pose.xRotationDegrees << std::endl;
          return 200;
        }
        lastFrameIndex = pose.frameIndex;
      }

      //===================================================================================
      // Capture the specified number of frames.

      // Create buffers to hold the data.
      std::vector < std::vector<uint8_t> > imageBuffers;
      for (size_t i = 0; i < pose.numFrames; ++i) {
        imageBuffers.push_back(std::vector<uint8_t>(frameSize));
      }

      // Construct a UDP receiver for a stream from the camera.
      ReceiverUDP receiverUDP(nic);
      if (receiverUDP.GetConstructorStatus() != OKAY) {
        std::cerr << "Error constructing ReceiverUDP: " << ErrorMessage(receiverUDP.GetConstructorStatus()) << std::endl;
        return 300;
      }
      uint16_t port;
      asdp::Status status = receiverUDP.GetPort(port);
      if (status != asdp::OKAY) {
        std::cerr << "Error getting port from ReceiverUDP: " << ErrorMessage(status) << std::endl;
        return 301;
      }

      // Request the camera to stream full-frame images, capturing every frame.
      StreamEndpoint endpoint(nic, port);
      SubregionDescription region;
      region.cameraID = pose.cameraID;
      region.skipFrames = 0;
      region.startTimeSeconds = 0;
      region.startTimeMicroseconds = 0;
      region.left = 0;
      region.top = 0;
      region.right = width - 1;    ///< @todo This assumes all cameras are the same size.
      region.bottom = height - 1;  ///< @todo This assumes all cameras are the same size.
      status = client->SendCommandPacket(CommandPacketStreamSubregion(endpoint, region));
      if (status != OKAY) {
        std::cerr << "Failed to stream images: " << ErrorMessage(status) << std::endl;
        return 302;
      }

      // Use a sorting queue to ensure that we process the messages in order even if the UDP packets
      // arrive out of order.
      StreamPacketSortedQueue sortedQueue(50);

      // Receive packets and stuff them into the buffers.
      size_t receivedFrames = 0;
      bool gotFrameBegin = false;
      do {
        // Service the main receiver, gobbling up any packets.
        std::shared_ptr<asdp::StreamPacket> mainPacket;
        size_t offset = 0;
        status = receiver->ReceiveStreamPacket(0, mainPacket, offset);
        if (status != asdp::OKAY && status != asdp::TIMEOUT) {
          std::cerr << "Error receiving main StreamPacket: " << ErrorMessage(status) << std::endl;
          return 303;
        }

        // Get the next UDP packet.
        // Add to the sorted queue and then handle any messages that are ready to be processed.
        std::shared_ptr<asdp::StreamPacket> packet;
        offset = 0;
        status = receiverUDP.ReceiveStreamPacket(0.0, packet, offset);
        if (status == asdp::TIMEOUT) {
          continue;
        }
        if (status != asdp::OKAY) {
          std::cerr << "Error receiving StreamPacket: " << ErrorMessage(status) << std::endl;
          return 304;
        }
        std::list< std::shared_ptr<StreamPacket> > readyPackets = sortedQueue.AddPacket(packet);
        if (readyPackets.size() > 1) {
          std::cerr << "Warning: More than one packet ready to process (re-ordered or missing packet)." << std::endl;
        }
        while (!readyPackets.empty()) {
          std::shared_ptr<asdp::StreamPacket> receiveStreamPacket = readyPackets.front();
          readyPackets.pop_front();

          // Get and handle all messages from the packet.
          std::shared_ptr<asdp::Message> message;
          status = receiveStreamPacket->GetNextMessage(message);
          if (status != asdp::OKAY) {
            std::cerr << "Error getting message from packet: " << ErrorMessage(status) << std::endl;
            return 305;
          }
          while (message != nullptr) {
            asdp::MessageID rID;
            status = message->GetType(rID);
            if (status != asdp::OKAY) {
              std::cerr << "Error getting type from message: " << ErrorMessage(status) << std::endl;
              return 306;
            }
            switch (rID) {
            case asdp::FRAME_BEGIN:
            {
              gotFrameBegin = true;
            }
            break;
            case asdp::FRAME_DATA:
            {
              // Don't do anything if we haven't gotten a begin-frame yet.
              if (!gotFrameBegin) { break; }
              // Find out how many pixels are in the frame and sum their values.
              asdp::MessageFrameData frameData(*message);
              if (frameData.GetConstructorStatus() != asdp::OKAY) {
                std::cerr << "Error constructing FrameData message: " << ErrorMessage(frameData.GetConstructorStatus()) << std::endl;
                return 307;
              }
              uint16_t stride = width;
              uint16_t left, right, top, bottom;
              status = frameData.GetLeft(left);
              if (status != asdp::OKAY) {
                std::cerr << "Error getting left from FrameData message: " << ErrorMessage(status) << std::endl;
                return 308;
              }
              status = frameData.GetRight(right);
              if (status != asdp::OKAY) {
                std::cerr << "Error getting right from FrameData message: " << ErrorMessage(status) << std::endl;
                return 309;
              }
              status = frameData.GetTop(top);
              if (status != asdp::OKAY) {
                std::cerr << "Error getting top from FrameData message: " << ErrorMessage(status) << std::endl;
                return 310;
              }
              status = frameData.GetBottom(bottom);
              if (status != asdp::OKAY) {
                std::cerr << "Error getting bottom from FrameData message: " << ErrorMessage(status) << std::endl;
                return 311;
              }
              uint8_t* rawData;
              status = frameData.GetDataPointer(rawData);
              if (status != asdp::OKAY) {
                std::cerr << "Error getting data pointer from FrameData message: " << ErrorMessage(status) << std::endl;
                return 312;
              }
              // NOTE: This makes use of the fact that we're asking for the full frame, and that the server sends
              // full lines at once when this is the case.  Otherwise, we'd need to copy the data line by line and
              // adjust for the full-image stride when doing offsets.
              size_t size = (right - left + 1) * (bottom - top + 1) * sizeof(uint16_t);
              memcpy(imageBuffers[receivedFrames].data() + (top * stride + left) * sizeof(uint16_t), rawData, size);
            }
            break;
            case asdp::FRAME_END:
            {
              // Don't do anything if we haven't seen a begin-frame message yet.
              if (!gotFrameBegin) { break; }
              receivedFrames++;
            }
            break;
            default:
              break;
            }

            status = receiveStreamPacket->GetNextMessage(message);
            if (status != asdp::OKAY) {
              std::cerr << "Error getting first message from packet: " << ErrorMessage(status) << std::endl;
              return 313;
            }
          }
        }

      } while (receivedFrames < pose.numFrames);

      // Turn off streaming.
      status = client->SendCommandPacket(CommandPacketCancelSubregion(pose.cameraID, endpoint));
      if (status != OKAY) {
        std::cerr << "Failed to cancel stream images: " << ErrorMessage(status) << std::endl;
        return 399;
      }

      //===================================================================================
      // Write the frames to the output directory.
      for (int f = 0; f < imageBuffers.size(); f++) {
        std::string fileName = outDir + "/" + std::to_string(pose.frameIndex) + "_" +
          std::to_string(pose.cameraID) + "_" + std::to_string(f+1) + ".pgm";
        FILE* of = fopen(fileName.c_str(), "wb");
        if (of == NULL) {
          std::cerr << "Error opening image file " << fileName << std::endl;
          return 400;
        }
        fprintf(of, "P5\n%d %d\n%d\n", 1280, 1024, 65535);
        fixEndian(imageBuffers[f].data(), imageBuffers[f].size());
        fwrite(imageBuffers[f].data(), sizeof(uint8_t), imageBuffers[f].size(), of);
        fclose(of);
      }
      std::cout << "Wrote " << imageBuffers.size() << " images for frame " << pose.frameIndex
        << " camera " << pose.cameraID << " (frame index " << lastFrameIndex << " of "
        << poseInfos.back().frameIndex << ")" << std::endl;
    }
  }

  // Done
  return 0;
}
