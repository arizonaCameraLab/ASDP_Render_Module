/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

// This is a client that connects to the first server it encounters and runs a Render Module.

/**
 * @file ASDP_Render_Module.cu
 * @brief Apache Strap-Down Pilotage Render Module.
 *
* @author ReliaSolve.
* @date May 20th, 2024.
*/

#include <iostream>
#include <fstream>
#include <chrono>
#include <map>
#include <set>
#include <mutex>
#include <thread>
#include <string>
#include <filesystem>
#include <ASDP_Core_API.h>
#include <ASDP_SpinFreeQueue.hpp>
#include <ASDP_BufferPool.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <nlohmann/json.hpp>
#include <Composite.h>

using namespace asdp;
using json = nlohmann::json;

static std::string VERSION = "1.0.0";

/// @brief The path to the configuration file. Defined in the CMakeLists file.
std::filesystem::path dirPath = CONFIG_FILE_PATH;

/// @brief Structure to hold the data needed to send data to the GPU and run the kernel.
/// @details These will all have been constructed by the thread that is pushing them onto the queue,
/// with custom destructors as needed to free the memory when the shared_ptr is destroyed.
struct DataToSendToGPU {
  std::shared_ptr<StreamPacket> streamPacketPtr;   ///< The stream packet that was received, which includes camera ID
  std::shared_ptr<unsigned char> cpuImageBufferPtr;///< The pinned-memory buffer on the CPU that holds the image data
  std::shared_ptr<unsigned char> gpuImageBufferPtr;///< The buffer on the GPU that holds the image data
  std::shared_ptr<cudaStream_t> streamPtr;         ///< Stream to use to for the copy and kernel run
};

/// @brief Function to copy data to the GPU and store it into the appropriate texture.
/// It must create and record an event after all operations are complete.  All operations must be
/// done on the stream that is passed in and they must all be asynchronous.
/// @param width The width of the image data.
/// @param height The height of the image data.
/// @param done A flag that is set to true when the program is done.
/// @param inQueue The queue that we receive requests on.
/// @param batchSize The number of lines to send to the GPU at once.  This is tuned to trade off latency
/// for throughput, and it should be set to a value that is large enough to amortize the cost of sending
/// data to the GPU, but small enough to keep latency low.  The value of 16 is a good starting point.
static void CopyDataToGPU(uint16_t width, uint16_t height,
  std::atomic<bool>& done,
  std::shared_ptr< SpinFreeQueue< std::shared_ptr<DataToSendToGPU> > > inQueue,
  size_t batchSize)
{
  Status status;

  while (!done) {
    std::shared_ptr<DataToSendToGPU> data;
    // Time out after 10 milliseconds so that we can check the done flag frequently.
    if (inQueue->dequeue(data, std::chrono::milliseconds(10))) {

      // Parse all of the messages in the stream packet, handling each of them in turn.
      std::shared_ptr<Message> message;
      status = data->streamPacketPtr->GetNextMessage(message);
      if (OKAY != status) {
        std::cerr << "CopyDataToGPU: GetNextMessage() failed: " << ErrorMessage(status) << std::endl;
        done = true;
        return;
      }
      while (message != nullptr) {
        MessageID messageType;
        if (OKAY != message->GetType(messageType)) { return; }
        switch (messageType) {
        case FRAME_BEGIN:
          // Nothing to do for the beginning of a frame.
          break;

        case FRAME_DATA:
          // Copy the data to the pinned CPU memory buffer, and then asynchronously to the GPU buffer as
          // we get enough data for a minimum block size.  We send the data to the GPU in chunks so that
          // we amortize the per-send cost, but we send in chunks to reduce the latency and enable overlap
          // between data copying and processing (which increases throughput).
          {
            // Get the region to copy and the data pointer from the message.
            MessageFrameData frameData(*message);
            if (frameData.GetConstructorStatus() != OKAY) {
              std::cerr << "CopyDataToGPU: Failed to construct MessageFrameData: " << ErrorMessage(frameData.GetConstructorStatus()) << std::endl;
              done = true;
              return;
            }
            uint16_t left, right, top, bottom;
            status = frameData.GetLeft(left);
            if (OKAY != status) {
              std::cerr << "CopyDataToGPU: GetLeft() failed: " << ErrorMessage(status) << std::endl;
              done = true;
              return;
            }
            status = frameData.GetRight(right);
            if (OKAY != status) {
              std::cerr << "CopyDataToGPU: GetRight() failed: " << ErrorMessage(status) << std::endl;
              done = true;
              return;
            }
            status = frameData.GetTop(top);
            if (OKAY != status) {
              std::cerr << "CopyDataToGPU: GetTop() failed: " << ErrorMessage(status) << std::endl;
              done = true;
              return;
            }
            status = frameData.GetBottom(bottom);
            if (OKAY != status) {
              std::cerr << "CopyDataToGPU: GetBottom() failed: " << ErrorMessage(status) << std::endl;
              done = true;
              return;
            }
            uint8_t* dataPtr;
            status = frameData.GetDataPointer(dataPtr);
            if (OKAY != status) {
              std::cerr << "CopyDataToGPU: GetDataPointer() failed: " << ErrorMessage(status) << std::endl;
              done = true;
              return;
            }

            // Copy the image data to the pinned CPU memory buffer.
            uint16_t* cpuBuffer = reinterpret_cast<uint16_t*>(data->cpuImageBufferPtr.get());
            uint16_t* dataPtr16 = reinterpret_cast<uint16_t*>(dataPtr);
            memcpy(cpuBuffer + top * width + left, dataPtr16, (right - left + 1) * (bottom - top + 1) * sizeof(uint16_t));

            // Copy the image data to the GPU if we've completed a chunk of lines.
            // We assume that the number of lines coming in from the camera is smaller than the batch size, so that we will
            // send at most one batch.  We check every line from bottom to the top of the region and send the batch including
            // that line if it is ever the last line in the region.
            for (uint16_t line = top; line <= bottom; ++line) {
              if ((line + 1) % batchSize == 0) {
                // The offset is to the start of the region, which is batchSize-1 lines before the current line.
                size_t offset = (line + 1 - batchSize) * width * sizeof(uint16_t);
                // Copy the batch to the GPU.
                cudaError_t ret =  cudaMemcpyAsync(data->gpuImageBufferPtr.get() + offset, data->cpuImageBufferPtr.get() + offset,
                  batchSize * width * sizeof(uint16_t),
                  cudaMemcpyHostToDevice, *data->streamPtr);
                if (ret != cudaSuccess) {
                  std::cerr << "CopyDataToGPU: cudaMemcpyAsync() failed: " << cudaGetErrorString(ret) << std::endl;
                  done = true;
                  return;
                }
              }
            }
          }
          break;

        case FRAME_END:
          // Run the kernel and enqueue the result.
          {
            // Construct the end-of-frame message from the message.
            MessageFrameEnd frameEnd(*message);
            if (frameEnd.GetConstructorStatus() != OKAY) {
              std::cerr << "CopyDataToGPU: Failed to construct MessageFrameEnd: " << ErrorMessage(frameEnd.GetConstructorStatus()) << std::endl;
              done = true;
              return;
            }

            //==================================================================================================
            /// @todo Replace this block of code with your own kernel launch or other work.


            /// @todo Replace this block of code with your own kernel launch or other work.
            //==================================================================================================

            // Note: This must be done after all other operations on the stream so that it will wait for them to complete.
            // Create the completion event, storing a pointer to it in a shared pointer whose destructor will delete
            // the event when there are no more references to it.  Record the event so that the caller can wait for it.
            /*
            cudaEvent_t* eventPtr = new cudaEvent_t;
            cudaEventCreate(eventPtr);
            cudaEventRecord(*eventPtr, *(data->streamPtr));
            */
            /// @todo Store in the appropriate location
          }
          break;

        default:
          // Nothing to do for other message types.
          break;
        } // End of switch on message type.

        status = data->streamPacketPtr->GetNextMessage(message);
        if (OKAY != status) {
          done = true;
          std::cerr << "CopyDataToGPU: GetNextMessage() failed: " << ErrorMessage(status) << std::endl;
          return;
        }
      } // End of while we have messages in the stream packet.
    } // End of if we got a message from the queue.
  } // End of while we are not done.
}

static void ReceiveDataThread(ReceiverUDP& receiveSocket, size_t maxBytesPerPacket, std::atomic<bool>& done,
  std::shared_ptr<unsigned char> cpuImageBufferPtr, std::shared_ptr<unsigned char> gpuImageBufferPtr,
  std::shared_ptr<cudaStream_t> streamPtr,
  std::shared_ptr< SpinFreeQueue< std::shared_ptr<DataToSendToGPU> > > outQueue)
{
  // Generate a buffer pool to use to get pre-allocated buffers for reading the data from
  // the network.  Initially fill it with 100 buffers to give us enough to handle buffering a fraction
  // of a frame before the first packets are handled.  It will automatically expand if needed.
  BufferPool bufferPool(maxBytesPerPacket, 100);

  // Loop through and receive packets until we've been told to quit.
  size_t packetsReceived = 0;
  DataToSendToGPU data;
  while (!done) {

    // Get the next packet into a preallocated buffer, timing out quickly to ensure that we check
    // the done flag.
    std::shared_ptr< std::vector<uint8_t> > buffer = bufferPool.GetBuffer();
    size_t offset = 0;
    std::shared_ptr<StreamPacket> streamPacket;
    Status status = receiveSocket.ReceiveStreamPacket(0.1, streamPacket, offset, buffer);
    if (status == TIMEOUT) {
      continue;
    }
    if (status != OKAY) {
      std::cerr << "Error receiving data: " << ErrorMessage(status) << std::endl;
      done = true;
      break;
    }

    // Verify that the data is correct and we haven't missed any packets
    uint32_t sequenceNumber;
    status = streamPacket->GetSequenceNumber(sequenceNumber);
    if (status != OKAY) {
      std::cerr << "Error getting sequence number: " << ErrorMessage(status) << std::endl;
      done = true;
      break;
    }
    if (sequenceNumber != packetsReceived) {
      std::cerr << "Error: Bad sequence number: expected " << packetsReceived << " but got " << sequenceNumber << std::endl;
      done = true;
      break;
    }

    // Increment the number of packets received
    packetsReceived++;

    // Enqueue the packet for processing.
    data.streamPacketPtr = streamPacket;
    data.cpuImageBufferPtr = cpuImageBufferPtr;
    data.gpuImageBufferPtr = gpuImageBufferPtr;
    data.streamPtr = streamPtr;
    outQueue->enqueue(std::make_shared<DataToSendToGPU>(data));
  }
}

std::shared_ptr<Message> WaitForMessageType(std::shared_ptr<Receiver> receiver, MessageID type, float seconds)
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

void usage(std::string name)
{
  std::cerr << "Usage: " << name << " [--framestride <frameStride>] <ip_address>" << std::endl;
  std::cerr << "  --frameStride <frameStride>  Read one out of every this many frames. Set to 1 for every frame." << std::endl;
  std::cerr << "  <ip_address>  The IP address to listen for servers on." << std::endl;
}

int main(int argc, char** argv)
{
  uint32_t frameStride = 1;     ///< Read one out of every this many frames. Set to 1 for every frame.
  std::string ip_address;       ///< The IP address to listen on.
  std::set<uint32_t> cameraIDs; ///< The camera IDs to render.
  size_t realParams = 0;

  // Parse the command line arguments, with the first non-flag argument being the
  // name of the IP address to listen on.
  for (int i = 1; i < argc; ++i) {
    if (std::string("--frameStride") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 2;
      }
      frameStride = std::stoi(argv[i]);
    } else if (argv[i][0] == '-') {
      std::cerr << "Unknown flag: " << argv[i] << std::endl;
      return 1;
    } else switch (realParams++) {
    case 0:
      ip_address = argv[i];
      break;
    default:
      usage(argv[0]);
      return 2;
    }
  }
  if (realParams != 1) {
    usage(argv[0]);
    return 2;
  }

  // Run inside a block so that the destructors will be called for all objects before we exit.
  {
    std::cout << "ASDP Render Module version " << VERSION << std::endl;

    // Open a client, specifying the IP address to listen on.
    CoreClient client(ip_address);
    if (client.GetConstructorStatus() != OKAY) {
      std::cerr << "Failed to open client: " << ErrorMessage(client.GetConstructorStatus()) << std::endl;
      return 3;
    }
    std::cout << "Listening for servers on " << ip_address << std::endl;

    // Wait for up to two seconds to allow servers to send Discovery messages.
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    std::vector<std::string> servers;
    Status threadStatus;
    Status status;
    do {
      status = client.GetDiscoveryThreadStatus(threadStatus);
      if (status != OKAY) {
        std::cerr << "Failed to get discovery thread status: " << ErrorMessage(status) << std::endl;
        return 4;
      }
      if (threadStatus != OKAY) {
        std::cerr << "Discovery thread status: " << ErrorMessage(threadStatus) << std::endl;
        return 5;
      }
      status = client.IdentifiedServers(servers);
      if (status != OKAY) {
        std::cerr << "Failed to get identified servers: " << ErrorMessage(status) << std::endl;
        return 6;
      }
      if (!servers.empty()) { break; }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() <= 2.0);
    if (servers.empty()) {
      std::cerr << "No servers found; be sure to run the server first." << std::endl;
      return 7;
    }
    std::cout << "Servers found: " << servers.size() << std::endl;
    for (const std::string& server : servers) {
      std::cout << "  " << server << std::endl;
    }

    // Connect to the first server found.
    std::cout << "Connecting to " << servers[0] << std::endl;
    uint16_t major, minor, patch;
    status = client.ConnectToServer(servers[0], major, minor, patch);
    if (status != OKAY) {
      std::cerr << "Failed to connect to server: " << ErrorMessage(status) << std::endl;
      return 8;
    }
    uint32_t serialNumber;
    status = client.GetServerSerialNumber(serialNumber);
    if (status != OKAY) {
      std::cerr << "Failed to get server serial number: " << ErrorMessage(status) << std::endl;
      return 9;
    }
    std::cout << "  Connected to server version " << major << "." << minor << "." << patch
      << " with serial number " << serialNumber << std::endl;

    // Get the main stream receiver
    std::shared_ptr<Receiver> receiver;
    status = client.GetMainStreamReceiver(receiver);
    if (status != OKAY) {
      std::cerr << "Failed to get main stream receiver: " << ErrorMessage(status) << std::endl;
      return 10;
    }

    // Ensure that we get a state message from the server within a reasonable time.
    // Report information about the cameras that were found.
    std::shared_ptr<Message> msg = WaitForMessageType(receiver, STATE, 5.0);
    if (msg == nullptr) {
      std::cerr << "Did not get state message." << std::endl;
      return 11;
    }
    MessageState state(*msg);
    if (state.GetConstructorStatus() != OKAY) {
      std::cerr << "Failed to construct state message: " << ErrorMessage(state.GetConstructorStatus()) << std::endl;
      return 12;
    }
    std::vector<CameraInfo> cameras;
    status = state.GetCameras(cameras);
    std::cout << "Found " << cameras.size() << " cameras" << std::endl;
    if (cameras.size() == 0) {
      return 13;
    }

    // If we have an empty set of camera IDs, then we want to analyze all cameras.
    /// @todo Replace with reading the configuration file to find out the cameras and their mappings
    if (cameraIDs.empty()) {
      for (uint32_t ID = 1; ID <= cameras.size(); ID++) {
        cameraIDs.insert(ID);
      }
    }

    // Read the configuration file asociated with the serial number for the server. Verify that
    // it has a matching serial number and number of cameras.
    std::filesystem::path configPath = dirPath / (std::to_string(serialNumber) + ".json");
    if (!std::filesystem::exists(configPath)) {
      std::cerr << "Configuration file not found: " << configPath << std::endl;
      return 14;
    }
    std::ifstream configFile(configPath);
    json config = json::parse(configFile);
    std::string configSerial = config["serialNumber"];
    if (serialNumber != std::stoi(configSerial)) {
      std::cerr << "Serial number mismatch: expected " << serialNumber << " but got " << config["serialNumber"] << std::endl;
      return 15;
    }
    if (cameras.size() != config["cameras"].size()) {
      std::cerr << "Number of cameras mismatch: expected " << cameras.size() << " but got " << config["cameras"].size() << std::endl;
      return 16;
    }
    std::cout << "Read configuration from " << configPath << std::endl;

    // Construct a vector of CameraRenderInfo objects from the configuration file, adding an image
    // queue to each.
    std::vector<asdp::render::CameraRenderInfo> cameraRenderInfos;
    try {
      for (const auto& camera : config["cameras"]) {
        asdp::render::CameraRenderInfo info;
        info.m_ID = camera["id"];
        info.m_positionMeters = camera["positionMeters"];
        info.m_orientationDegrees = camera["orientationDegrees"];
        info.m_resolutionPixels = camera["resolutionPixels"];
        info.m_fovDegrees = camera["fieldOfViewDegrees"];
        for (double d : camera["distortion"]) {
          info.m_distortion.push_back(d);
        }
        info.m_imageQueue = std::make_shared<asdp::render::ImageQueue>();
        cameraRenderInfos.push_back(info);
      }
    }  catch (const std::exception& e) {
      std::cerr << "Error parsing configuration file: " << e.what() << std::endl;
      return 17;
    }

    // Construct shared pointers to the data structures that we'll need to do rendering, with the
    // custom destructors that will clean up when the shared_ptr is destroyed.
    std::atomic<bool> done(false);
    std::shared_ptr< SpinFreeQueue< std::shared_ptr<DataToSendToGPU> > > dataQueue =
      std::make_shared< SpinFreeQueue< std::shared_ptr<DataToSendToGPU> > >();
    std::vector< std::shared_ptr<unsigned char> > cpuPinnedImageBuffers;
    std::vector< std::shared_ptr<unsigned char> > gpuImageBuffers;
    std::vector< std::shared_ptr<cudaStream_t> > streams;
    std::vector< std::shared_ptr<ReceiverUDP> > UDPReceivers;
    for (size_t i = 0; i < cameras.size(); i++) {
      // Allocate pinned memory for the CPU image buffer.
      unsigned char* cpuPinnedImageBuffer;
      cudaMallocHost(&cpuPinnedImageBuffer, cameras[i].width * cameras[i].height * sizeof(uint16_t));
      cpuPinnedImageBuffers.push_back(std::shared_ptr<unsigned char>(cpuPinnedImageBuffer,
        [](unsigned char* ptr) { cudaFreeHost(ptr); }
      ));

      // Allocate memory for the GPU image buffer.
      unsigned char* gpuImageBuffer;
      cudaMalloc(&gpuImageBuffer, cameras[i].width * cameras[i].height * sizeof(uint16_t));
      gpuImageBuffers.push_back(std::shared_ptr<unsigned char>(gpuImageBuffer,
        [](unsigned char* ptr) { cudaFree(ptr); }
      ));

      // Create a stream for the GPU to use.
      cudaStream_t* streamPtr = new cudaStream_t;
      cudaStreamCreate(streamPtr);
      streams.push_back(std::shared_ptr<cudaStream_t>(streamPtr,
        [](cudaStream_t* ptr) { cudaStreamDestroy(*ptr); delete ptr; }
      ));

      // Create a UDP receiver for the camera.
      std::shared_ptr<ReceiverUDP> receiverUDP = std::make_shared<ReceiverUDP>();
      if (receiverUDP->GetConstructorStatus() != OKAY) {
        std::cerr << "Error constructing ReceiverUDP: " << ErrorMessage(receiverUDP->GetConstructorStatus()) << std::endl;
        return 18;
      }
      UDPReceivers.push_back(receiverUDP);
    }

    // Launch the threads, hooking them together using the queues.
    std::thread copyDataToGPUThread(CopyDataToGPU, cameras[0].width, cameras[0].height,
      std::ref(done), dataQueue, 16);
    std::vector<std::thread> receiveDataThreads;
    for (size_t i = 0; i < cameras.size(); i++) {
      receiveDataThreads.push_back(std::thread(ReceiveDataThread, std::ref(*UDPReceivers[i]), 9000,
        std::ref(done), cpuPinnedImageBuffers[i], gpuImageBuffers[i], streams[i], dataQueue));
    }

    // Request streaming on the cameras at their maximum rates.
    std::cout << "Streaming every " << frameStride << " frames from " << cameraIDs.size() << " cameras" << std::endl;
    for (auto& camID : cameraIDs) {
      // Find the minimum period for the camera and which internal trigger ID it uses, then
      // configure the trigger to run at that rate.
      TriggerInfo ti;
      ti.ID = cameras[camID - 1].trigger;
      ti.mode = 1;
      ti.period = cameras[camID - 1].minTriggerPeriod;
      ti.offset = 0;
      ti.trackingFactor = 0.5;
      status = client.SendCommandPacket(CommandPacketConfigureTrigger(ti));
      if (status != OKAY) {
        std::cerr << "Failed to configure trigger: " << ErrorMessage(status) << std::endl;
        return 29;
      }

      // Request the camera to stream full-frame images once every frameStride frames.
      uint16_t port;
      status = UDPReceivers[camID - 1]->GetPort(port);
      if (status != OKAY) {
        std::cerr << "Failed to get port: " << ErrorMessage(status) << std::endl;
        return 30;
      }
      StreamEndpoint endpoint(ip_address, port);
      SubregionDescription region;
      region.cameraID = camID;
      region.skipFrames = frameStride - 1;
      region.startTimeSeconds = 0;
      region.startTimeMicroseconds = 0;
      region.left = 0;
      region.top = 0;
      region.right = cameras[camID - 1].width - 1;
      region.bottom = cameras[camID - 1].height - 1;
      status = client.SendCommandPacket(CommandPacketStreamSubregion(endpoint, region));
      if (status != OKAY) {
        std::cerr << "Failed to stream images: " << ErrorMessage(status) << std::endl;
        return 32;
      }
    }

    // Render frames until someone has marked us to be done.
    start = std::chrono::steady_clock::now();
    while (!done) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Set done and wait for all of our singleton threads to join.
    done = true;
    copyDataToGPUThread.join();

    // Clear all remaining data from the queues.
    // Do this in a block so that the shared_ptrs are destroyed before we join the receiveDataThreads.
    {
      std::shared_ptr<DataToSendToGPU> data;
      while (dataQueue->dequeue(data, std::chrono::milliseconds(1))) {}
    }

    // Now that all of the buffers have been returned to the buffer queue, join our receive-data threads.
    for (auto& thread : receiveDataThreads) {
      thread.join();
    }
  }

  return 0;
}