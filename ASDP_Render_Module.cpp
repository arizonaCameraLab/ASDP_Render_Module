/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

// This is a client that connects to the first server it encounters and runs a Render Module.

/**
 * @file ASDP_Render_Module.cpp
 * @brief Apache Strap-Down Pilotage Render Module.
 *
* @author ReliaSolve.
* @date May 20th, 2024.
*/

#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include <map>
#include <set>
#include <mutex>
#include <thread>
#include <string>
#include <filesystem>
#include <vector>
#include <list>
#include <atomic>
#include <ASDP_Core_API.h>
#include <ASDP_SpinFreeQueue.hpp>
#include <ASDP_BufferPool.h>
#include <ASDP_StreamPacketSortedQueue.h>
#include <ASDP_ClockSynchronizer.h>
#include "PinnedBufferPool.h"
#include "GPUBufferPool.h"
#include <nlohmann/json.hpp>
#include <GL/glew.h>
#include <ToneMap.h>
#include <RenderTimingInfo.h>
#include <CameraRenderInfo.h>
#include <Composite.h>
#include <Display.h>
#include <CPUDataToTextureHandler.h>
#include <PoseAdjuster.h>
#include <DepthEstimator.h>
#include <ImageStatistics.h>
#include <RangeEstimator.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>

using namespace asdp;
using namespace asdp::render;
using json = nlohmann::json;

static std::string VERSION = "2.38.0";

/// @brief The path to the configuration file. Defined in the CMakeLists file.
std::filesystem::path g_dirPath = CONFIG_FILE_PATH;

/// @brief Global variable set by callback handlers to tell when we're playing and pausing.
std::atomic<bool> g_paused(false);

/// @brief Global variable to hold the timing information for the program.
asdp::render::RenderTimingInfo g_timingInfo;

/// @brief Global variable to hold the depth estimator.
std::shared_ptr<DepthEstimator> g_depthEstimator;

/// @brief Global variables to hold the visible and depth cameras.
std::vector< std::shared_ptr<asdp::render::CameraRenderInfo> > g_visibleCameras, g_depthCameras;

/// @brief Global variable to hold the composite cameras.
std::shared_ptr<CompositeCameras> g_composite;

/// @brief Callback handler to toggle play and pause.
static void ChangePlayPause(bool nowPlaying, void* /* unused */)
{
  g_paused = !nowPlaying;
  std::cout << "Toggled play/pause to: " << (g_paused ? "paused" : "playing") << std::endl;
}

/// @brief Callback handler to compute depth information for the cameras.
static void ComputeDepth(Time renderTime, void* /* unused */)
{
  g_timingInfo.depthStartTimes.push_back(std::chrono::steady_clock::now());

  // Make a snapshot of the images from all cameras at the same time and store it into
  // a custom ImageQueue that has a single entry from the same time for all of them.
  /// @todo

  // Compute the depth and then use it to adjust the mesh for all rendered cameras and
  // then update the vertex buffer for the camera on the Composite.
  std::string ret = g_depthEstimator->ComputeDepthEstimate(renderTime);
  if (ret != "") {
    std::cerr << "Error computing depth estimate: " << ret << std::endl;
  } else {
    for (std::shared_ptr<asdp::render::CameraRenderInfo> cri : g_visibleCameras) {
      g_depthEstimator->UpdateMesh(*cri);
      g_composite->UpdateVertexBuffer(*cri);
    }
  }

  g_timingInfo.depthEndTimes.push_back(std::chrono::steady_clock::now());
}

/// @brief Callback handler to turn on and off depth rendering on the visible cameras.
static void ChangeDepthRendering(bool depthRendering, void* /* unused */)
{
  for (std::shared_ptr<asdp::render::CameraRenderInfo> cri : g_visibleCameras) {
    if (depthRendering) {
      // Set to clamp to white at a distance of 200 meters, to give us some resolution below that.
      cri->m_depthScale = 1.0f / 200;
    } else {
      cri->m_depthScale = -1.0f;
    }
  }
  std::cout << "Toggled depth rendering to: " << (depthRendering ? "on" : "off") << std::endl;
}

/// @brief Helper function to pull information from a FRAME_BEGIN message.
/// @param message The message to pull the information from.
/// @param cameraID The camera ID that the data is for.
/// @param width The width of the image data.
/// @param height The height of the image data.
/// @param exposure The exposure time for the image data.
/// @param gain The gain for the image data.
/// @return OKAY on success, error status on failure.
asdp::Status ParseFrameBeginMessage(Message &message, uint32_t &cameraID, uint16_t& width, uint16_t& height,
  float &exposure, float &gain)
{
  MessageFrameBegin frameBegin(message);
  if (frameBegin.GetConstructorStatus() != OKAY) {
    return frameBegin.GetConstructorStatus();
  }
  Status status = frameBegin.GetCameraID(cameraID);
  if (status != OKAY) {
    return status;
  }
  status = frameBegin.GetSensorWidth(width);
  if (status != OKAY) {
    return status;
  }
  status = frameBegin.GetSensorHeight(height);
  if (status != OKAY) {
    return status;
  }
  status = frameBegin.GetExposure(exposure);
  if (status != OKAY) {
    return status;
  }
  status = frameBegin.GetGain(gain);
  if (status != OKAY) {
    return status;
  }
  return OKAY;
}

/// @brief Helper function to pull information from a FRAME_DATA message.
/// @param message The message to pull the information from.
/// @param cameraID The camera ID that the data is for.
/// @param left The left edge of the region to process.
/// @param right The right edge of the region to process.
/// @param top The top edge of the region to process.
/// @param bottom The bottom edge of the region to process.
/// @param dataPtr The pointer to the data to process.
/// @return OKAY on success, error status on failure.
asdp::Status ParseFrameDataMessage(Message &message, uint32_t &cameraID,
  uint16_t& left, uint16_t& right, uint16_t& top, uint16_t& bottom, uint8_t*& dataPtr)
{
  MessageFrameData frameData(message);
  if (frameData.GetConstructorStatus() != OKAY) {
    return frameData.GetConstructorStatus();
  }
  Status status = frameData.GetCameraID(cameraID);
  if (status != OKAY) {
    return status;
  }
  status = frameData.GetLeft(left);
  if (status != OKAY) {
    return status;
  }
  status = frameData.GetRight(right);
  if (status != OKAY) {
    return status;
  }
  status = frameData.GetTop(top);
  if (status != OKAY) {
    return status;
  }
  status = frameData.GetBottom(bottom);
  if (status != OKAY) {
    return status;
  }
  status = frameData.GetDataPointer(dataPtr);
  if (status != OKAY) {
    return status;
  }
  return OKAY;
}

/// @brief Helper function to pull information from a FRAME_END message.
/// @param message The message to pull the information from.
/// @param cameraID The camera ID that the data is for.
/// @return OKAY on success, error status on failure.
asdp::Status ParseFrameEndMessage(Message &message, uint32_t &cameraID)
{
  MessageFrameEnd frameEnd(message);
  if (frameEnd.GetConstructorStatus() != OKAY) {
    return frameEnd.GetConstructorStatus();
  }
  Status status = frameEnd.GetCameraID(cameraID);
  if (status != OKAY) {
    return status;
  }
  return OKAY;
}


/// @brief Thread for each camera that receives the data from the network and sends it to the GPU.
/// @param receiveSocket The socket to receive the data on.
/// @param maxBytesPerPacket The maximum number of bytes in a packet.
/// @param done The flag to set when we're done.
/// @param cpuImageBuffers Pool of pinned memory buffer on the CPU to hold the image data.
/// @param gpuImageBuffers Pool of buffers on the GPU to hold the image data.
/// @param streamPtr The stream to use for copy and kernel calls.
/// @param imageQueue The image queue to store the textures in.
/// @param outQueue The queue to send the data to the GPU-feeding thread.
/// @param frameBeginTimes Store the times for the begin frame message receipts.
/// @param frameEndTimes Store the times for the end frame message receipts.
void ReceiveDataThread(ReceiverUDP& receiveSocket, size_t maxBytesPerPacket, std::atomic<bool>& done,
  std::shared_ptr<PinnedBufferPool> cpuImageBuffers, std::shared_ptr<GPUBufferPool> gpuImageBuffers,
  std::shared_ptr<cudaStream_t> streamPtr,
  std::shared_ptr<asdp::render::ImageQueue> imageQueue,
  std::shared_ptr< SpinFreeQueue< std::shared_ptr<DataToSendToGPU> > > outQueue,
  std::vector<std::chrono::steady_clock::time_point> &frameBeginTimes,
  std::vector<std::chrono::steady_clock::time_point> &frameEndTimes)
{
  // Generate a buffer pool to use to get pre-allocated buffers for reading the data from
  // the network.  Initially fill it with 100 buffers to give us enough to handle buffering a fraction
  // of a frame before the first packets are handled.  It will automatically expand if needed.
  BufferPool bufferPool(maxBytesPerPacket, 100);

  // CPU and GPU buffers to hold the image data.  These will be created when we get a frame begin message,
  // and we wait until we get a frame begin message before we start processing data.
  std::shared_ptr<uint8_t> cpuImageBufferPtr;
  std::shared_ptr<uint8_t> gpuImageBufferPtr;

  // Image width
  uint16_t cameraWidth = 0;

  // Use a sorting queue to ensure that we process the messages in order even if the UDP packets
  // arrive out of order.
  StreamPacketSortedQueue sortedQueue(50);

  // Loop through and receive packets until we've been told to quit.
  DataToSendToGPU data;
  bool waitingForFrameBegin = true;
  while (!done) {

    // Get the next packet into a preallocated buffer, timing out quickly to ensure that we check
    // the done flag.
    std::shared_ptr< std::vector<uint8_t> > buffer = bufferPool.GetBuffer();
    size_t offset = 0;
    std::shared_ptr<StreamPacket> packet;
    Status status = receiveSocket.ReceiveStreamPacket(0.1, packet, offset, buffer);
    if (status == TIMEOUT) {
      continue;
    }
    if (status != OKAY) {
      std::cerr << "Error receiving data: " << ErrorMessage(status) << std::endl;
      done = true;
      break;
    }

    // Add to the sorted queue and then handle any messages that are ready to be processed.
    std::list< std::shared_ptr<StreamPacket> > readyPackets = sortedQueue.AddPacket(packet);
    if (readyPackets.size() > 1) {
      std::cerr << "Warning: More than one packet ready to process (re-ordered or missing packet)." << std::endl;
    }
    while (!readyPackets.empty()) {
      std::shared_ptr<StreamPacket> streamPacket = readyPackets.front();
      readyPackets.pop_front();

      // Because we must copy the data into pinned memory for data messages, and because we must
      // check for begin-frame messages before sending anything, we must parse all of the
      // messages in the packet and handle them in turn.  We will ignore any messages that are not
      // these types.  Store summaries of each message so we can process them in the other thread.
      // NOTE: The following code relies on every message being sent in a separate packet so that
      // we don't swap out the pinned memory buffer or GPU buffer while they are being used.
      std::vector<MessageSummary> messageSummaries;
      std::shared_ptr<Message> message;
      status = streamPacket->GetNextMessage(message);
      if (OKAY != status) {
        std::cerr << "ReceiveDataThread: GetNextMessage() failed: " << ErrorMessage(status) << std::endl;
        done = true;
        return;
      }
      while (message != nullptr) {
        MessageID messageType;
        if (OKAY != message->GetType(messageType)) {
          std::cerr << "ReceiveDataThread: Error getting message type: " << ErrorMessage(status) << std::endl;
          done = true;
          return;
        }
        switch (messageType) {
        case FRAME_BEGIN:
          {
            // Log our timing data.
            frameBeginTimes.push_back(std::chrono::steady_clock::now());

            // We found a begin frame message, so we can start processing the data.
            waitingForFrameBegin = false;

            // Pull the information from the frame so that we can store the width data for this
            // camera.
            uint32_t cameraID;
            uint16_t width, height;
            float exposure, gain;
            status = ParseFrameBeginMessage(*message, cameraID, width, height, exposure, gain);
            if (OKAY != status) {
              std::cerr << "ReceiveDataThread: ParseFrameBeginMessage() failed: " << ErrorMessage(status) << std::endl;
              done = true;
              return;
            }
            cameraWidth = width;

            // Get a new pinned CPU memory and GPU memory buffer to hold the image data.
            // The old ones will be returned to the pool when the shared pointers are reset.
            try {
              // Do not allocate new buffers if they are depleted; wait for them to be returned.
              cpuImageBufferPtr = cpuImageBuffers->GetBuffer(false);
              gpuImageBufferPtr = gpuImageBuffers->GetBuffer(false);
            } catch (std::exception& e) {
              std::cerr << "Error getting buffers: " << e.what() << std::endl;
              done = true;
              return;
            }

            // Store the summary
            MessageSummary summary;
            summary.messageType = FRAME_BEGIN;
            message->GetTime(summary.time);
            summary.cameraID = cameraID;
            summary.width = width;
            summary.height = height;
            summary.exposure = exposure;
            summary.gain = gain;
            messageSummaries.push_back(summary);
          }
          break;
        case FRAME_DATA:
          // We're waiting for the frame begin message, so we ignore this packet that does not have one.
          if (!waitingForFrameBegin) {
            // Get the region to copy and the data pointer from the message.
            uint32_t cameraID;
            uint16_t left, right, top, bottom;
            uint8_t* data;
            status = ParseFrameDataMessage(*message, cameraID, left, right, top, bottom, data);
            if (OKAY != status) {
              std::cerr << "ReceiveDataThread: ParseFrameDataMessage() failed: " << ErrorMessage(status) << std::endl;
              done = true;
              return;
            }

            // Copy the data to the pinned CPU memory buffer.
            uint16_t regionWidth = right - left + 1;
            uint16_t regionHeight = bottom - top + 1;
            uint16_t* cpuBuffer16 = reinterpret_cast<uint16_t*>(cpuImageBufferPtr.get());
            uint16_t* data16 = reinterpret_cast<uint16_t*>(data);
            if (cameraWidth != 0) {
              if ((left == 0) && (regionWidth == cameraWidth)) {
                // If we're copying whole lines, we can do it all at once.
                memcpy(cpuBuffer16 + top * regionWidth, data16, regionWidth * regionHeight * sizeof(uint16_t));
              } else {
                // Otherwise, we must do it line by line.
                for (uint16_t line = top; line <= bottom; ++line) {
                  memcpy(cpuBuffer16 + line * cameraWidth + left, data16 + (line - top) * regionWidth, regionWidth * sizeof(uint16_t));
                }
              }
            }

            // Store the summary
            MessageSummary summary;
            summary.messageType = FRAME_DATA;
            message->GetTime(summary.time);
            summary.cameraID = cameraID;
            summary.left = left;
            summary.right = right;
            summary.top = top;
            summary.bottom = bottom;
            messageSummaries.push_back(summary);
          }
          break;
        case FRAME_END:
          // We're waiting for the frame begin message, so we ignore this packet that does not have one.
          if (!waitingForFrameBegin) {

            // Log our timing data.
            frameEndTimes.push_back(std::chrono::steady_clock::now());

            // Parse the message so we can make a summary.  We don't do anything else with it here.
            uint32_t cameraID;
            status = ParseFrameEndMessage(*message, cameraID);
            if (OKAY != status) {
              std::cerr << "ReceiveDataThread: ParseFrameEndMessage() failed: " << ErrorMessage(status) << std::endl;
              done = true;
              return;
            }

            // Store the summary
            MessageSummary summary;
            summary.messageType = FRAME_END;
            message->GetTime(summary.time);
            summary.cameraID = cameraID;
            messageSummaries.push_back(summary);

            // We're at the end of a frame, so we need to get a begin-frame message next.
            waitingForFrameBegin = true;
          }
          break;
        default:
          // Ignore other message types.
          break;
        }

        status = streamPacket->GetNextMessage(message);
        if (OKAY != status) {
          done = true;
          std::cerr << "ReceiveDataThread: GetNextMessage() failed: " << ErrorMessage(status) << std::endl;
          return;
        }
      }

      // Don't queue if we have no messages.
      if (messageSummaries.size() == 0) {
        continue;
      }

      // Make sure that we only have one message per packet.  If we get more, we will have to modify the
      // pinned and GPU memory buffers to handle it.
      if (messageSummaries.size() > 1) {
        std::cerr << "Error: More than one message per packet." << std::endl;
        done = true;
        return;
      }

      // Enqueue the packet for processing.
      data.messages = messageSummaries;
      data.cpuImageBufferPtr = cpuImageBufferPtr;
      data.gpuImageBufferPtr = gpuImageBufferPtr;
      data.imageQueuePtr = imageQueue;
      data.streamPtr = streamPtr;
      outQueue->enqueue(std::make_shared<DataToSendToGPU>(data));
    } // End of processing ready packets.
  } // End of while we are not done.

  // Release our out-queue pointer so it will be destroyed and release all its resources back to our
  // buffer pool.
  outQueue.reset();
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

/// @param [out] replayDone Set to true if we are at the end of replay, set to false otherwise.
static Status HandleStreamPacket(std::shared_ptr<StreamPacket> packet, std::shared_ptr<ClockSynchronizer> clockSync,
  std::shared_ptr<PoseAdjuster> poseAdjuster, bool &replayDone, std::vector<std::shared_ptr<Display>> &displays,
  std::shared_ptr<Timer> timer, Time &pausedTime)
{
  // Not done replaying unless we get a message telling us that we are.
  replayDone = false;

  // Parse all of the messages in the stream packet, handling each of them in turn.
  std::shared_ptr<Message> message;
  Status status = packet->GetNextMessage(message);
  if (OKAY != status) {
    return status;
  }
  while (message != nullptr) {
    MessageID messageType;
    status = message->GetType(messageType);
    if (OKAY != status) {
      return status;
    }

    switch (messageType) {
    case EVENT:
      {
        // Find the event type and handle it.
        MessageEvent event(*message);
        if (event.GetConstructorStatus() != OKAY) {
          return event.GetConstructorStatus();
        }
        EventID eventType;
        status = event.GetType(eventType);
        if (status != OKAY) {
          return status;
        }
        // Get the string message.
        std::string messageString;
        status = event.GetParam(messageString);
        if (status != OKAY) {
          return status;
        }
        switch (eventType) {
          case START_OF_REPLAY:
            {
              // Reset the clock-sync estimates when we start, stop, or resume replay.
              clockSync->ClearHistory();
            }
            break;
          case END_OF_REPLAY:
            {
              // Reset the clock-sync estimates when we start, stop, or resume replay.
              clockSync->ClearHistory();
            }
            break;
          case REPLAY_PAUSED:
            {
              // Store the time that we're paused at so that we can reset our clock-sync estimates
              // when we resume.
              status = message->GetTime(pausedTime);
              if (status != OKAY) {
                return status;
              }

              // Tell all of our Displays that we're paused.
              for (auto &display : displays) {
                display->SetNowPlaying(false);
              }
            }
            break;
          case REPLAY_RESUMED:
            {
              // Reset the clock-sync estimates when we start, stop, or resume replay.
              clockSync->ClearHistory();

              // Add an entry to the clock-sync estimates based on the time we were paused,
              // making the current time match it.  Then reset the history again so that this
              // phantom entry doesn't affect the estimates.  We needed to reset the history
              // before doing this so that our single entry causes the shift that we want.
              if (!clockSync->AddDataPoint(pausedTime, std::chrono::steady_clock::now())) {
                return UNEXPECTED_INTERNAL_STATE;
              }
              clockSync->ClearHistory();

              // Tell all of our Displays that we're no longer paused.
              for (auto& display : displays) {
                display->SetNowPlaying(true);
              }
            }
            break;

          case CLOCK_SYNC:
            {
              // Adjust the timer offset based on clock-sync messages.  The first message (or the first one
              // after replay resumes, or the first one after replay stops), sets the estimated offset based
              // on that single number and the relative rate to 1.0. Later ones adjust based on an average of
              // the previous ones as described in the render implementation document.
              Time messageTime;
              status = message->GetTime(messageTime);
              if (status != OKAY) {
                return status;
              }
              clockSync->AddDataPoint(messageTime, std::chrono::steady_clock::now());
            }
            break;

          case INVALID_OPERATION:
            {
              // If we get an invalid operation message, say so
              std::cerr << "Invalid operation message received from server: " << messageString << std::endl;
            }
            break;

          case INTERNAL_ERROR:
          {
            // If we get an internal error message, say so
            std::cerr << "Internal error message received from server: " << messageString << std::endl;
          }
          break;

          case UNRECOGNIZED_OPCODE:
          {
            // If we get an unrecognized opcode message, say so
            std::cerr << "Unrecognized opcode message received from server: " << messageString << std::endl;
          }
          break;

          default:
            break;
        }
      }
      break;
    case STATE:
      {
        // Parse the state message and keep track of anything we need to.
        MessageState state(*message);
        if (state.GetConstructorStatus() != OKAY) {
          return state.GetConstructorStatus();
        }
        uint8_t replaying;
        status = state.GetReplaying(replaying);
        if (status != OKAY) {
          return status;
        }
        //std::cout << "XXX Replaying = " << (replaying ? "true" : "false") << std::endl;

        // If we're replaying and we're at the end of replay, indicate this.
        if (replaying) {
          uint8_t endOfReplay;
          status = state.GetReplayAtEnd(endOfReplay);
          if (status != OKAY) {
            return status;
          }
          if (endOfReplay) {
            replayDone = true;
          }
        }
      }
      break;
    case POSE:
      {
        // Parse the pose message and add the pose to the adjuster.
        MessagePose pose(*message);
        if (pose.GetConstructorStatus() != OKAY) {
          return pose.GetConstructorStatus();
        }
        poseAdjuster->AddPose(pose);
      }
      break;
    default:
      // Ignore other message types.
      break;
    }

    status = packet->GetNextMessage(message);
    if (OKAY != status) {
      return status;
    }
  }

  return OKAY;
}

/// @brief Structure to hold display information
struct DisplayInfo
{
  ToneMap toneMap = ToneMap();  ///< The tone map to use.
  bool useOpenXR = false;       ///< Use OpenXR for rendering? If so, overrides all of the following.
  std::string XSightNIC = "";   ///< NIC to listen to XSight on for rendering. If not empty, overrides all of the following.
  int width = 1280;             ///< The width of the display.
  int height = 1024;            ///< The height of the display.
  float hFOV = 40.0f;           ///< The horizontal field of view in degrees.
  std::string joystick = "";    ///< The joystick to use for input.
  float fps = 60.0f;            ///< The frames per second to run at.
  bool fullScreen = false;      ///< Run in full screen mode.
  int fullScreenDisplay = 0;    ///< The display to run in full screen mode on.

  //======================================
  // Added by Sang Yoon to add a flag for enabling the cylindrical projection.
  bool enableCP = false;        ///< The flag to enable the cylindrical projection
  //======================================

  //======================================
  // Added by Sang Yoon to indicate if the window associated with the display is overview window, detailed view windowe, or neither.
  // Where the number of displays is greater than 1, the window that has the widest horizontal FOV is considered as an overview window,
  // and the window that has the narrowest hFOV is considered as a detailed view window.
  bool overview = false;
  bool detailed_view = false;
  //======================================
};

static std::string TimeIntervalToStringMilliseconds(std::chrono::duration<float> interval)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(9) << interval.count() * 1000;
  return oss.str();
}

static std::chrono::steady_clock::time_point LargestTimeLessThan(std::chrono::steady_clock::time_point time,
  const std::vector<std::chrono::steady_clock::time_point>& times)
{
  std::chrono::steady_clock::time_point largest = std::chrono::steady_clock::time_point::min();
  for (auto &t : times) {
    if (t < time) {
      largest = std::max(largest, t);
    }
  }
  return largest;
}

static void usage(std::string name)
{
  std::cerr << "Usage: " << name << " [options] <ip_address>" << std::endl;
  std::cerr << "  <ip_address>                        The IP address to listen for servers on." << std::endl;
  std::cerr << "  Options:" << std::endl;
  std::cerr << "  --help                              Print this help message." << std::endl;
  std::cerr << "  --frameStride <frame stride>        Read one out of every this many frames. Set to 1 for every frame." << std::endl;
  std::cerr << "  --toneMap <tone map>                The tone map to use.  Options are: linear blackbody bluesky" << std::endl;
  std::cerr << "  --addDisplay                        Add another display with defaults that can be overridden" << std::endl;
  std::cerr << "  --replay <stream id>                ID of the stream to replay (1+)." << std::endl;
  std::cerr << "  --loopReplay                        Loop the replay (default not)." << std::endl;
  std::cerr << "  --lineBatchesPerGPUSend <int>       The number of batches of lines to group (default 16 Linux, 110 Windows)" << std::endl;
  std::cerr << "  --noPoses                           Do not stream poses from the server, so no latency adjustment." << std::endl;
  std::cerr << "  --dumpTiming <file name base>       Write timing on quit to CSV files with the specified base name." << std::endl;
  std::cerr << "  --triggerAheadMicroseconds <int>    Microseconds ahead of render to trigger camera (default 22000)." << std::endl;
  std::cerr << "  --depthAheadMicroseconds <int>      Microseconds ahead of render to compute depth (default 8000)." << std::endl;
  std::cerr << "  --lockRotation                      Lock the rotation of the viewer to the initial helicopter pose." << std::endl;
  std::cerr << "  --disableLatencyCompensation        Disable latency compensation." << std::endl;
  std::cerr << "  --autoRangeStd <below> <above>      Adjust color range to specified standard deviations above and below the mean." << std::endl;
  std::cerr << "  --noDepth                           Do not compute depth even when stereo cameras are available." << std::endl;
  std::cerr << "  --maxDepth <float>                  Maximum depth to test for in meters (default 200)." << std::endl;
  std::cerr << "  --depthThreshold <float>            Depth threshold in squared pixel value differences (default 10.0)." << std::endl;
  std::cerr << "  --cameraFPS <frames per second>     The frames per second to run the camera at (default is maximum rate)." << std::endl;
  std::cerr << "  --enableCP                          Enable the cylindrical projection." << std::endl; // Added by Sang Yoon
  std::cerr << "  --openXR                            Use OpenXR for rendering. If set, overrides the following and sets lineBatchesPerGPUSend to 10000." << std::endl;
  std::cerr << "  --xSight <ip of NIC to listen on>   Render to XSight on specified NIC. If set, overrides the following." << std::endl;
  std::cerr << "  --width <width>                     The width of the window (default 1280)." << std::endl;
  std::cerr << "  --height <height>                   The height of the window (default 1024)." << std::endl;
  std::cerr << "  --hFOV <horizontal field of view>   The horizontal field of view in degrees (default 40)." << std::endl;
  std::cerr << "  --joystick <string>                 The joystick to use for input (e.g. GLFW::0)." << std::endl;
  std::cerr << "  --fps <frames per second>           The frames per second to run at (default 60)." << std::endl;
  std::cerr << "  --fullScreen <display>              Run in full screen mode on the specified display (0+)." << std::endl;
};

int main(int argc, char** argv)
{
  uint32_t frameStride = 1;     ///< Read one out of every this many frames. Set to 1 for every frame.
  std::vector<DisplayInfo> displayInfos = { DisplayInfo() }; ///< Information for each display that is to be created.
  std::string ip_address;       ///< The IP address to listen on.
  uint32_t replayStreamID = 0;  ///< The stream ID to replay, 0 for live.
  bool loopReplay = false;      ///< Loop the replay when it reaches the end if this is true.
#ifdef _WIN32
  // On Windows, throughput tests when receiving data from the network show that we must be larger
  // to keep up.  Linux is more efficient here, and can handle 16 batches at a time.
  int lineBatchesPerGPUSend = 110; ///< The number of batches of lines to group for sending to the GPU.
#else
  int lineBatchesPerGPUSend = 16; ///< The number of batches of lines to group for sending to the GPU.
#endif
  bool doStreamPoses = true;      ///< Stream poses from the server, so we can adjust for latency.
  std::string dumpTimingFileName; ///< The base name for the timing files.
  unsigned triggerAheadMicroseconds = 22000; ///< Microseconds ahead of render to trigger camera.
  unsigned depthAheadMicroseconds = 8000;    ///< Microseconds ahead of render to compute depth.
  bool lockRotation = false;      ///< Lock the rotation of the viewer to the initial helicopter pose.
  bool disableLatencyCompensation = false; ///< Disable latency compensation.
  double cameraFPS = 0.0;         ///< The frames per second to run the camera at, 0 defaults to camera-specified maximum.
  double autoRangeStdBelow = 0.0; ///< Adjust color range to this many standard deviations below the mean.
  double autoRangeStdAbove = 0.0; ///< Adjust color range to this many standard deviations above the mean.
  bool computeDepth = true;       ///< Compute depth when stereo cameras are available.
  float maxDepth = 200.0f;        ///< Maximum depth to test for in meters.
  float depthThreshold = 10.0f;   ///< Depth threshold in squared pixel value differences.
  size_t realParams = 0;          ///< The number of non-flag parameters we've seen.

  // Parse the command line arguments, with the first non-flag argument being the
  // name of the IP address to listen on.
  for (int i = 1; i < argc; ++i) {
    if (std::string("--frameStride") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 2;
      }
      frameStride = std::stoi(argv[i]);
    }
    else if (std::string("--width") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 2;
      }
      displayInfos.back().width = std::stoi(argv[i]);
    }
    else if (std::string("--height") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 2;
      }
      displayInfos.back().height = std::stoi(argv[i]);
    }
    else if (std::string("--hFOV") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 2;
      }
      displayInfos.back().hFOV = std::stof(argv[i]);
    }
    else if (std::string("--openXR") == argv[i]) {
      displayInfos.back().useOpenXR = true;
      lineBatchesPerGPUSend = 10000;
    }
    else if (std::string("--xSight") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 2;
      }
      displayInfos.back().XSightNIC = argv[i];
    }
    else if (std::string("--fullScreen") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 2;
      }
      displayInfos.back().fullScreen = true;
      displayInfos.back().fullScreenDisplay = std::stoi(argv[i]);
    } else if (std::string("--fps") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 2;
      }
      displayInfos.back().fps = std::stof(argv[i]);
    } else if (std::string("--joystick") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 2;
      }
      displayInfos.back().joystick = argv[i];
    } else if (std::string("--lineBatchesPerGPUSend") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 2;
      }
      lineBatchesPerGPUSend = std::stoi(argv[i]);
    } else if (std::string("--toneMap") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 2;
      }
      if (std::string("linear") == argv[i]) {
        displayInfos.back().toneMap = ToneMap();
      } else if (std::string("blackbody") == argv[i]) {
        displayInfos.back().toneMap = ToneMapBlackbody();
      } else if (std::string("bluesky") == argv[i]) {
        displayInfos.back().toneMap = ToneMapBlueSky();
      } else {
        std::cerr << "Unknown tone map: " << argv[i] << std::endl;
        return 2;
      }
    } else if (std::string("--addDisplay") == argv[i]) {
      displayInfos.push_back(DisplayInfo());
    } else if (std::string("--replay") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 2;
      }
      replayStreamID = std::stoi(argv[i]);
    } else if (std::string("--loopReplay") == argv[i]) {
      loopReplay = true;
    } else if (std::string("--noPoses") == argv[i]) {
      doStreamPoses = false;
    } else if (std::string("--dumpTiming") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 2;
      }
      dumpTimingFileName = argv[i];
    } else if (std::string("--triggerAheadMicroseconds") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 2;
      }
      triggerAheadMicroseconds = std::stoi(argv[i]);
    } else if (std::string("--depthAheadMicroseconds") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 2;
      }
      depthAheadMicroseconds = std::stoi(argv[i]);
    } else if (std::string("--lockRotation") == argv[i]) {
      lockRotation = true;
    } else if (std::string("--disableLatencyCompensation") == argv[i]) {
      disableLatencyCompensation = true;
    } else if (std::string("--autoRangeStd") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 2;
      }
      autoRangeStdBelow = std::stod(argv[i]);
      if (++i >= argc) {
        usage(argv[0]);
        return 2;
      }
      autoRangeStdAbove = std::stod(argv[i]);
    } else if (std::string("--noDepth") == argv[i]) {
      computeDepth = false;
    } else if (std::string("--maxDepth") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 2;
      }
      maxDepth = std::stof(argv[i]);
    } else if (std::string("--depthThreshold") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 2;
      }
      depthThreshold = std::stof(argv[i]);
    } else if (std::string("--cameraFPS") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 2;
      }
      cameraFPS = std::stod(argv[i]);
    } else if (std::string("--help") == argv[i]) {
      usage(argv[0]);
      return 0;

    //======================================
    // Added by Sang Yoon to add a command line argument to enable the cylindrical projection for the current display.
    // Note that each display (or window) can use either perspective projection (default) or cylindrical projection (enabled with --enableCP).
    }
    else if (std::string("--enableCP") == argv[i]) {
        displayInfos.back().enableCP = true;
    //======================================

    } else if (argv[i][0] == '-') {
      usage(argv[0]);
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

    // Create a PoseAdjuster to handle helicopter motion.
    PoseAdjusterCoordinates poseAdjusterCoordinates = HELICOPTER;
    if (lockRotation) {
      poseAdjusterCoordinates = INITIAL_ORIENTATION;
    }
    std::shared_ptr<PoseAdjuster> poseAdjuster = std::make_shared<PoseAdjuster>(2000, poseAdjusterCoordinates,
      disableLatencyCompensation);

    // Open a client, specifying the IP address to listen on.
    std::shared_ptr<CoreClient> client = std::make_shared<CoreClient>(ip_address);
    if (client->GetConstructorStatus() != OKAY) {
      std::cerr << "Failed to open client: " << ErrorMessage(client->GetConstructorStatus()) << std::endl;
      return 3;
    }
    std::cout << "Listening for servers on " << ip_address << std::endl;

    // Wait for up to two seconds to allow servers to send Discovery messages.
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    std::vector<std::string> servers;
    Status threadStatus;
    Status status;
    do {
      status = client->GetDiscoveryThreadStatus(threadStatus);
      if (status != OKAY) {
        std::cerr << "Failed to get discovery thread status: " << ErrorMessage(status) << std::endl;
        return 4;
      }
      if (threadStatus != OKAY) {
        std::cerr << "Discovery thread status: " << ErrorMessage(threadStatus) << std::endl;
        return 5;
      }
      status = client->IdentifiedServers(servers);
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
    status = client->ConnectToServer(servers[0], major, minor, patch);
    if (status != OKAY) {
      std::cerr << "Failed to connect to server: " << ErrorMessage(status) << std::endl;
      return 8;
    }
    uint32_t serialNumber;
    status = client->GetServerSerialNumber(serialNumber);
    if (status != OKAY) {
      std::cerr << "Failed to get server serial number: " << ErrorMessage(status) << std::endl;
      return 9;
    }
    std::cout << "  Connected to server version " << major << "." << minor << "." << patch
      << " with serial number " << serialNumber << std::endl;

    // Get the main stream receiver
    std::shared_ptr<Receiver> receiver;
    status = client->GetMainStreamReceiver(receiver);
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
    std::vector<FeatureID> features;
    status = state.GetFeatures(features);
    if (status != OKAY) {
      std::cerr << "Failed to get features: " << ErrorMessage(status) << std::endl;
      return 1000;
    }
    bool hasStorage = false;
    bool hasTemperatures = false;
    bool hasPoses = false;
    for (const auto& feature : features) {
      if (feature == STORAGE_API_AVAILABLE) {
        hasStorage = true;
      } else if (feature == TEMPERATURE_API_AVAILABLE) {
        hasTemperatures = true;
      } else if (feature == POSE_API_POSITION_AVAILABLE || feature == POSE_API_ORIENTATION_AVAILABLE) {
        hasPoses = true;
      }
    }

    // Find the trigger for the first camera, which we will use to synchronize to the display.  We assume that
    // they are all using the same trigger.  We don't send triggers when we replay.
    uint8_t triggerID = 0;
    if (cameras.size() > 0 && replayStreamID == 0) {
      triggerID = cameras[0].trigger;
    }

    // Read the configuration file associated with the serial number for the server. Verify that
    // it has a matching serial number and number of cameras.
    std::filesystem::path configPath = g_dirPath / (std::to_string(serialNumber) + ".json");
    if (!std::filesystem::exists(configPath)) {
      std::cerr << "Configuration file not found: " << configPath << std::endl;
      return 14;
    }
    std::ifstream configFile(configPath);
    json config = json::parse(configFile);
    uint32_t configSerial = config["serialNumber"];
    if (cameras.size() != config["cameras"].size()) {
      std::cerr << "Number of cameras mismatch: expected " << cameras.size() << " but got " << config["cameras"].size() << std::endl;
      return 16;
    }
    std::cout << "Read configuration from " << configPath << std::endl;

    // Construct a DisplayTexture object to handle textures.  It will be the base object that all others will use
    // to share contexts.
    std::shared_ptr<DisplayTexture> displayTexture = std::make_shared<DisplayTexture>();

    // Make additional OpenGL contexts for all but the first texture thread -- re-use the original for
    // the first one.
    int NUM_TEXTURE_THREADS = 2;
    if (cameras.size() > 21) {
#ifdef _WIN32
      // On Windows, we need larger batches of lines to keep up with more than 21 cameras. The jump from
      // default 110 to 330 has both cases ending at 990, which is just below the 1024 limit so will make
      // a small final batch, reducing the latency from the end of the frame receipt to texture upload.
      lineBatchesPerGPUSend *= 3;
#else
      // On Linux, we need an extra thread to keep up with the data rate when we get more than 21 cameras.
      // It may be that we could increase the lineBatchesPerGPUSend and get by with two threads.
      NUM_TEXTURE_THREADS = 3;
#endif
    }

    std::vector< std::shared_ptr<DisplayTexture> > displayTextures = { displayTexture };
    for (size_t i = 1; i < NUM_TEXTURE_THREADS; i++) {
      std::shared_ptr<DisplayTexture> dt = std::make_shared<DisplayTexture>(displayTexture.get());
      displayTextures.push_back(dt);
    }

    // Construct a vector of CameraRenderInfo objects from the configuration file, adding an image
    // queue to each.
    std::vector< std::shared_ptr<asdp::render::CameraRenderInfo> > cameraRenderInfos;
    try {
      for (const auto& camera : config["cameras"]) {
        std::shared_ptr<Distortion> dist;
        json distortion = camera["distortion"];
        if (distortion["type"] == "none") {
          DistortionNone* distortion = new DistortionNone;
          dist = std::shared_ptr<Distortion>(distortion);
        } else if (distortion["type"] == "radial") {
          json parameters = distortion["parameters"];
          // The center of projection in the file is specified in fractional half-image span in
          // the X and Y directions, but in piercing location of the Z=-1 plane for the Distortion
          // object.  We must convert the fractional half-image span to a piercing location.
          std::array<double, 2> center = parameters["COP"];
          double halfWidth = tan(glm::radians(double(camera["fieldOfViewDegrees"][0])) / 2.0);
          double halfHeight = tan(glm::radians(double(camera["fieldOfViewDegrees"][1])) / 2.0);
          center[0] *= halfWidth;
          center[1] *= halfHeight;
          json map = parameters["map"];
          std::vector< std::array<double, 2> > mapPoints = map;
          DistortionRadialLERP* distortion = new DistortionRadialLERP(center, mapPoints);
          dist = std::shared_ptr<Distortion>(distortion);
        } else if (distortion["type"] == "bagOfMappings") {
          json parameters = distortion["parameters"];
          DistortionBagOfMappings::Bag map = parameters["map"];
          DistortionBagOfMappings* distortion = new DistortionBagOfMappings(map);
          dist = std::shared_ptr<Distortion>(distortion);
        } else {
          std::cerr << "Error: Unknown distortion type: " << distortion["type"] << std::endl;
          return 17;
        }

        std::shared_ptr<Vignette> vig(new VignetteNone);
        if (camera.contains("vignette")) try {
          json vignette = camera["vignette"];
          if (!vignette.contains("type")) {
            // No vignette specified, so use the default.
          } else if (vignette["type"] == "evenPolynomial") {
            json parameters = vignette["parameters"];
            std::array<double, 2> center = parameters["COP"];
            std::array<double, 2> cArray = parameters["coefficients"];
            std::vector<double> coefficients(cArray.begin(), cArray.end());
            VignetteRadialPolynomail* vignette = new VignetteRadialPolynomail(center,
              camera["fieldOfViewDegrees"], coefficients);
            vig = std::shared_ptr<Vignette>(vignette);
          } else {
            std::cerr << "Error: Unknown vignette type: " << vignette["type"] << std::endl;
            return 18;
          }
        }
        catch (...) {
          // No vignette specified, so use the default.
        }

        std::shared_ptr<asdp::render::CameraRenderInfo> info =
          std::make_shared<CameraRenderInfo>(camera["id"],
        camera["positionMeters"], camera["orientationDegrees"],
        camera["resolutionPixels"], camera["fieldOfViewDegrees"],
        dist, vig, std::make_shared<asdp::render::ImageQueue>(), -1.0f);

        // Read the offset and gain from the color object if it is present and they are present.
        // Override the default values if they are present.
        if (camera.contains("color")) {
          float offset = 0.0f, gain = 1.0f;
          if (camera["color"].contains("offset")) {
            offset = camera["color"]["offset"];
          }
          if (camera["color"].contains("gain")) {
            gain = camera["color"]["gain"];
          }
          info->SetColorOffsetGain(offset, gain);
        }

        //==================================================================================================
        // Fill in three textures for this camera, all gray and at time zero.
        // We must borrow the context from the displayTexture so that we can create the textures.
        if (!displayTexture->BorrowContext()) {
          std::cerr << "Error borrowing context from displayTexture." << std::endl;
          return 17;
        }

        unsigned int width = info->m_resolutionPixels[0];
        unsigned int height = info->m_resolutionPixels[1];
        std::vector<uint16_t> image(width * height, 32767);

        // Create the textures for the camera. Make two for each Composite to pull when it is looking
        // for the next image to render, one for the texture thread to write to, one for an image-statistics
        // class to use, and one to lie fallow.
        for (size_t i = 0; i < 2*displayInfos.size() + 1 + 1 + 1; i++) {
          std::shared_ptr<ImageData> imageData = std::make_shared<ImageData>();

          unsigned int texture;
          glGenTextures(1, &texture);
          glBindTexture(GL_TEXTURE_2D, texture);
          // Set the texture wrapping parameters
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
          // Set texture filtering parameters
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

          // Load image into the texture
          glTexImage2D(GL_TEXTURE_2D, 0, GL_R16, width, height, 0, GL_RED, GL_UNSIGNED_SHORT, image.data());
          glBindTexture(GL_TEXTURE_2D, 0);

          imageData->texture = texture;
          info->m_imageQueue->InsertImage(imageData);
        }

        if (!displayTexture->ReturnContext()) {
          std::cerr << "Error returning context to displayTexture." << std::endl;
          return 18;
        }
        //
        //==================================================================================================

        // Push the CameraRenderInfo onto the vector.
        cameraRenderInfos.push_back(info);
      }
    }  catch (const std::exception& e) {
      std::cerr << "Error parsing configuration file: " << e.what() << std::endl;
      return 19;
    }

    std::vector<uint32_t> cameraIDs; ///< The camera IDs to render, in the same order as the records in the configuration file.
    for (uint32_t i = 0; i < cameraRenderInfos.size(); i++) {
      cameraIDs.push_back(cameraRenderInfos[i]->m_ID);
    }

    // If the camera FPS is not set, find the minimum period for one of the cameras and use that.
    // This assumes that all cameras capture at the same frame rate.
    if (cameraFPS == 0.0 && cameras.size() > 0) {
      cameraFPS = 1.0 / cameras[0].minTriggerPeriod;
    }
    std::cout << "Camera frame rate: " << cameraFPS << " fps" << std::endl;

    // Initialize the timing information, making an entry for each camera.  We make sure that there is
    // the maximum camera ID so that we can use the camera ID as an index.
    uint32_t maxID = 0;
    for (auto ID: cameraIDs) {
      if (ID > maxID) {
        maxID = ID;
      }
    }
    g_timingInfo.SetNumCameras(maxID);

    // Separate the cameras into two groups: those with IDs less than 22 are visible cameras and those
    // with larger ones are depth-estimation cameras.
    for (size_t j = 0; j < cameraRenderInfos.size(); j++) {
      if (cameraRenderInfos[j]->m_ID < 22) {
        g_visibleCameras.push_back(cameraRenderInfos[j]);
      }
      else if (computeDepth) {
        g_depthCameras.push_back(cameraRenderInfos[j]);
      }
    }

    // If we've been asked to do standard-deviation-based auto-ranging, set that up.
    std::shared_ptr<asdp::render::imageStatistics::MeanStdGroup> meanStdGroup;
    std::shared_ptr<RangeEstimator> rangeEstimator = std::make_shared<RangeEstimatorFixed>();
    if (autoRangeStdAbove != 0 || autoRangeStdBelow != 0) {
      // Make a display object that shares textures with the others.
      std::shared_ptr<Display> display = std::make_shared<DisplayTexture>(displayTexture.get());
      // Make a MeanStdGroup object to handle the statistics.
      meanStdGroup = std::make_shared<asdp::render::imageStatistics::MeanStdGroup>(g_visibleCameras,
        display, 1.0/cameraFPS);
      // Make a RangeEstimator object to handle the range.
      rangeEstimator = std::make_shared<RangeEstimatorStdRanges>(meanStdGroup,
        autoRangeStdBelow, autoRangeStdAbove);
    }

    // Construct a depth-estimation object if there are any depth-estimation cameras.
    // There must be sets of two camera pairs for depth estimation.
    if (g_depthCameras.size() > 0) {
      if (g_depthCameras.size() % 2 != 0) {
        std::cerr << "Error: There must be an even number of depth-estimation cameras." << std::endl;
        return 20;
      }
      std::vector< std::array<std::shared_ptr<asdp::render::CameraRenderInfo>, 2> > cameras;
      for (size_t i = 0; i < g_depthCameras.size(); i += 2) {
        std::array<std::shared_ptr<asdp::render::CameraRenderInfo>, 2> pair = { g_depthCameras[i], g_depthCameras[i + 1] };
        cameras.push_back(pair);
      }

      if (!displayTexture->BorrowContext()) {
        std::cerr << "Error borrowing context from displayTexture for DepthEstimator." << std::endl;
        return 100;
      }

      // Initialize GLEW in our context. It is okay to initialize it more than once.
      glewExperimental = true;
      if (glewInit() != GLEW_OK) {
        std::cerr << "Failed to initialize GLEW before DepthTexture" << std::endl;
        return false;
      }
      // Clear any GL error that Glew caused.  Apparently on Non-Windows
      // platforms, this can cause a spurious error 1280.
      glGetError();

      // Determine the range of depths to use for the depth estimater and then construct it.
      std::vector<float> depths(7);
      depths[depths.size()-1] = maxDepth;
      for (int i = depths.size() - 2; i >= 0; i--) {
        depths[i] = depths[i + 1] / 2;
      }
      g_depthEstimator = std::make_shared<DepthEstimator>(cameras, rangeEstimator, poseAdjuster, float(1.0/cameraFPS),
        g_depthCameras[0]->m_resolutionPixels[0] * 2 / 100, g_depthCameras[0]->m_resolutionPixels[1] * 2 / 100,
        depths, depthThreshold);
      std::cout << "Constructed DepthEstimator with " << cameras.size() << " camera pairs." << std::endl;

      // Compute a depth estimate to get all of the machinery set up and GLEW initialized on this thread.
      g_depthEstimator->ComputeDepthEstimate(0);

      if (!displayTexture->ReturnContext()) {
        std::cerr << "Error returning context to displayTexture for DepthEstimator." << std::endl;
        return 101;
      }
    }

    // Configure an event structure to handle callbacks for the display windows.
    std::shared_ptr<EventHandlers> handlers = std::make_shared<EventHandlers>();
    handlers->ChangePlayPause = ChangePlayPause;
    if (g_depthEstimator) {
      handlers->ComputeDepth = ComputeDepth;
    }
    handlers->SetToRenderDepth = ChangeDepthRendering;

    // Construct one or more Display objects to render the cameras.  They all share objects with the texture Display.
    std::vector<std::shared_ptr<Display>> displays;
    std::vector<GLuint> toneMapTextures;  ///< Stores these for later deletion.

    //======================================
    // Added by Sang Yoon to determine overview window and detail view window.
    // Where the number of displays is greater than 1, the widest window is considered as an overview window,
    // and the narrowest window is considered as a detail view window.
    int overview_displayID = -1; // display ID of overview window
    int detailed_view_displayID = -1; // display ID of detailed view window

    if (displayInfos.size() > 1) {
        float widest_hFOV = 0.0f;
        float narrowest_hFOV = 360.0f;
        for (size_t i = 0; i < displayInfos.size(); i++) {
            if (displayInfos[i].hFOV >= widest_hFOV) {
                widest_hFOV = displayInfos[i].hFOV;
                overview_displayID = i;
            }
            if (displayInfos[i].hFOV < narrowest_hFOV || displayInfos[i].useOpenXR) {
                narrowest_hFOV = displayInfos[i].hFOV;
                detailed_view_displayID = i;
            }
        }
        displayInfos[overview_displayID].overview = true;
        displayInfos[detailed_view_displayID].detailed_view = true;
    }
    //======================================

    for (size_t i = 0; i < displayInfos.size(); i++) {

      // Construct a Tone Map texture to use for rendering the cameras.
      if (!displayTexture->BorrowContext()) {
        std::cerr << "Error borrowing context from displayTexture for ToneMap." << std::endl;
        return 21;
      }
      GLuint toneMapTexture = displayInfos[i].toneMap.GenerateTexture();
      toneMapTextures.push_back(toneMapTexture);
      if (toneMapTexture == 0) {
        std::cerr << "Error generating texture for ToneMap." << std::endl;
        return 22;
      }
      if (!displayTexture->ReturnContext()) {
        std::cerr << "Error returning context to displayTexture for ToneMap." << std::endl;
        return 23;
      }

      // Construct a Composite object to render the visible cameras.  We need a separate Composite per Display so that each
      // can cache consistent camera images for the whole frame while views are being rendered.
      // Two displays cannot share a SetupRenderFrame() call because they may have different frame rates.
      std::shared_ptr<Timer> timer;
      status = client->GetTimer(timer);
      if (status != OKAY) {
        std::cerr << "Failed to get timer: " << ErrorMessage(status) << std::endl;
        return 24;
      }
      uint32_t renderOffsetMicroseconds = 0;
      if (replayStreamID != 0) {
        // Set up to run 1.5 frames behind the curre
        // nt time, which empirically was much
        // smoother than a single frame behind and slightly smoother than 2 frames.
        renderOffsetMicroseconds = 0.5 * (1000000 / cameraFPS); //1.5
      }
      g_composite = std::make_shared<CompositeCameras>(
        g_visibleCameras, toneMapTexture, poseAdjuster, Time(1/cameraFPS),
        renderOffsetMicroseconds,
        Time(0, 1000000 / displayInfos[i].fps), (i == 0) ? (&g_timingInfo) : nullptr,
        rangeEstimator);

      //======================================
      // Added by Sang Yoon to just pass the status of enabling the cylindrical projection (true or false) from DisplayInfos[i] to composite.
      // Note that the cylinderical projection is processed in Composite Submodule.
      g_composite->m_CP_enabled = displayInfos[i].enableCP;
      //======================================

      //======================================
      // Added by Sang Yoon to just pass the status of overview and detailed view for the current display to composite.
      // Note that the overview and detailed view are handled in Composite Submodule.
      g_composite->m_overview = displayInfos[i].overview;
      g_composite->m_detailed_view = displayInfos[i].detailed_view;
      //======================================

      // Only time the first listed display, to avoid race conditions
      if (displayInfos[i].useOpenXR) {
        displays.push_back(std::make_shared<DisplayOpenXR>(g_composite, displayTexture.get(),
          client, triggerID, triggerAheadMicroseconds, depthAheadMicroseconds, 2500, 1, handlers, nullptr,
          (i == 0) ? (&g_timingInfo) : nullptr, replayStreamID != 0));
      } else if (!displayInfos[i].XSightNIC.empty()) {
        displays.push_back(std::make_shared<DisplayXSight>(displayInfos[i].XSightNIC, g_composite, displayTexture.get(),
          client, triggerID, triggerAheadMicroseconds,
          depthAheadMicroseconds,
          2500,
          handlers, nullptr,
          (i == 0) ? (&g_timingInfo) : nullptr, replayStreamID != 0
        )
        );
      } else {
        displays.push_back(std::make_shared<DisplayWindow>("ASDP Render Module " + std::to_string(i),
          g_composite, client, triggerID, triggerAheadMicroseconds, depthAheadMicroseconds, displayInfos[i].fps, 2500,
          displayInfos[i].width, displayInfos[i].height,
          displayInfos[i].hFOV, displayInfos[i].joystick, displayTexture.get(),
          displayInfos[i].fullScreen, displayInfos[i].fullScreenDisplay, false, handlers, nullptr,
          (i == 0) ? (&g_timingInfo) : nullptr, replayStreamID != 0));
      }
      if (displays.back()->GetStatus() != "") {
        std::cerr << "Error constructing Display " << i << ": " << displays.back()->GetStatus() << std::endl;
        displays.clear();
        return 25;
      }
    }

    // Construct shared pointers to the data structures that we'll need to do rendering, with
    // custom destructors that will clean up when the shared_ptr is destroyed.
    std::atomic<bool> done(false);
    std::vector< std::shared_ptr<PinnedBufferPool> > cpuPinnedImageBuffers;
    std::vector< std::shared_ptr<GPUBufferPool> > gpuImageBuffers;
    std::vector< std::shared_ptr<cudaStream_t> > streams;
    std::vector< std::shared_ptr<ReceiverUDP> > UDPReceivers;
    for (size_t i = 0; i < cameras.size(); i++) {
      try {
        // Preload pinned memory buffers for the CPU.
        cpuPinnedImageBuffers.push_back(std::make_shared<PinnedBufferPool>(cameras[i].width* cameras[i].height * sizeof(uint16_t), 5));

        // Preload pinned memory buffers for the GPU.
        gpuImageBuffers.push_back(std::make_shared<GPUBufferPool>(cameras[i].width* cameras[i].height * sizeof(uint16_t), 5));
      } catch (std::exception &e) {
        std::cerr << "Error creating buffer pools: " << e.what() << std::endl;
        return 26;
      }

      // Create a stream for the GPU to use.
      cudaStream_t* streamPtr = new cudaStream_t;
      cudaStreamCreate(streamPtr);
      streams.push_back(std::shared_ptr<cudaStream_t>(streamPtr,
        [](cudaStream_t* ptr) { cudaStreamDestroy(*ptr); delete ptr; }
      ));

      // Create a UDP receiver for the camera.
      std::shared_ptr<ReceiverUDP> receiverUDP = std::make_shared<ReceiverUDP>(ip_address);
      if (receiverUDP->GetConstructorStatus() != OKAY) {
        std::cerr << "Error constructing ReceiverUDP: " << ErrorMessage(receiverUDP->GetConstructorStatus()) << std::endl;
        return 27;
      }
      UDPReceivers.push_back(receiverUDP);
    }

    // Make the queues to pass data between the receiver and texture threads, one for each texture thread.
    // The cameras will be spread among the threads in a round-robin fashion.
    std::vector< std::shared_ptr< SpinFreeQueue< std::shared_ptr<DataToSendToGPU> > > > dataQueues;
    for (size_t i = 0; i < NUM_TEXTURE_THREADS; i++) {
      dataQueues.push_back(std::make_shared< SpinFreeQueue< std::shared_ptr<DataToSendToGPU> > >());
    }

    // Launch the threads to copy data to the GPU, each having its own queue.
    std::vector<std::thread> copyDataToGPUThread;
    for (size_t i = 0; i < NUM_TEXTURE_THREADS; i++) {
      copyDataToGPUThread.push_back(std::thread(CopyDataToTextures, cameras[0].width, cameras[0].height, std::ref(done),
        dataQueues[i], lineBatchesPerGPUSend, displayTextures[i], std::ref(g_timingInfo.cameras)));
    }

    // Launch the data receiving threads, hooking them together using the queues and passing the texture OpenGL
    // context to it.  Round-robin the data queues among the cameras.
    std::vector<std::thread> receiveDataThreads;
    for (size_t i = 0; i < cameras.size(); i++) {
      receiveDataThreads.push_back(std::thread(ReceiveDataThread, std::ref(*UDPReceivers[i]), 9000,
        std::ref(done), cpuPinnedImageBuffers[i], gpuImageBuffers[i], streams[i], cameraRenderInfos[i]->m_imageQueue,
        dataQueues[i % NUM_TEXTURE_THREADS],
        std::ref(g_timingInfo.cameras[i].frameBeginTimes), std::ref(g_timingInfo.cameras[i].frameEndTimes)));
    }

    // Ask for streaming pose and temperature data.
    if (doStreamPoses && hasPoses) {
      std::cout << "Requesting pose data." << std::endl;
      status = client->SendCommandPacket(CommandPacketStreamPoses());
      if (status != OKAY) {
        std::cerr << "Failed to request pose data: " << ErrorMessage(status) << std::endl;
        return 28;
      }
    }
    if (hasTemperatures) {
      std::cout << "Requesting temperature data." << std::endl;
      status = client->SendCommandPacket(CommandPacketStreamTemperatures());
      if (status != OKAY) {
        std::cerr << "Failed to request temperature data: " << ErrorMessage(status) << std::endl;
        return 29;
      }
    }

    // Request streaming on the cameras at their maximum rates from their associated ID.
    std::cout << "Streaming every " << frameStride << " frames from " << cameraIDs.size() << " cameras" << std::endl;
    for (size_t i = 0; i < cameras.size(); i++) {
      uint32_t camID = cameraIDs[i];
      CameraInfo &camera = cameras[i];

      TriggerInfo ti;
      ti.ID = camera.trigger;
      ti.mode = 3;
      ti.period = 1/cameraFPS;
      ti.offset = 0;
      ti.trackingFactor = 0.005;
      ti.externalID = camera.trigger;
      status = client->SendCommandPacket(CommandPacketConfigureTrigger(ti));
      if (status != OKAY) {
        std::cerr << "Failed to configure trigger: " << ErrorMessage(status) << std::endl;
        return 30;
      }
      std::cout << std::setprecision(10) << "  Configured trigger for camera " << camID << " with period " << ti.period << " seconds" << std::endl;

      // Request the camera to stream full-frame images once every frameStride frames.
      uint16_t port;
      status = UDPReceivers[i]->GetPort(port);
      if (status != OKAY) {
        std::cerr << "Failed to get port: " << ErrorMessage(status) << std::endl;
        return 31;
      }
      StreamEndpoint endpoint(ip_address, port);
      SubregionDescription region;
      region.cameraID = camID;
      region.skipFrames = frameStride - 1;
      region.startTimeSeconds = 0;
      region.startTimeMicroseconds = 0;
      region.left = 0;
      region.top = 0;
      region.right = camera.width - 1;
      region.bottom = camera.height - 1;
      status = client->SendCommandPacket(CommandPacketStreamSubregion(endpoint, region));
      if (status != OKAY) {
        std::cerr << "Failed to stream images: " << ErrorMessage(status) << std::endl;
        return 32;
      }
    }

    // If we've been asked to replay a stream, then send a request to do this.
    if (replayStreamID) {
      if (!hasStorage) {
        std::cerr << "Error: Storage API not available when replay requested." << std::endl;
        return 33;
      }
      std::cout << "Requesting replay of stream " << replayStreamID << std::endl;
      // Set the initial time to be above zero so that we never predict backwards to negative time.
      status = client->SendCommandPacket(CommandPacketStartReplay(replayStreamID, Time(10,0)));
      if (status != OKAY) {
        std::cerr << "Failed to start replay: " << ErrorMessage(status) << std::endl;
        return 34;
      }
    }

    // Get a shared pointer to the timer so that we can use it to convert times, and can adjust
    // it based on sync events from the server.
    std::shared_ptr<Timer> timer;
    status = client->GetTimer(timer);
    if (status != OKAY) {
      std::cerr << "Failed to get timer: " << ErrorMessage(status) << std::endl;
      return 35;
    }

    // Create a ClockSynchronizer that will manage adjusting the timer based on clock-sync messages.
    std::shared_ptr<ClockSynchronizer> clockSync = std::make_shared<ClockSynchronizer>(timer);

    // Keeps track of when we were paused so we can adjust our clock synchronization when resumed.
    Time pausedTime = {};

    // Render frames until someone has marked us to be done.
    bool nowPaused = false;
    bool replayDone = false;
    start = std::chrono::steady_clock::now();
    while (!done) {

      // Receive and handle any message from the server, waiting at most 100ms for a
      // new packet before looping back around.
      std::shared_ptr<StreamPacket> response;
      size_t offset = 0;
      Status status = receiver->ReceiveStreamPacket(0.1, response, offset);
      if (status == OKAY) {
        status = HandleStreamPacket(response, clockSync, poseAdjuster, replayDone, displays, timer, pausedTime);
        if (status != OKAY) {
          std::cerr << "Error handling stream packet: " << ErrorMessage(status) << std::endl;
          done = true;
        }
      } else if (status != TIMEOUT) {
        std::cerr << "Error receiving stream packet: " << ErrorMessage(status) << std::endl;
        done = true;
      }

      // If we've been asked to loop replays and replay is done, request a new replay with the offset
      // at the current time.
      if (replayDone && loopReplay) {
        std::cout << "Replay done, requesting new replay." << std::endl;
        Time nowTime;
        status = timer->GetCoreTime(nowTime);
        if (status != OKAY) {
          std::cerr << "Failed to get time: " << ErrorMessage(status) << std::endl;
          done = true;
        }
        status = client->SendCommandPacket(CommandPacketStartReplay(replayStreamID, nowTime));
        if (status != OKAY) {
          std::cerr << "Failed to start replay: " << ErrorMessage(status) << std::endl;
          done = true;
        }
        replayDone = false;
      }

      // If all of our Displays have been closed (or are broken), then we're done.
      bool allClosed = true;
      for (auto& display : displays) {
        if (display->GetStatus() == "") {
          allClosed = false;
          break;
        }
      }
      if (allClosed) {
        done = true;
      }

      // If our state of play/pause has switched and we're pausing or replaying, send a command to the server.
      if (replayStreamID) {
        if (nowPaused != g_paused) {
          if (g_paused) {
            status = client->SendCommandPacket(CommandPacketPauseReplay());
          } else {
            status = client->SendCommandPacket(CommandPacketResumeReplay());
          }
          nowPaused = g_paused;
        }
      }
    }

    // Set done and wait for all of our GPU data threads to join.
    done = true;
    for (auto& thread : copyDataToGPUThread) {
      if (thread.joinable()) {
        thread.join();
      }
    }

    // Destroy our client
    client.reset();

    // Clear all remaining data from the queues now that the receivers are done.
    // All of the receiving threads will also delete this before they exit, which will remove all of the
    // references and push their buffers back onto their empty queues.
    for (auto& queue : dataQueues) {
      queue.reset();
    }

    // Now that all of the buffers have been returned to the buffer queue, join our receive-data threads.
    for (auto& thread : receiveDataThreads) {
      if (thread.joinable()) {
        thread.join();
      }
    }

    // Now borrow the context from the displayTexture so that we can delete the textures.
    if (!displayTexture->BorrowContext()) {
      std::cerr << "Error borrowing context from displayTexture." << std::endl;
      return 36;
    }
    cameraRenderInfos.clear();

    glDeleteTextures(toneMapTextures.size(), toneMapTextures.data());
    if (!displayTexture->ReturnContext()) {
      std::cerr << "Error returning context to displayTexture." << std::endl;
      return 37;
    }

    // If we've been asked to dump the timing information, do so.
    if (!dumpTimingFileName.empty()) {

      // Find the maximum number of entries in any of the timing vectors.
      size_t maxEntries = 0;
      if (g_timingInfo.renderStartTimes.size() > maxEntries) {
        maxEntries = g_timingInfo.renderStartTimes.size();
      }
      if (g_timingInfo.renderSubmitTimes.size() > maxEntries) {
        maxEntries = g_timingInfo.renderSubmitTimes.size();
      }
      if (g_timingInfo.depthStartTimes.size() > maxEntries) {
        maxEntries = g_timingInfo.depthStartTimes.size();
      }
      if (g_timingInfo.depthEndTimes.size() > maxEntries) {
        maxEntries = g_timingInfo.depthEndTimes.size();
      }
      for (size_t i = 0; i < g_timingInfo.cameras.size(); i++) {
        if (g_timingInfo.cameras[i].frameBeginTimes.size() > maxEntries) {
          maxEntries = g_timingInfo.cameras[i].frameBeginTimes.size();
        }
        if (g_timingInfo.cameras[i].frameEndTimes.size() > maxEntries) {
          maxEntries = g_timingInfo.cameras[i].frameEndTimes.size();
        }
        if (g_timingInfo.cameras[i].textureTimes.size() > maxEntries) {
          maxEntries = g_timingInfo.cameras[i].textureTimes.size();
        }
      }

      //==================================================================================================
      // Write the raw file.
      std::string rawTimingFileName = dumpTimingFileName + ".csv";
      std::ofstream dumpTimingFile(rawTimingFileName);
      std::cout << "Dumping " << maxEntries << " raw timing information to " << rawTimingFileName << std::endl;
      dumpTimingFile << "Depth Start,Depth End,Render start,Render submit";
      for (size_t i = 0; i < g_timingInfo.cameras.size(); i++) {
        dumpTimingFile << ",Camera " << i+1 << " frame begin,Camera " << i+1
          << " frame end,Camera " << i+1 << " texture complete,Camera " << i+1
          << " center time seconds, Camera " << i+1 << " center time microseconds";
      }
      dumpTimingFile << std::endl;
      dumpTimingFile << std::setprecision(20);
      for (size_t i = 0; i < maxEntries; i++) {
        if (i < g_timingInfo.depthStartTimes.size()) {
          dumpTimingFile << TimeIntervalToStringMilliseconds(g_timingInfo.depthStartTimes[i] - g_timingInfo.startTime);
        }
        dumpTimingFile << ",";
        if (i < g_timingInfo.depthEndTimes.size()) {
          dumpTimingFile << TimeIntervalToStringMilliseconds(g_timingInfo.depthEndTimes[i] - g_timingInfo.startTime);
        }
        dumpTimingFile << ",";
        if (i < g_timingInfo.renderStartTimes.size()) {
          dumpTimingFile << TimeIntervalToStringMilliseconds(g_timingInfo.renderStartTimes[i] - g_timingInfo.startTime);
        }
        dumpTimingFile << ",";
        if (i < g_timingInfo.renderSubmitTimes.size()) {
          dumpTimingFile << TimeIntervalToStringMilliseconds(g_timingInfo.renderSubmitTimes[i] - g_timingInfo.startTime);
        }
        // No comma here; we'll append them with the following
        for (size_t j = 0; j < g_timingInfo.cameras.size(); j++) {
          dumpTimingFile << ",";
          if (i < g_timingInfo.cameras[j].frameBeginTimes.size()) {
            dumpTimingFile << TimeIntervalToStringMilliseconds(g_timingInfo.cameras[j].frameBeginTimes[i] - g_timingInfo.startTime);
          }
          dumpTimingFile << ",";
          if (i < g_timingInfo.cameras[j].frameEndTimes.size()) {
            dumpTimingFile << TimeIntervalToStringMilliseconds(g_timingInfo.cameras[j].frameEndTimes[i] - g_timingInfo.startTime);
          }
          dumpTimingFile << ",";
          if (i < g_timingInfo.cameras[j].textureTimes.size()) {
            dumpTimingFile << TimeIntervalToStringMilliseconds(g_timingInfo.cameras[j].textureTimes[i] - g_timingInfo.startTime);
          }
          if (i < g_timingInfo.cameras[j].centerRenderTimes.size()) {
            dumpTimingFile << ",";
            dumpTimingFile << std::to_string(g_timingInfo.cameras[j].centerRenderTimes[i].seconds);
            dumpTimingFile << ",";
            dumpTimingFile << std::to_string(g_timingInfo.cameras[j].centerRenderTimes[i].microseconds);
          } else {
            dumpTimingFile << ",,";
          }
        }
        dumpTimingFile << std::endl;
      }
      dumpTimingFile.close();

      //==================================================================================================
      // Write the intervals file.
      std::string intervalTimingFileName = dumpTimingFileName + "_intervals.csv";
      std::ofstream intervalTimingFile(intervalTimingFileName);
      std::cout << "Dumping " << maxEntries-1 << " interval timing information to " << intervalTimingFileName << std::endl;
      intervalTimingFile << "Depth start interval,Depth end interval,Render start interval,Render submit interval";
      for (size_t i = 0; i < g_timingInfo.cameras.size(); i++) {
        intervalTimingFile << ",Camera " << i+1 << " frame begin interval, " << i+1 << " frame end interval,Camera"
          << i+1 << " texture complete interval,Camera " << i+1 << " center time interval";
      }
      intervalTimingFile << std::endl;
      intervalTimingFile << std::setprecision(20);
      for (size_t i = 1; i < maxEntries; i++) {
        if (i < g_timingInfo.depthStartTimes.size()) {
          intervalTimingFile << TimeIntervalToStringMilliseconds(g_timingInfo.depthStartTimes[i] - g_timingInfo.depthStartTimes[i - 1]);
        }
        intervalTimingFile << ",";
        if (i < g_timingInfo.depthEndTimes.size()) {
          intervalTimingFile << TimeIntervalToStringMilliseconds(g_timingInfo.depthEndTimes[i] - g_timingInfo.depthEndTimes[i - 1]);
        }
        intervalTimingFile << ",";
        if (i < g_timingInfo.renderStartTimes.size()) {
          intervalTimingFile << TimeIntervalToStringMilliseconds(g_timingInfo.renderStartTimes[i] - g_timingInfo.renderStartTimes[i - 1]);
        }
        intervalTimingFile << ",";
        if (i < g_timingInfo.renderSubmitTimes.size()) {
          intervalTimingFile << TimeIntervalToStringMilliseconds(g_timingInfo.renderSubmitTimes[i] - g_timingInfo.renderSubmitTimes[i - 1]);
        }
        // No comma here; we'll append them with the following
        for (size_t j = 0; j < g_timingInfo.cameras.size(); j++) {
          intervalTimingFile << ",";
          if (i < g_timingInfo.cameras[j].frameBeginTimes.size()) {
            intervalTimingFile << TimeIntervalToStringMilliseconds(g_timingInfo.cameras[j].frameBeginTimes[i] - g_timingInfo.cameras[j].frameBeginTimes[i - 1]);
          }
          intervalTimingFile << ",";
          if (i < g_timingInfo.cameras[j].frameEndTimes.size()) {
            intervalTimingFile << TimeIntervalToStringMilliseconds(g_timingInfo.cameras[j].frameEndTimes[i] - g_timingInfo.cameras[j].frameEndTimes[i - 1]);
          }
          intervalTimingFile << ",";
          if (i < g_timingInfo.cameras[j].textureTimes.size()) {
            intervalTimingFile << TimeIntervalToStringMilliseconds(g_timingInfo.cameras[j].textureTimes[i] - g_timingInfo.cameras[j].textureTimes[i - 1]);
          }
          intervalTimingFile << ",";
          if (i < g_timingInfo.cameras[j].centerRenderTimes.size()) {
            Time diff = g_timingInfo.cameras[j].centerRenderTimes[i] - g_timingInfo.cameras[j].centerRenderTimes[i - 1];
            double diffms = diff.seconds * 1000.0 + diff.microseconds / 1000.0;
            intervalTimingFile << std::to_string(diffms);
          }
        }
        intervalTimingFile << std::endl;
      }
      intervalTimingFile.close();

      //==================================================================================================
      // Write a summary file that describes the min and max behavior for each frame that has camera timing info.
      std::string summaryTimingFileName = dumpTimingFileName + "_summary.csv";
      std::ofstream summaryTimingFile(summaryTimingFileName);
      std::cout << "Dumping summary timing information to " << summaryTimingFileName << std::endl;
      summaryTimingFile << "Depth start to depth end,Depth end to render start,Render start to submit,Render start interval,Min camera end to render"
        << ",Max camera end to render, Min camera texture to render, Max camera texture to render"
        << ",Min center interval,Max center interval" << std::endl;
      summaryTimingFile << std::setprecision(20);
      for (size_t i = 1; i < g_timingInfo.renderStartTimes.size(); i++) {
        if (i < g_timingInfo.depthStartTimes.size() && i < g_timingInfo.depthEndTimes.size()) {
          summaryTimingFile << TimeIntervalToStringMilliseconds(g_timingInfo.depthEndTimes[i] - g_timingInfo.depthStartTimes[i]);
        }
        summaryTimingFile << ",";
        if (i < g_timingInfo.depthEndTimes.size() && i < g_timingInfo.renderStartTimes.size()) {
          summaryTimingFile << TimeIntervalToStringMilliseconds(g_timingInfo.renderStartTimes[i] - g_timingInfo.depthEndTimes[i]);
        }
        summaryTimingFile << ",";
        if (i < g_timingInfo.renderSubmitTimes.size()) {
          summaryTimingFile << TimeIntervalToStringMilliseconds(g_timingInfo.renderSubmitTimes[i] - g_timingInfo.renderStartTimes[i]);
        }
        summaryTimingFile << ",";
        summaryTimingFile << TimeIntervalToStringMilliseconds(g_timingInfo.renderStartTimes[i] - g_timingInfo.renderStartTimes[i - 1]);
        summaryTimingFile << ",";

        // Find the largest camera end-frame time less than the render start time for each camera, and then find the
        // minimum and maximum of these.
        std::chrono::steady_clock::time_point minTime = std::chrono::steady_clock::time_point::max();
        std::chrono::steady_clock::time_point maxTime = std::chrono::steady_clock::time_point::min();
        for (size_t j = 0; j < g_timingInfo.cameras.size(); j++) {
          std::chrono::steady_clock::time_point t = LargestTimeLessThan(g_timingInfo.renderStartTimes[i], g_timingInfo.cameras[j].frameEndTimes);
          minTime = std::min(minTime, t);
          maxTime = std::max(maxTime, t);
        }
        if (minTime == std::chrono::steady_clock::time_point::min()) {
          summaryTimingFile << ",,";
        } else {
          summaryTimingFile << TimeIntervalToStringMilliseconds(g_timingInfo.renderStartTimes[i] - maxTime) << ",";
          summaryTimingFile << TimeIntervalToStringMilliseconds(g_timingInfo.renderStartTimes[i] - minTime) << ",";
        }

        // Find the largest texture time less than the render start time for each camera, and then find the
        // minimum and maximum of these.
        minTime = std::chrono::steady_clock::time_point::max();
        maxTime = std::chrono::steady_clock::time_point::min();
        for (size_t j = 0; j < g_timingInfo.cameras.size(); j++) {
          std::chrono::steady_clock::time_point t = LargestTimeLessThan(g_timingInfo.renderStartTimes[i], g_timingInfo.cameras[j].textureTimes);
          minTime = std::min(minTime, t);
          maxTime = std::max(maxTime, t);
        }
        if (minTime == std::chrono::steady_clock::time_point::min()) {
          summaryTimingFile << "," << ",";
        } else {
          summaryTimingFile << TimeIntervalToStringMilliseconds(g_timingInfo.renderStartTimes[i] - maxTime) << ",";
          summaryTimingFile << TimeIntervalToStringMilliseconds(g_timingInfo.renderStartTimes[i] - minTime) << ",";
        }

        // Find the minimum and maximum center render intervals.
        Time minCenterInterval = { 1000000, 0 };
        Time maxCenterInterval = { 0, 0 };
        for (size_t j = 0; j < g_timingInfo.cameras.size(); j++) {
          if (g_timingInfo.cameras[j].centerRenderTimes.size() > i) {
            Time diff = g_timingInfo.cameras[j].centerRenderTimes[i] - g_timingInfo.cameras[j].centerRenderTimes[i - 1];
            if (diff < minCenterInterval) {
              minCenterInterval = diff;
            }
            if (diff > maxCenterInterval) {
              maxCenterInterval = diff;
            }
          }
        }
        summaryTimingFile << std::to_string(minCenterInterval.seconds * 1000.0 + minCenterInterval.microseconds / 1000.0);
        summaryTimingFile << ",";
        summaryTimingFile << std::to_string(maxCenterInterval.seconds * 1000.0 + maxCenterInterval.microseconds / 1000.0);
        summaryTimingFile << std::endl;
      }
      summaryTimingFile.close();
    }
  } // End of block that causes destruction of all objects before returning.

  // Clean up the global objects.
  g_visibleCameras.clear();
  g_depthCameras.clear();
  g_depthEstimator.reset();
  g_composite.reset();

  return 0;
}
