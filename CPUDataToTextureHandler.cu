/*
 * Copyright (C) 2024-2025: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include "CPUDataToTextureHandler.h"
#include <cuda_gl_interop.h>
#include <iostream>
using namespace asdp;
using namespace asdp::render;

/// @brief CUDA kernel to write a uint16 image to a surface (OpenGL texture).
/// @details The number of blocks in Y is just enough to cover the amount of data that we have
/// to send, with perhaps an overage because we're not writing an even number of blocks of lines.
/// @param surface The surface to write to.
/// @param buffer The buffer containing the uint16 data.  This is a pointer to the beginning of the whole-frame data.
/// @param offy The y offset to apply to the coordinate (added to the y coordinate in pixels).
/// @param nx The total width of the image.
/// @param ny The total height of the image.
__global__ void WriteSurfaceKernel(cudaSurfaceObject_t surface, uint16_t* buffer,
  uint16_t offy, uint16_t nx, uint16_t ny)
{
  uint16_t x = blockIdx.x * blockDim.x + threadIdx.x;
  uint16_t y = offy + blockIdx.y * blockDim.y + threadIdx.y;
  if (x < nx && y < ny) {
    // Write the data to the surface. The x coordinate is in bytes, so we need to multiply by the
    // size of the data type.
    surf2Dwrite(buffer[x + y * nx], surface, x * sizeof(buffer[0]), y);
  }
}

CPUDataToTextureHandler::CPUDataToTextureHandler(
  std::shared_ptr< std::map<GLuint, cudaGraphicsResource*> > texturesToCUDAMap,
  std::shared_ptr<DataToSendToGPU> dataPtr,
  uint16_t width, uint16_t height, uint16_t batchSize,
  float exposure, float gain)
  : m_status(""), m_dataPtr(dataPtr)
  , m_width(width), m_height(height), m_batchSize(batchSize)
  , m_exposure(exposure), m_gain(gain)
  , m_lastLineSent(-1), m_largestLineReceived(0)
  , m_imageData(nullptr), m_resource(nullptr), m_textureData(nullptr), m_surfObj(0)
{
  cudaError_t cudaStatus;

  // Get the texture ID to use for the image data and store it away for use in the destructor.
  m_imageData = m_dataPtr->imageQueuePtr->GetOldestImage();
  if (m_imageData == nullptr) {
    m_status = "Error getting image data from image queue.";
    return;
  }
  unsigned int textureID = m_imageData->texture;

  {
    // Register the OpenGL texture with CUDA if we don't already have it registered.
    auto texture = texturesToCUDAMap->find(textureID);
    if (texture != texturesToCUDAMap->end()) {
      m_resource = texture->second;
    } else {
      cudaStatus = cudaGraphicsGLRegisterImage(&m_resource, textureID, GL_TEXTURE_2D, cudaGraphicsRegisterFlagsSurfaceLoadStore);
      if (cudaStatus != cudaSuccess) {
        m_status = "Failed to register texture: " + std::string(cudaGetErrorString(cudaStatus));
        return;
      }
      (*texturesToCUDAMap)[textureID] = m_resource;
    }

    // Map the texture for writing by CUDA
    cudaGraphicsMapResources(1, &m_resource, *(m_dataPtr->streamPtr));
    cudaStatus = cudaGraphicsSubResourceGetMappedArray(&m_textureData, m_resource, 0, 0);
    if (cudaStatus != cudaSuccess) {
      m_status = "Failed to map texture: " + std::string(cudaGetErrorString(cudaStatus));
      cudaGraphicsUnmapResources(1, &m_resource, *(m_dataPtr->streamPtr));
      (*texturesToCUDAMap)[textureID] = nullptr;
      return;
    }
  }

  // Create a 2D surface object
  cudaResourceDesc resDesc;
  memset(&resDesc, 0, sizeof(resDesc));
  resDesc.resType = cudaResourceTypeArray;
  resDesc.res.array.array = m_textureData;

  cudaStatus = cudaCreateSurfaceObject(&m_surfObj, &resDesc);
  if (cudaStatus != cudaSuccess) {
    m_status = "Failed to create surface object: " + std::string(cudaGetErrorString(cudaStatus));
    cudaGraphicsUnmapResources(1, &m_resource, *(m_dataPtr->streamPtr));
    (*texturesToCUDAMap)[textureID] = nullptr;
    return;
  }
}

CPUDataToTextureHandler::~CPUDataToTextureHandler()
{
  if (m_imageData == nullptr) {
    std::cerr << "CPUDataToTextureHandler::~CPUDataToTextureHandler(): No m_imageData." << std::endl;
    return;
  }

  // Send any unsent data to the GPU.
  std::string ret = SendToGPU();
  if (!ret.empty()) {
    std::cerr << "CPUDataToTextureHandler::~CPUDataToTextureHandler(): Error sending data to GPU: " << ret << std::endl;
  }

  // Set the time on the image data to the average of the begin and end times.
  m_imageData->imageCenterTime = m_centerTime;

  // Set the exposure and gain on the image data.
  m_imageData->exposure = m_exposure;
  m_imageData->gain = m_gain;

  // Ensure that the stream completes (so OpenGL on other threads in other contexts won't race).
  // This may be superfluous because the call to cudaGraphicsUnmapResources() handles this at least
  // for OpenGL work on the current context and thread.  Adding it did not impact either the GPU
  // or CPU resources when streaming 21 cameras.
  cudaStreamSynchronize(*(m_dataPtr->streamPtr));

  // Free up our resources
  cudaDestroySurfaceObject(m_surfObj);
  // As a side effect, this call guarantees that all CUDA work completes before any later-called OpenGL work starts.
  cudaGraphicsUnmapResources(1, &m_resource, *(m_dataPtr->streamPtr));

  // Be sure that everything is registered with OpenGL before putting the texture back into use on another thread.
  // Adding this call fixed a misalignment between cameras where neighbors had different-timed images.
  glFinish();

  // Put the texture back into the image queue so the Composite can use it.
  m_dataPtr->imageQueuePtr->InsertImage(m_imageData);
}

std::string CPUDataToTextureHandler::SetCenterTime(asdp::Time centerTime)
{
  m_centerTime = centerTime;
  return "";
}

std::string CPUDataToTextureHandler::SendToGPU()
{
  // The offset is just past the largest line sent so far.
  int offsetY = m_lastLineSent + 1;
  size_t offset = offsetY * m_width * sizeof(uint16_t);
  unsigned linesToSend = m_largestLineReceived - m_lastLineSent;
  if (linesToSend == 0) {
    return "";
  }

  // Copy the batch to the GPU.
  cudaError_t ret = cudaMemcpyAsync(m_dataPtr->gpuImageBufferPtr.get() + offset, m_dataPtr->cpuImageBufferPtr.get() + offset,
    linesToSend * m_width * sizeof(uint16_t),
    cudaMemcpyHostToDevice, *m_dataPtr->streamPtr);
  if (ret != cudaSuccess) {
    return "CopyDataToGPU: cudaMemcpyAsync() failed: " + std::string(cudaGetErrorString(ret));
  }

  // Run the kernel to write this subset of the data to the texture.
  // Run it on the same stream so that it will wait for the copy to complete before running.
  dim3 dimBlock(128, 8); ///< Using a kernel that is wide but not tall because our batch sizes may be small
  dim3 dimGrid((m_width + dimBlock.x - 1) / dimBlock.x, (linesToSend + dimBlock.y - 1) / dimBlock.y);
  WriteSurfaceKernel << <dimGrid, dimBlock, 0, *(m_dataPtr->streamPtr) >> > (
    m_surfObj, reinterpret_cast<uint16_t*>(m_dataPtr->gpuImageBufferPtr.get()),
    offsetY, m_width, m_height);

  // Record the fact that we've written up through this line.
  m_lastLineSent = m_largestLineReceived;

  return "";
}

std::string CPUDataToTextureHandler::ProcessImageSubset(
  uint16_t left, uint16_t top, uint16_t right, uint16_t bottom)
{
  if (m_imageData == nullptr) {
    return "No m_imageData";
  }

  // Keep track of the largest line received so far.
  m_largestLineReceived = std::max(m_largestLineReceived, bottom);

  // Copy the image data to the GPU if we've completed a chunk of lines, or if we're writing to the last line in the image.
  // We check every line from the top of the region to the bottom and send all unsent lines if it is ever the last line
  // in the region or the frame.  We always send entire lines, even if they have only
  // been partially filled, so that we don't have to keep a mask for each line.
  for (uint16_t line = top; line <= bottom; ++line) {
    if ((line + 1 == m_height) || ((line + 1) % m_batchSize == 0)) {
      std::string ret = SendToGPU();
      if (!ret.empty()) {
        return ret;
      }
      // We sent all of the unsent lines, so we don't need to keep looking.
      break;
    }
  }

  return "";
}

void asdp::render::CopyDataToTextures(uint16_t width, uint16_t height,
  std::atomic<bool>& done,
  std::shared_ptr< SpinFreeQueue< std::shared_ptr<DataToSendToGPU> > > inQueue,
  size_t batchSize, std::shared_ptr<Display> sharedContext,
  std::vector<RenderTimingInfo::camera>& cameraTimings)
{
  // Borrow the context from the shared context so that we can use it to map textures.
  if (!sharedContext->BorrowContext()) {
    std::cerr << "CopyDataToGPU: Error borrowing context from shared context." << std::endl;
    done = true;
    return;
  }

  // Vector of handlers to process the data for each camera.  There will be one handler for each camera,
  // indexed by its ID.  We add to this vector as we get new cameras.
  std::vector< std::shared_ptr<CPUDataToTextureHandler> > handlers;

  // Vector of times for the current frame from each camera, indexed by camera ID.  We add to this vector
  // as we get new cameras.
  std::vector<asdp::Time> frameTimes;

  auto lastPrint = std::chrono::steady_clock::now();

  // Map from Texture ID to cudaGraphicsResource* for the texture data.  This is used by the CPUDataToTextureHandler
  // objects to know which texture to use without having to repeatedly register and unregister it.
  std::shared_ptr< std::map<GLuint, cudaGraphicsResource*> > texturesToCUDAMap =
    std::make_shared< std::map<GLuint, cudaGraphicsResource*> >();

  while (!done) {
    // Once per second, print out the size of the input queue
    if (std::chrono::duration<double>(std::chrono::steady_clock::now() - lastPrint).count() > 5.0) {
      std::cout << "Input queue size: " << inQueue->size() << std::endl;
      lastPrint = std::chrono::steady_clock::now();
    }

    std::shared_ptr<DataToSendToGPU> data;
    // Time out after 10 milliseconds so that we can check the done flag frequently.
    if (inQueue->dequeue(data, std::chrono::milliseconds(10))) {

      // Parse all of the messages in the stream packet, handling each of them in turn.
      for (auto& message : data->messages) {
        if (message.isFrameBegin) {
          // Construct the CPUDataToTextureHandler object to handle the data for this frame and store it in the vector
          // of handlers.  This will be used to process the data as it comes in.  Make more handlers as needed.
          if (message.cameraID >= handlers.size()) {
            handlers.resize(message.cameraID + 1);
          }
          handlers[message.cameraID] = std::make_shared<CPUDataToTextureHandler>(texturesToCUDAMap, data,
            message.width, message.height, static_cast<uint16_t>(batchSize), message.exposure, message.gain);
          if (!handlers[message.cameraID]->GetStatus().empty()) {
            std::cerr << "Error creating CPUDataToTextureHandler: " << handlers[message.cameraID]->GetStatus() << std::endl;
            done = true;
            return;
          }

          // Store the initial frame time for this camera.
          if (message.cameraID >= frameTimes.size()) {
            frameTimes.resize(message.cameraID + 1);
          }
          frameTimes[message.cameraID] = message.frameStartTime;
        }

        // Handle the data
        if (message.cameraID >= handlers.size()) {
          std::cerr << "CopyDataToGPU: FRAME_DATA: Error: Camera ID " << message.cameraID << " not found." << std::endl;
          done = true;
          return;
        }
        if (handlers[message.cameraID] == nullptr) {
          std::cerr << "CopyDataToGPU: FRAME_DATA: Warning: Camera ID " << message.cameraID << " frame data without begin." << std::endl;
          break;
        }
        std::string ret = handlers[message.cameraID]->ProcessImageSubset(message.left, message.top, message.right, message.bottom);
        if (!ret.empty()) {
          std::cerr << "Error processing image subset: " << ret << std::endl;
          done = true;
          return;
        }

        if (message.isFrameEnd) {
          // Set the center time for the image data, which is the average of the frame begin and end times.
          if (message.cameraID >= frameTimes.size()) {
            std::cerr << "CopyDataToGPU: FRAME_END: Error: Camera ID " << message.cameraID << " not found in frameTimes." << std::endl;
            done = true;
            return;
          }
          asdp::Time duration = message.time - frameTimes[message.cameraID];
          float durationSeconds = duration.seconds + duration.microseconds / 1.0e6f;
          float halfDurationSeconds = durationSeconds / 2;
          asdp::Time centerTime = frameTimes[message.cameraID] + asdp::Time(halfDurationSeconds);
          std::string ret = handlers[message.cameraID]->SetCenterTime(centerTime);
          if (!ret.empty()) {
            std::cerr << "Error setting center time: " << ret << std::endl;
            done = true;
            return;
          }

          // Done with this frame, so we reset the pointer to delete the handler, which will clean
          // up and push the data to the texture before returning.
          if (message.cameraID >= handlers.size()) {
            std::cerr << "CopyDataToGPU: FRAME_END: Warning: Camera ID " << message.cameraID << " frame end without begin." << std::endl;
            break;
          }
          handlers[message.cameraID].reset();
          if ((cameraTimings.size() > 0) && (message.cameraID <= cameraTimings.size())) {
            cameraTimings[message.cameraID - 1].textureTimes.push_back(std::chrono::steady_clock::now());
          }
        }
      } // End of message summary loop.
    } // End of if we got a message from the queue.
  } // End of while we are not done.

  // Unregister all of our textures from CUDA.
  for (auto& texture : *texturesToCUDAMap) {
    cudaGraphicsUnregisterResource(texture.second);
  }

  // Return the context borrowed from the shared context so that we can use it to map textures.
  if (!sharedContext->ReturnContext()) {
    std::cerr << "CopyDataToGPU: Error return context to shared context." << std::endl;
    done = true;
    return;
  }
}

/// @brief Helper function to pull information from a CONSOLIDATED_FRAME_DATA message.
/// @param message The message to pull the information from.
/// @param cameraID The camera ID that the data is for.
/// @param width The width of the image data.
/// @param height The height of the image data.
/// @param exposure The exposure time for the image data.
/// @param gain The gain for the image data.
/// @return OKAY on success, error status on failure.
static asdp::Status ParseFrameMessage(Message& message, bool &isFrameBegin, bool &isFrameEnd, Time& frameStartTime, Time& time,
  uint32_t& cameraID, uint16_t& width, uint16_t& height, uint16_t& left, uint16_t& top, uint16_t& right, uint16_t& bottom,
  float& exposure, float& gain, uint8_t*& dataPtr)
{
  MessageConsolidatedFrameData frameData(message);
  if (frameData.GetConstructorStatus() != OKAY) {
    return frameData.GetConstructorStatus();
  }
  Status status;
  status = frameData.GetBeginFrameFlag(isFrameBegin);
  if (status != OKAY) {
    return status;
  }
  status = frameData.GetEndFrameFlag(isFrameEnd);
  if (status != OKAY) {
    return status;
  }
  status = frameData.GetFrameStartTime(frameStartTime);
  if (status != OKAY) {
    return status;
  }
  status = frameData.GetTime(time);
  if (status != OKAY) {
    return status;
  }
  status = frameData.GetCameraID(cameraID);
  if (status != OKAY) {
    return status;
  }
  status = frameData.GetSensorWidth(width);
  if (status != OKAY) {
    return status;
  }
  status = frameData.GetSensorHeight(height);
  if (status != OKAY) {
    return status;
  }
  status = frameData.GetLeft(left);
  if (status != OKAY) {
    return status;
  }
  status = frameData.GetTop(top);
  if (status != OKAY) {
    return status;
  }
  status = frameData.GetRight(right);
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
  status = frameData.GetExposure(exposure);
  if (status != OKAY) {
    return status;
  }
  status = frameData.GetGain(gain);
  if (status != OKAY) {
    return status;
  }
  return OKAY;
}

void asdp::render::ReceiveDataThread(ReceiverUDP& receiveSocket, size_t maxBytesPerPacket, std::atomic<bool>& done,
  std::shared_ptr<PinnedBufferPool> cpuImageBuffers, std::shared_ptr<GPUBufferPool> gpuImageBuffers,
  std::shared_ptr<cudaStream_t> streamPtr,
  std::shared_ptr<asdp::render::ImageQueue> imageQueue,
  std::shared_ptr< SpinFreeQueue< std::shared_ptr<DataToSendToGPU> > > outQueue,
  std::vector<std::chrono::steady_clock::time_point>* frameBeginTimes,
  std::vector<std::chrono::steady_clock::time_point>* frameEndTimes)
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
        case CONSOLIDATED_FRAME_DATA:
          {
            // Pull the information from the frame so that we can store the width data for this
            // camera.
            bool isFrameBegin, isFrameEnd;
            Time frameStartTime, time;
            uint32_t cameraID;
            uint16_t width, height;
            uint16_t left, right, top, bottom;
            uint8_t* data;
            float exposure, gain;
            status = ParseFrameMessage(*message, isFrameBegin, isFrameEnd, frameStartTime, time, cameraID, width, height,
              left, top, right, bottom, exposure, gain, data);
            if (OKAY != status) {
              std::cerr << "ReceiveDataThread: ParseFrameMessage() failed: " << ErrorMessage(status) << std::endl;
              done = true;
              return;
            }
            cameraWidth = width;

            if (isFrameBegin) {
              // Log our timing data.
              if (frameBeginTimes) { frameBeginTimes->push_back(std::chrono::steady_clock::now()); }

              // We found a begin frame message, so we can start processing the data.
              waitingForFrameBegin = false;

              // Get a new pinned CPU memory and GPU memory buffer to hold the image data.
              // The old ones will be returned to the pool when the shared pointers are reset.
              try {
                // Do not allocate new buffers if they are depleted; wait for them to be returned.
                cpuImageBufferPtr = cpuImageBuffers->GetBuffer(false);
                gpuImageBufferPtr = gpuImageBuffers->GetBuffer(false);
              }
              catch (std::exception& e) {
                std::cerr << "Error getting buffers: " << e.what() << std::endl;
                done = true;
                return;
              }
            }

            if (waitingForFrameBegin) {
              // We're waiting for the frame begin message, so we ignore this packet that does not have one.
              break;
            }

            // Copy the data to the pinned CPU memory buffer.
            uint16_t regionWidth = right - left + 1;
            size_t padding = (regionWidth % 2 == 0) ? 0 : 1;
            uint16_t regionHeight = bottom - top + 1;
            uint16_t* cpuBuffer16 = reinterpret_cast<uint16_t*>(cpuImageBufferPtr.get());
            uint16_t* data16 = reinterpret_cast<uint16_t*>(data);
            if (cameraWidth != 0) {
              if ((left == 0) && (regionWidth == cameraWidth) && (cameraWidth % 2 == 0)) {
                // If we're copying whole lines and there is no line padding, we can do it all at once.
                memcpy(cpuBuffer16 + top * regionWidth, data16, regionWidth * regionHeight * sizeof(uint16_t));
              }
              else {
                // Otherwise, we must do it line by line.
                for (uint16_t line = top; line <= bottom; ++line) {
                  memcpy(cpuBuffer16 + line * cameraWidth + left, data16 + (line - top) * regionWidth + padding, regionWidth * sizeof(uint16_t));
                }
              }
            }

            // Store the summary
            MessageSummary summary;
            summary.isFrameBegin = isFrameBegin;
            summary.isFrameEnd = isFrameEnd;
            summary.frameStartTime = frameStartTime;
            summary.time = time;
            summary.cameraID = cameraID;
            summary.width = width;
            summary.height = height;
            summary.left = left;
            summary.top = top;
            summary.right = right;
            summary.bottom = bottom;
            summary.exposure = exposure;
            summary.gain = gain;
            messageSummaries.push_back(summary);

            if (isFrameEnd) {
              // Log our timing data.
              if (frameEndTimes) { frameEndTimes->push_back(std::chrono::steady_clock::now()); }

              // We're at the end of a frame, so we need to get a begin-frame message next.
              waitingForFrameBegin = true;
            }
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
