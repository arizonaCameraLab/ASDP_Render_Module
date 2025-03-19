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
        switch (message.messageType) {
        case FRAME_BEGIN:
        {
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
          frameTimes[message.cameraID] = message.time;
        }
        break;

        case FRAME_DATA:
          // Asynchronously send data to the GPU buffer as we get enough data for a minimum block size. 
          // We send the data to the GPU in chunks so that we amortize the per-send cost and reduce
          // latency.  We send asynchronously to enable overlap between data copying and processing
          // (which increases throughput).
        {
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
        }
        break;

        case FRAME_END:
          // Run the kernel and enqueue the result.
        {
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
          cameraTimings[message.cameraID - 1].textureTimes.push_back(std::chrono::steady_clock::now());
        }
        break;

        default:
          // Nothing to do for other message types.
          break;
        } // End of switch on message type.

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
