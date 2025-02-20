/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
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
    surf2Dwrite(buffer[x + y * nx], surface, x * sizeof(uint16_t), y);
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
