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
#include <vector>
#include <ASDP_Core_API.h>
#include <ASDP_SpinFreeQueue.hpp>
#include <ASDP_BufferPool.h>
#include <nlohmann/json.hpp>
#include <GL/glew.h>
#include <ToneMap.h>
#include <Composite.h>
#include <Display.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>

using namespace asdp;
using namespace asdp::render;
using json = nlohmann::json;

static std::string VERSION = "1.0.0";

/// @brief The path to the configuration file. Defined in the CMakeLists file.
std::filesystem::path dirPath = CONFIG_FILE_PATH;

/// @brief Structure to hold the data needed to send data to the GPU and run the kernel.
/// @details These will all have been constructed by the thread that is pushing them onto the queue,
/// with custom destructors as needed to free the memory when the shared_ptr is destroyed.
/// This has all of the information needed to get the image all the way into the texture to be rendered.
struct DataToSendToGPU {
  std::shared_ptr<StreamPacket> streamPacketPtr;   ///< The stream packet that was received, which includes camera ID
  std::shared_ptr<unsigned char> cpuImageBufferPtr;///< The pinned-memory buffer on the CPU that holds the image data
  std::shared_ptr<unsigned char> gpuImageBufferPtr;///< The buffer on the GPU that holds the image data
  std::shared_ptr<asdp::render::ImageQueue> imageQueuePtr;///< The image queue holding the textures to store into
  std::shared_ptr<cudaStream_t> streamPtr;         ///< Stream to use to for copy and kernel calls
};

/// @brief Helper function to pull information from a FRAME_BEGIN message.
/// @param message The message to pull the information from.
/// @param cameraID The camera ID that the data is for.
/// @param width The width of the image data.
/// @param height The height of the image data.
/// @return OKAY on success, error status on failure.
asdp::Status ParseFrameBeginMessage(Message &message, uint32_t &cameraID, uint16_t& width, uint16_t& height)
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

/// @brief CUDA kernel to write an offset and scaled copy of a uint16 image to a surface (OpenGL texture).
/// @param surface The surface to write to.
/// @param buffer The buffer containing the uint16 data.  This is a pointer to the beginning of the whole-frame data.
/// @param offy The y offset to apply to the coordinate (added to the y coordinate in pixels).
/// @param nx The total width of the image.
/// @param ny The total height of the image.
/// @param offset The offset to apply to the data (added to the data before scaling, normally negative in the range
/// 0 to -65535).
/// @param scale The scale to apply to the data (multiplied by the data after offsetting, should be positive).
__global__ void WriteScaledOffsetKernel(cudaSurfaceObject_t surface, uint16_t* buffer,
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

/// @brief Class to handle processing of the data from the cameras and sending it to texture.
class CPUDataToTextureHandler {
public:
  /// @brief Constructor to create the handler and set up the resources needed to process a frame.
  /// @details Be sure to call GetStatus() after construction to verify that the constructor succeeded.
  /// @param dataPtr Pointer to the structure that holds the data to send to the GPU and the stream to use.
  /// @param sharedContext The shared context to borrow from when handling textures.
  /// @param width The width of the image data (the whole image).
  /// @param height The height of the image data (the whole image).
  /// @param batchSize The number of lines to send to the GPU at once (the height of the region that will be sent).
  CPUDataToTextureHandler(std::shared_ptr<DataToSendToGPU> dataPtr, std::shared_ptr<Display> sharedContext,
    uint16_t width, uint16_t height, uint16_t batchSize);

  ~CPUDataToTextureHandler();

  /// @brief Process the image subset, sending to GPU memory and then running the kernel to store into texture.
  /// @param offset The offset to add to each pixel value before scaling it during copy (total range 0-65535),
  /// negative values to reduce the pixel count.
  /// @param dataPtr Pointer to the non-pinned CPU data to ingest.  This is not the beginnnig of the image but
  /// rather the beginning of a 16-bit block of tightly-packed data.
  /// @param left The left edge of the region to process.
  /// @param top The top edge of the region to process.
  /// @param width The width of the region to process.
  /// @param height The height of the region to process.
  /// @return Empty string on success, description of error on failure.
  std::string ProcessImageSubset(uint8_t* dataPtr,
    uint16_t left, uint16_t top, uint16_t width, uint16_t height);

  /// @brief Get the status of the constructor.
  /// @return The status of the constructor, empty for good, error message for bad.
  std::string GetStatus() const { return m_status; }

protected:
  std::string m_status;                       ///< The status of the constructor, empty for good, error message for bad

  std::shared_ptr<DataToSendToGPU> m_dataPtr; ///< Information about the structure we're handling
  std::shared_ptr<Display> m_sharedContext;   ///< The shared context to borrow from when handling textures
  uint16_t m_width;                           ///< The width of the image data
  uint16_t m_height;                          ///< The height of the image data
  uint16_t m_batchSize;                       ///< The number of lines to send to the GPU at once
  uint16_t m_lastLineSent;                    ///< The last line sent to the GPU
  uint16_t m_largestLineReceived;             ///< The largest line received so far

  std::shared_ptr<ImageData> m_imageData;     ///< The image data for the texture, including time and texure ID
  cudaGraphicsResource* m_resource;           ///< The CUDA graphics resource for the texture
  cudaArray* m_textureData;                   ///< The CUDA array for the texture
  cudaSurfaceObject_t m_surfObj;              ///< The CUDA surface object for the texture

  /// @brief Function to send all unsent data to the GPU and run the kernel to store it into the texture.
  /// @return Empty string on success, description of error on failure.
  std::string SendToGPU();
};

CPUDataToTextureHandler::CPUDataToTextureHandler(std::shared_ptr<DataToSendToGPU> dataPtr, std::shared_ptr<Display> sharedContext,
  uint16_t width, uint16_t height, uint16_t batchSize)
  : m_status(""), m_dataPtr(dataPtr), m_sharedContext(sharedContext)
  , m_width(width), m_height(height), m_batchSize(batchSize)
  , m_lastLineSent(0), m_largestLineReceived(0)
  , m_imageData(nullptr), m_resource(nullptr), m_textureData(nullptr), m_surfObj(0)
{
  cudaError_t cudaStatus;

  // Get the texture ID to use for the image data and store it away for use in the destructor.
  m_imageData = m_dataPtr->imageQueuePtr->PopOldestImage();
  if (m_imageData == nullptr) {
    m_status = "Error getting image data from image queue.";
    return;
  }
  unsigned int textureID = m_imageData->texture;

  // Borrow the context from the shared context so that we can use it to map textures.
  if (!m_sharedContext->BorrowContext()) {
    m_status = "Error borrowing context from shared context.";
    return;
  }

  {
    // Register the OpenGL texture with CUDA
    cudaStatus = cudaGraphicsGLRegisterImage(&m_resource, textureID, GL_TEXTURE_2D, cudaGraphicsRegisterFlagsSurfaceLoadStore);
    if (cudaStatus != cudaSuccess) {
      m_status = "Failed to register texture: " + std::string(cudaGetErrorString(cudaStatus));
      return;
    }

    // Map the texture for writing by CUDA
    cudaGraphicsMapResources(1, &m_resource, 0);
    cudaStatus = cudaGraphicsSubResourceGetMappedArray(&m_textureData, m_resource, 0, 0);
    if (cudaStatus != cudaSuccess) {
      m_status = "Failed to map texture: " + std::string(cudaGetErrorString(cudaStatus));
      cudaGraphicsUnregisterResource(m_resource);
      return;
    }
  }

  // Return the context to the shared context since we don't need it for the rest of the processing.
  if (!m_sharedContext->ReturnContext()) {
    m_status = "CopyDataToGPU: Error returning context to shared context.";
    return;
  }

  // Create a 2D surface object
  cudaResourceDesc resDesc;
  memset(&resDesc, 0, sizeof(resDesc));
  resDesc.resType = cudaResourceTypeArray;
  resDesc.res.array.array = m_textureData;

  cudaStatus = cudaCreateSurfaceObject(&m_surfObj, &resDesc);
  if (cudaStatus != cudaSuccess) {
    m_status = "Failed to create surface object: " + std::string(cudaGetErrorString(cudaStatus));
    cudaGraphicsUnmapResources(1, &m_resource, 0);
    cudaGraphicsUnregisterResource(m_resource);
    return;
  }
}

CPUDataToTextureHandler::~CPUDataToTextureHandler()
{
  // Send any unsent data to the GPU.
  std::string ret = SendToGPU();
  if (!ret.empty()) {
    std::cerr << "CPUDataToTextureHandler::~CPUDataToTextureHandler(): Error sending data to GPU: " << ret << std::endl;
  }

  // Wait for the operations on this stream to complete
  cudaEvent_t event;
  cudaEventCreate(&event);
  cudaEventRecord(event, *(m_dataPtr->streamPtr));
  cudaEventSynchronize(event);
  cudaEventDestroy(event);

  // Free up our resources
  cudaDestroySurfaceObject(m_surfObj);
  cudaGraphicsUnmapResources(1, &m_resource, 0);
  cudaGraphicsUnregisterResource(m_resource);

  // Put the texture back into the image queue as the newest image so the Composite will use it.
  m_dataPtr->imageQueuePtr->AddNewestImage(m_imageData);
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

  // Wait for the copy to complete before we start the kernel
  cudaEvent_t event;
  cudaEventCreate(&event);
  cudaEventRecord(event, *(m_dataPtr->streamPtr));
  cudaEventSynchronize(event);
  cudaEventDestroy(event);

  // Run the kernel to write this subset of the data to the texture, adding offset and scale.
  dim3 dimBlock(128, 8); ///< Using a kernel that is wide but not tall because our batch sizes may be small
  dim3 dimGrid((m_width + dimBlock.x - 1) / dimBlock.x, (linesToSend + dimBlock.y - 1) / dimBlock.y);
  WriteScaledOffsetKernel << <dimGrid, dimBlock >> > (m_surfObj, reinterpret_cast<uint16_t*>(m_dataPtr->gpuImageBufferPtr.get()),
    offsetY, m_width, m_height);

  // Record the fact that we've written up through this line.
  m_lastLineSent = m_largestLineReceived;

  return "";
}

std::string CPUDataToTextureHandler::ProcessImageSubset(uint8_t* dataPtr,
  uint16_t left, uint16_t top, uint16_t right, uint16_t bottom)
{
  // Keep track of the largest line received so far.
  m_largestLineReceived = std::max(m_largestLineReceived, bottom);

  // Copy the image data to the GPU if we've completed a chunk of lines, or if we're writing to the last line in the image.
  // We check every line from the top of the region to the bottom and send all unsent lines if it is ever the last line
  // in the region or the frame.  We always send entire lines, even if they have only
  // been partially filled, so that we don't have to keep a mask for each line.
  for (uint16_t line = top; line <= bottom; ++line) {
    if ( (line + 1 == m_height) || ((line + 1) % m_batchSize == 0) ) {
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

/// @brief Function to copy data to the GPU and store it into the appropriate textures.
/// It must create and record an event after all operations are complete.  All operations must be
/// done on the stream that is passed in and they must all be asynchronous.  There is a single
/// thread to handle all cameras; it handles all cameras, using different CUDA streams to overlap the operations.
/// To be able to map textures, it must have an OpenGL context whose objects are shared with the Display submodule that
/// will be rendering the images.
/// @param width The width of the image data.
/// @param height The height of the image data.
/// @param done A flag that is set to true when the program is done.
/// @param inQueue The queue that we receive requests on.
/// @param batchSize The number of lines to send to the GPU at once.  This is tuned to trade off latency
/// for throughput, and it should be set to a value that is large enough to amortize the cost of sending
/// data to the GPU, but small enough to keep latency low.  The value of 16 is a good starting point.
/// @param sharedContext The Display object that shares the OpenGL context with the rendering Display.
static void CopyDataToTextures(uint16_t width, uint16_t height,
  std::atomic<bool>& done,
  std::shared_ptr< SpinFreeQueue< std::shared_ptr<DataToSendToGPU> > > inQueue,
  size_t batchSize, std::shared_ptr<Display> sharedContext)
{
  Status status;

  /// Vector of handlers to process the data for each camera.  There will be one handler for each camera,
  // indexed by its ID.  We add to this vector as we get new cameras.
  std::vector< std::shared_ptr<CPUDataToTextureHandler> > handlers;

  auto lastPrint = std::chrono::steady_clock::now();

  while (!done) {
    // Once per second, print out the size of the input queue
    if (std::chrono::duration<double>(std::chrono::steady_clock::now() - lastPrint).count() > 1.0) {
      std::cout << "Input queue size: " << inQueue->size() << std::endl;
      lastPrint = std::chrono::steady_clock::now();
    }

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
          {
            // Get the camera ID, width and height from the message.
            uint32_t cameraID;
            uint16_t width, height;
            status = ParseFrameBeginMessage(*message, cameraID, width, height);
            if (OKAY != status) {
              std::cerr << "CopyDataToGPU: ParseFrameBeginMessage() failed: " << ErrorMessage(status) << std::endl;
              done = true;
              return;
            }

            // Construct the CPUDataToTextureHandler object to handle the data for this frame and store it in the vector
            // of handlers.  This will be used to process the data as it comes in.  Make more handlers as needed.
            if (cameraID >= handlers.size()) {
              handlers.resize(cameraID + 1);
            }
            handlers[cameraID] = std::make_shared<CPUDataToTextureHandler>(data, sharedContext, width, height,
              static_cast<uint16_t>(batchSize));
          }
          // Nothing to do for the beginning of a frame.
          break;

        case FRAME_DATA:
          // Copy the data to the pinned CPU memory buffer, and then asynchronously to the GPU buffer as
          // we get enough data for a minimum block size.  We send the data to the GPU in chunks so that
          // we amortize the per-send cost, but we send in chunks to reduce the latency and enable overlap
          // between data copying and processing (which increases throughput).
          {
            // Get the region to copy and the data pointer from the message.
            uint32_t cameraID;
            uint16_t left, right, top, bottom;
            uint8_t* dataPtr;
            status = ParseFrameDataMessage(*message, cameraID, left, right, top, bottom, dataPtr);
            if (OKAY != status) {
              std::cerr << "CopyDataToGPU: ParseFrameDataMessage() failed: " << ErrorMessage(status) << std::endl;
              done = true;
              return;
            }

            // Handle the data
            if (cameraID >= handlers.size()) {
              std::cerr << "CopyDataToGPU: FRAME_DATA: Error: Camera ID " << cameraID << " not found." << std::endl;
              done = true;
              return;
            }
            if (handlers[cameraID] == nullptr) {
              std::cerr << "CopyDataToGPU: FRAME_DATA: Warning: Camera ID " << cameraID << " frame data without begin data." << std::endl;
              break;
            }
            std::string ret = handlers[cameraID]->ProcessImageSubset(dataPtr, left, top, right, bottom);
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
            // Construct the end-of-frame message from the message.
            MessageFrameEnd frameEnd(*message);
            if (frameEnd.GetConstructorStatus() != OKAY) {
              std::cerr << "CopyDataToGPU: Failed to construct MessageFrameEnd: "
                << ErrorMessage(frameEnd.GetConstructorStatus()) << std::endl;
              done = true;
              return;
            }

            // Done with this frame, so we reset the pointer to delete the handler, which will clean
            // up and push the data to the texture before returning.
            /// @todo Consider putting these into a completion list and polling for done rather than hanging here.
            uint32_t cameraID;
            status = frameEnd.GetCameraID(cameraID);
            if (OKAY != status) {
              std::cerr << "CopyDataToGPU: GetCameraID() failed: " << ErrorMessage(status) << std::endl;
              done = true;
              return;
            }
            if (cameraID >= handlers.size()) {
              std::cerr << "CopyDataToGPU: FRAME_END: Error: Camera ID " << cameraID << " not found." << std::endl;
              done = true;
              return;
            }
            handlers[cameraID].reset();

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

/// @brief Thread for each camera that receives the data from the network and sends it to the GPU.
/// @param receiveSocket The socket to receive the data on.
/// @param maxBytesPerPacket The maximum number of bytes in a packet.
/// @param done The flag to set when we're done.
/// @param cpuImageBufferPtr The pinned memory buffer on the CPU to hold the image data.
/// @param gpuImageBufferPtr The buffer on the GPU to hold the image data.
/// @param streamPtr The stream to use for copy and kernel calls.
/// @param imageQueue The image queue to store the textures in.
/// @param outQueue The queue to send the data to the GPU-feeding thread.
static void ReceiveDataThread(ReceiverUDP& receiveSocket, size_t maxBytesPerPacket, std::atomic<bool>& done,
  std::shared_ptr<unsigned char> cpuImageBufferPtr, std::shared_ptr<unsigned char> gpuImageBufferPtr,
  std::shared_ptr<cudaStream_t> streamPtr,
  std::shared_ptr<asdp::render::ImageQueue> imageQueue,
  std::shared_ptr< SpinFreeQueue< std::shared_ptr<DataToSendToGPU> > > outQueue)
{
  // Generate a buffer pool to use to get pre-allocated buffers for reading the data from
  // the network.  Initially fill it with 100 buffers to give us enough to handle buffering a fraction
  // of a frame before the first packets are handled.  It will automatically expand if needed.
  BufferPool bufferPool(maxBytesPerPacket, 100);

  // Image width
  uint16_t cameraWidth = 0;

  // Loop through and receive packets until we've been told to quit.
  size_t packetsReceived = 0;
  DataToSendToGPU data;
  bool waitingForFrameBegin = true;
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

    // Verify that the data is correct and we haven't missed any packets.
    // If we have missed a packet, we will reset the sequence number and start over
    // at the next frame begin.
    uint32_t sequenceNumber;
    status = streamPacket->GetSequenceNumber(sequenceNumber);
    if (status != OKAY) {
      std::cerr << "Error getting sequence number: " << ErrorMessage(status) << std::endl;
      done = true;
      break;
    }
    if (sequenceNumber != packetsReceived) {
      std::cerr << "Warning: Bad sequence number: expected " << packetsReceived << " but got " << sequenceNumber << std::endl;
      packetsReceived = sequenceNumber + 1;
      waitingForFrameBegin = true;
      continue;
    }

    // Increment the number of packets received
    packetsReceived++;

    // Because we must copy the data into pinned memory for data messages, and because we must
    // check for begin-frame messages before sending anything, we must parse all of the
    // messages in the packet and handle them in turn.  We will ignore any messages that are not
    // these types.
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
          // We found a begin frame message, so we can start processing the data.
          waitingForFrameBegin = false;

          // Pull the information from the frame so that we can store the width data for this
          // camera.
          uint32_t cameraID;
          uint16_t width, height;
          status = ParseFrameBeginMessage(*message, cameraID, width, height);
          if (OKAY != status) {
            std::cerr << "ReceiveDataThread: ParseFrameBeginMessage() failed: " << ErrorMessage(status) << std::endl;
            done = true;
            return;
          }
          cameraWidth = width;
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
          uint16_t *cpuBuffer16 = reinterpret_cast<uint16_t*>(cpuImageBufferPtr.get());
          uint16_t *data16 = reinterpret_cast<uint16_t*>(data);
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

    // We're waiting for the frame begin message, so we ignore this packet that does not have one.
    if (waitingForFrameBegin) {
      continue;
    }

    // Enqueue the packet for processing.
    data.streamPacketPtr = streamPacket;
    data.cpuImageBufferPtr = cpuImageBufferPtr;
    data.gpuImageBufferPtr = gpuImageBufferPtr;
    data.imageQueuePtr = imageQueue;
    data.streamPtr = streamPtr;
    outQueue->enqueue(std::make_shared<DataToSendToGPU>(data));
  }

  // Release our out-queue pointer so it will be destroyed and release all its resources back to our
  // buffer pool.
  outQueue.reset();
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
  std::cerr << "Usage: " << name << " [--framestride <frameStride>] [--fullScreen] [--toneMap <tone map>] <ip_address>" << std::endl;
  std::cerr << "  --frameStride <frameStride>  Read one out of every this many frames. Set to 1 for every frame." << std::endl;
  std::cerr << "  --fullScreen                 Run in full screen mode." << std::endl;
  std::cerr << "  --toneMap <tone map>         The tone map to use.  Options are: linear blackbody bluesky" << std::endl;
  std::cerr << "  <ip_address>  The IP address to listen for servers on." << std::endl;
}

int main(int argc, char** argv)
{
  uint32_t frameStride = 1;     ///< Read one out of every this many frames. Set to 1 for every frame.
  bool fullScreen = false;      ///< Run in full screen mode.
  ToneMap toneMap = ToneMap();  ///< The tone map to use, default linear.
  std::string ip_address;       ///< The IP address to listen on.
  std::set<uint32_t> cameraIDs; ///< The camera IDs to render.
  size_t realParams = 0;        ///< The number of non-flag parameters we've seen.

  // Parse the command line arguments, with the first non-flag argument being the
  // name of the IP address to listen on.
  for (int i = 1; i < argc; ++i) {
    if (std::string("--frameStride") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 2;
      }
      frameStride = std::stoi(argv[i]);
    } else if (std::string("--fullScreen") == argv[i]) {
      fullScreen = true;
    } else if (std::string("--toneMap") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 2;
      }
      if (std::string("linear") == argv[i]) {
        toneMap = ToneMap();
      } else if (std::string("blackbody") == argv[i]) {
        toneMap = ToneMapBlackbody();
      } else if (std::string("bluesky") == argv[i]) {
        toneMap = ToneMapBlueSky();
      } else {
        std::cerr << "Unknown tone map: " << argv[i] << std::endl;
        return 1;
      }
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

    // If we have an empty set of camera IDs, then we want to analyze all cameras.
    /// @todo Replace with reading the configuration file to find out the cameras and their mappings
    if (cameraIDs.empty()) {
      for (uint32_t ID = 1; ID <= cameras.size(); ID++) {
        cameraIDs.insert(ID);
      }
    }

    // Read the configuration file associated with the serial number for the server. Verify that
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

    // Construct a DisplayTexture object to handle textures.  It will be the base object that all others will use
    // to share contexts.
    std::shared_ptr<DisplayTexture> displayTexture = std::make_shared<DisplayTexture>();

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
        json distortion = camera["distortion"];
        if (distortion["type"] == "none") {
          DistortionNone* distortion = new DistortionNone;
          info.m_distortion = std::shared_ptr<Distortion>(distortion);
          /// @todo Handle radial distortion
        } else if (distortion["type"] == "radial") {
          json parameters = distortion["parameters"];
          std::array<double, 2> center = parameters["COP"];
          json map = parameters["map"];
          std::vector< std::array<double, 2> > mapPoints = map;
          DistortionRadialLERP* distortion = new DistortionRadialLERP(center, mapPoints);
          info.m_distortion = std::shared_ptr<Distortion>(distortion);
        } else {
          std::cerr << "Error: Unknown distortion type: " << distortion["type"] << std::endl;
          return 17;
        }
        info.m_imageQueue = std::make_shared<asdp::render::ImageQueue>();

        //==================================================================================================
        // Fill in three textures for this camera, all gray and at time zero.
        // This creates one for the display to be using, one for the texture thread to write to, and
        // one to lie fallow so that there is a buffer between the two when the writing thread switches.
        // We must borrow the context from the displayTexture so that we can create the textures.
        if (!displayTexture->BorrowContext()) {
          std::cerr << "Error borrowing context from displayTexture." << std::endl;
          return 17;
        }

        unsigned int width = info.m_resolutionPixels[0];
        unsigned int height = info.m_resolutionPixels[1];
        std::vector<uint16_t> image(width * height, 32767);

        for (size_t i = 0; i < 3; i++) {
          // Create the textures for the camera.
          std::shared_ptr<ImageData> imageData = std::make_shared<ImageData>();

          unsigned int texture;
          glGenTextures(1, &texture);
          glBindTexture(GL_TEXTURE_2D, texture);
          // Set the texture wrapping parameters
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
          // Set texture filtering parameters
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

          // Load image into the texture
          glTexImage2D(GL_TEXTURE_2D, 0, GL_R16, width, height, 0, GL_RED, GL_UNSIGNED_SHORT, image.data());
          glBindTexture(GL_TEXTURE_2D, 0);

          imageData->texture = texture;
          info.m_imageQueue->AddNewestImage(imageData);
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

    // Construct a Tone Map texture to use for rendering the cameras.
    if (!displayTexture->BorrowContext()) {
      std::cerr << "Error borrowing context from displayTexture for ToneMap." << std::endl;
      return 20;
    }
    GLuint toneMapTexture = toneMap.GenerateTexture();
    if (toneMapTexture == 0) {
      std::cerr << "Error generating texture for ToneMap." << std::endl;
      return 21;
    }
    if (!displayTexture->ReturnContext()) {
      std::cerr << "Error returning context to displayTexture for ToneMap." << std::endl;
      return 21;
    }

    // Construct a Composite object to render the cameras.
    std::shared_ptr<Composite> composite = std::make_shared<CompositeCameras>(cameraRenderInfos, toneMapTexture);

    // Construct one or more Display objects to render the cameras.  They all share objects with the texture Display.
    std::vector<std::shared_ptr<DisplayWindow>> displays;
    displays.push_back(std::make_shared<DisplayWindow>("ASDP Render Module", composite, client, 0, 0, 60.0f, 2500, 1280, 1024,
      40.0f, "", displayTexture.get(), fullScreen));

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
        return 25;
      }
      UDPReceivers.push_back(receiverUDP);
    }

    // Launch the threads, hooking them together using the queues and passing the texture OpenGL context to it.
    std::thread copyDataToGPUThread(CopyDataToTextures, cameras[0].width, cameras[0].height,
      std::ref(done), dataQueue, 16, displayTexture);
    std::vector<std::thread> receiveDataThreads;
    for (size_t i = 0; i < cameras.size(); i++) {
      receiveDataThreads.push_back(std::thread(ReceiveDataThread, std::ref(*UDPReceivers[i]), 9000,
        std::ref(done), cpuPinnedImageBuffers[i], gpuImageBuffers[i], streams[i], cameraRenderInfos[i].m_imageQueue,
        dataQueue));
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
      status = client->SendCommandPacket(CommandPacketConfigureTrigger(ti));
      if (status != OKAY) {
        std::cerr << "Failed to configure trigger: " << ErrorMessage(status) << std::endl;
        return 29;
      }
      std::cout << "  Configured trigger for camera " << camID << " with period " << ti.period << " seconds" << std::endl;

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
      status = client->SendCommandPacket(CommandPacketStreamSubregion(endpoint, region));
      if (status != OKAY) {
        std::cerr << "Failed to stream images: " << ErrorMessage(status) << std::endl;
        return 32;
      }
    }

    // Render frames until someone has marked us to be done.
    start = std::chrono::steady_clock::now();
    while (!done) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));

      // If all of our DisplayWindows have been closed (or are broken), then we're done.
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
    }

    // Set done and wait for all of our singleton threads to join.
    done = true;
    copyDataToGPUThread.join();

    // Destroy our client
    client.reset();

    // Clear all remaining data from the queue now that the receivers are done.
    // All of the receiving threads will also delete this before they exit, which will remove all of the
    // references and push their buffers back onto their empty queues.
    dataQueue.reset();

    // Now that all of the buffers have been returned to the buffer queue, join our receive-data threads.
    for (auto& thread : receiveDataThreads) {
      if (thread.joinable()) {
        thread.join();
      }
    }

    // Now borrow the context from the displayTexture so that we can delete the textures.
    if (!displayTexture->BorrowContext()) {
      std::cerr << "Error borrowing context from displayTexture." << std::endl;
      return 33;
    }
    cameraRenderInfos.clear();
    glDeleteTextures(1, &toneMapTexture);
    if (!displayTexture->ReturnContext()) {
      std::cerr << "Error returning context to displayTexture." << std::endl;
      return 34;
    }
  }

  return 0;
}