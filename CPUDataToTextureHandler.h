/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

 /**
  * @file CPUDataToTextureHandler.h
  * @brief Apache Strap-Down Pilotage Render/CPUDataToTextureHandler class header file.
  *
  * @author ReliaSolve.
  * @date September 13, 2024.
  */

#pragma once
#include <GL/glew.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <string>
#include <memory>
#include <map>
#include <vector>
#include <ASDP_Core_API.h>
#include <ASDP_SpinFreeQueue.hpp>
#include <ASDP_BufferPool.h>
#include <ASDP_StreamPacketSortedQueue.h>
#include "Display.h"
#include "ImageQueue.h"
#include "CUDABufferPool.h"

namespace asdp {
  namespace render {

/// @brief Summary of information from image messages.
/// @details This enables us to retain the information about what messages were in a packet
/// so that the data can be processed in the other thread without needing to keep around the
/// stream packet object or re-parse it. This is necessary to avoid causing a bottleneck in the
/// ReceiveDataThread.
struct MessageSummary {
  bool isFrameBegin = false;        ///< True if this is the start of a frame
  bool isFrameEnd = false;          ///< True if this is the end of a frame
  asdp::Time time;                  ///< The time associated with the message
  uint32_t cameraID = 0;            ///< The camera ID
  uint16_t width = 0;               ///< The width of the image data (if present for a message type)
  uint16_t height = 0;              ///< The height of the image data (if present for a message type)
  uint16_t left = 0;                ///< The left edge of the region to process (if present for a message type)
  uint16_t top = 0;                 ///< The top edge of the region to process (if present for a message type)
  uint16_t right = 0;               ///< The right edge of the region to process (if present for a message type)
  uint16_t bottom = 0;              ///< The bottom edge of the region to process (if present for a message type)
  float exposure = 0.0f;            ///< The exposure time for the image (if present for a message type)
  float gain = 0.0f;                ///< The gain for the image (if present for a message type)
};

/// @brief Structure to hold the data needed to send data to the GPU and run the kernel.
/// @details These will all have been constructed by the thread that is pushing them onto the queue,
/// with custom destructors as needed to free the memory when the shared_ptr is destroyed.
/// This has all of the information needed to get the image all the way into the texture to be rendered.
struct DataToSendToGPU {
  std::vector<MessageSummary> messages;   ///< Summaries of the messages included in this data packet
  std::shared_ptr<unsigned char> cpuImageBufferPtr;///< The pinned-memory buffer on the CPU that holds the image data
  std::shared_ptr<unsigned char> gpuImageBufferPtr;///< The buffer on the GPU that holds the image data
  std::shared_ptr<asdp::render::ImageQueue> imageQueuePtr;///< The image queue holding the textures to store into
  std::shared_ptr<cudaStream_t> streamPtr;         ///< Stream to use to for copy and kernel calls
  std::shared_ptr<uint8_t> gpuNUCGainPtr;          ///< The buffer on the GPU that holds the per-pixel NUC gain data, if any
  std::shared_ptr<uint8_t> gpuNUCOffsetPtr;        ///< The buffer on the GPU that holds the per-pixel NUC offset data, if any
};

/// @brief Class to handle processing of the data from the cameras and sending it to texture.
class CPUDataToTextureHandler {
public:
  /// @brief Constructor to create the handler and set up the resources needed to process a frame.
  /// @details Be sure to call GetStatus() after construction to verify that the constructor succeeded.
  /// @param texturesToCUDAMap The map from texture ID to CUDA graphics resource for the texture data.
  /// @param dataPtr Pointer to the structure that holds the data to send to the GPU and the stream to use.
  /// @param width The width of the image data (the whole image).
  /// @param height The height of the image data (the whole image).
  /// @param batchSize The number of lines to send to the GPU at once (the height of the region that will be sent).
  /// @param exposure The exposure time for the image.
  /// @param gain The gain for the image.
  CPUDataToTextureHandler(std::shared_ptr< std::map<GLuint, cudaGraphicsResource*> > texturesToCUDAMap,
    std::shared_ptr<DataToSendToGPU> dataPtr,
    uint16_t width, uint16_t height, uint16_t batchSize, float exposure, float gain);

  ~CPUDataToTextureHandler();

  /// @brief Process the image subset, sending to GPU memory and then running the kernel to store into texture.
  /// @param left The left edge of the region to process.
  /// @param top The top edge of the region to process.
  /// @param right The right edge of the region to process.
  /// @param bottom The bottom of the region to process.
  /// @return Empty string on success, description of error on failure.
  std::string ProcessImageSubset(uint16_t left, uint16_t top, uint16_t right, uint16_t bottom);

  /// @brief Set the center time for the image data.  Can be called any time before destruction.
  /// @param centerTime The time the image was taken.
  /// @return Empty string on success, description of error on failure.
  std::string SetCenterTime(asdp::Time centerTime);

  /// @brief Get the status of the constructor.
  /// @return The status of the constructor, empty for good, error message for bad.
  std::string GetStatus() const { return m_status; }

protected:
  std::string m_status;                       ///< The status of the constructor, empty for good, error message for bad

  std::shared_ptr<DataToSendToGPU> m_dataPtr; ///< Information about the structure we're handling
  uint16_t m_width;                           ///< The width of the image data
  uint16_t m_height;                          ///< The height of the image data
  uint16_t m_batchSize;                       ///< The number of lines to send to the GPU at once
  int16_t m_lastLineSent;                     ///< The last line sent to the GPU, starts at -1 which is just below 0
  uint16_t m_largestLineReceived;             ///< The largest line received so far
  asdp::Time m_centerTime;                    ///< The time the image was taken
  float m_exposure;                           ///< The exposure time for the image
  float m_gain;                               ///< The gain for the image

  std::shared_ptr<asdp::render::ImageData> m_imageData;///< The image data for the texture, including time and texure ID
  cudaGraphicsResource* m_resource;           ///< The CUDA graphics resource for the texture
  cudaArray* m_textureData;                   ///< The CUDA array for the texture
  cudaSurfaceObject_t m_surfObj;              ///< The CUDA surface object for the texture

  /// @brief Function to send all unsent data to the GPU and run the kernel to store it into the texture.
  /// @return Empty string on success, description of error on failure.
  std::string SendToGPU();
};

/// @brief Function to copy data to the GPU and store it into the appropriate textures.
/// It must create and record an event after all operations are complete.  All operations must be
/// done on the stream that is passed in and they must all be asynchronous.  There is a single
/// thread to handle all cameras; it uses different CUDA streams to overlap the operations.
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
/// @param cameraTimings The timing information for each camera, fill in the texture time for the appropriate camera.
/// This does not fill in camera timings if the vector is empty.
void CopyDataToTextures(uint16_t width, uint16_t height,
  std::atomic<bool>& done,
  std::shared_ptr< SpinFreeQueue< std::shared_ptr<DataToSendToGPU> > > inQueue,
  size_t batchSize, std::shared_ptr<Display> sharedContext,
  std::vector<RenderTimingInfo::camera>& cameraTimings);

/// @brief Class that holds shared pointers to both the gain and offset NUC data for a camera,
/// so that they can be kept consistent on the same queue.
struct NUCDataPair {
  std::shared_ptr<uint8_t> gainBuffer;   ///< The buffer for the gain data
  std::shared_ptr<uint8_t> offsetBuffer; ///< The buffer for the offset data
};

/// @brief Thread for each camera that receives the data from the network and sends it to the GPU.
/// @param receiveSocket The socket to receive the data on.
/// @param maxBytesPerPacket The maximum number of bytes in a packet.
/// @param done The flag to set when we're done.
/// @param cpuImageBuffers Pool of pinned memory buffer on the CPU to hold the image data.
/// @param gpuImageBuffers Pool of buffers on the GPU to hold the image data.
/// @param streamPtr The stream to use for copy and kernel calls.
/// @param imageQueue The image queue to store the textures in.
/// @param outQueue The queue to send the data to the GPU-feeding thread.
/// @param frameBeginTimes Store the times for the begin frame message receipts. nullptr
/// does not record them.
/// @param frameEndTimes Store the times for the end frame message receipts. nullptr
/// does not record them.
/// @param NUCTablesQueue A queue for the client to send new pairs of NUC gain and offset tables
/// to the receiver thread, nullptr if not used.
void ReceiveDataThread(ReceiverUDP& receiveSocket, size_t maxBytesPerPacket, std::atomic<bool>& done,
  std::shared_ptr<CUDABufferPool> cpuImageBuffers, std::shared_ptr<CUDABufferPool> gpuImageBuffers,
  std::shared_ptr<cudaStream_t> streamPtr,
  std::shared_ptr<asdp::render::ImageQueue> imageQueue,
  std::shared_ptr< SpinFreeQueue< std::shared_ptr<DataToSendToGPU> > > outQueue,
  std::vector<std::chrono::steady_clock::time_point>* frameBeginTimes,
  std::vector<std::chrono::steady_clock::time_point>* frameEndTimes,
  std::shared_ptr < SpinFreeQueue < NUCDataPair > > NUCTablesQueue);

  } // namespace render
} // namespace asdp
