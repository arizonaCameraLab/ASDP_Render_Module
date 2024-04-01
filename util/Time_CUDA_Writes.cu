// Copyright 2024, Apache Strap-Down Pilotage contract, University of Arizona.
// This program is a speed test to ensure that we are able to send data from
// pinned CPU buffers to GPU buffers quickly enough, sending several lines at
// a time.
// The maximum number of cameras to be used is 25, each with 1280x1024 16-bit
// pixels running at a maximum of 60Hz.

#ifdef _WIN32
#include <windows.h>
#endif
#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <atomic>
#include <cuda.h>
#include <cuda_runtime.h>

std::atomic<int> totalPacketsReceived(0);
std::atomic<bool> beginSending(false);

__global__ void squareAndDivideKernel(uint16_t* data, int size)
{
  int index = threadIdx.x + blockIdx.x * blockDim.x;
  if (index < size) {
    int16_t val = data[index];
    // Do about as much math as the transmission time on a GeForce 4080 laptop GPU
    for (size_t i = 0; i < 300; i++) {
      val = val * val / 10;
    }
    data[index] = val;
  }
}

void copyThread(int which, int count, std::vector<unsigned char*> const &cpuBuffers,
  std::vector<unsigned char*> const& gpuBuffers, std::vector<cudaStream_t> const& streams,
  int totalIterations, int packetsPerFrame, int bytesPerPacket,
  bool compute)
{
  // Wait for the trigger signal and then go
  while (!beginSending) {}

  // Perform asynchronous copies among all of the buffers, using one stream for
  // each camera and sending round-robin among the streams. Then perform a
  // computation on the data if we've been asked to.
  for (int i = 0; i < totalIterations; ++i) {
    for (int k = 0; k < packetsPerFrame; ++k) {
      // Skip this unless it is ours.
      if (k % count != which % count) {
        continue;
      }
      int packetOffset = k * bytesPerPacket;
      for (int j = 0; j < streams.size(); ++j) {
        int cameraIndex = j;
        int streamIndex = j;
        cudaError_t ret = cudaMemcpyAsync(gpuBuffers[cameraIndex] + packetOffset,
          cpuBuffers[cameraIndex] + packetOffset,
          bytesPerPacket,
          cudaMemcpyHostToDevice,
          streams[streamIndex]);
        if (ret != cudaSuccess) {
          std::cerr << "Error: " << cudaGetErrorString(ret) << std::endl;
          return;
        }
        if (compute) {
          // Perform a dummy computation on the data.
          int size = bytesPerPacket / 2;
          int blockSize = 256;
          int numBlocks = (size + blockSize - 1) / blockSize;
          uint16_t *data = (uint16_t*)(gpuBuffers[cameraIndex] + packetOffset);
          squareAndDivideKernel << <numBlocks, blockSize, 0, streams[streamIndex]>> > (data, size);
        }
      }
    }
  }
}

int main() {
  int imageWidth = 1280;
  int imageHeight = 1024;
  int bitsPerPixel = 16;
  int bytesPerFrame = imageWidth * imageHeight * bitsPerPixel / 8;
  int numCameras = 25;
  int totalIterations = 480;
  int numThreads = 1;   // Multiple threads did not help performance

  int ret = 0;

  for (int shift = 10; shift >= 3; --shift) {
    int linesPerPacket = 1 << shift;  // 16 is minimum for 60fps with 25 cameras, 8 with 21 cameras
    int bytesPerLine = (imageWidth * bitsPerPixel) / 8;
    int bytesPerPacket = bytesPerLine * linesPerPacket;
    int packetsPerFrame = imageHeight / linesPerPacket;
    int packetsPerIteration = numCameras * packetsPerFrame;

    for (int compute = 0; compute <= 1; compute++) {
      std::cout << "Testing with linesPerPacket = " << linesPerPacket
          << ", compute = " << compute << std::endl;

      // Check the parameters
      if ((imageHeight / linesPerPacket) * linesPerPacket != imageHeight) {
        std::cerr << "linesPerPacket must evenly divide imageHeight" << std::endl;
        return 101;
      }
      if (bytesPerPacket / linesPerPacket != bytesPerLine) {
        std::cerr << "linesPerPacket must evenly divide bytesPerLine" << std::endl;
        return 102;
      }
      if ((imageHeight / numThreads) * numThreads != imageHeight) {
        std::cerr << "numThreads must evenly divide imageHeight" << std::endl;
        return 103;
      }

      // Allocate the pinned CPU buffers and GPU buffers. Construct streams.
      std::vector<unsigned char*> cpuBuffers;
      std::vector<unsigned char*> gpuBuffers;
      std::vector<cudaStream_t> streams;
      for (int i = 0; i < numCameras; ++i) {
        // Allocate pinned host memory
        unsigned char* cpuBuffer = nullptr;
        cudaMallocHost((void**)&cpuBuffer, bytesPerFrame);
        cpuBuffers.push_back(cpuBuffer);

        // Allocate device memory
        unsigned char* gpuBuffer = nullptr;
        cudaMalloc((void**)&gpuBuffer, bytesPerFrame);
        gpuBuffers.push_back(gpuBuffer);

        // Create stream
        cudaStream_t stream;
        cudaStreamCreate(&stream);
        streams.push_back(stream);
      }

      std::cout << "  Performing " << packetsPerIteration * totalIterations << " total copies"
        << " of size " << bytesPerPacket << " bytes" << std::endl;

      // Start the sender threads
      std::vector<std::thread> senderThreads;
      for (int i = 0; i < numThreads; i++) {
        std::thread senderThread(copyThread, i, numThreads, cpuBuffers, gpuBuffers, streams,
          totalIterations, packetsPerFrame, bytesPerPacket, compute);
        senderThreads.push_back(std::move(senderThread));
      }

      // Sleep to enable send and receive threads to start
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));

      // Measure the time taken for sending all iterations, starting the
      // clock just before starting the copies.
      auto start = std::chrono::high_resolution_clock::now();
      beginSending = true;

      // Wait for the sender threads to finish
      for (unsigned i = 0; i < senderThreads.size(); i++) {
        senderThreads[i].join();
      }
      beginSending = false;

      // Synchronize the device and then measure the time when all copies have
      // completed.
      cudaDeviceSynchronize();

      cudaError_t err = cudaGetLastError();
      if (err != cudaSuccess) {
        std::cerr << "Error: " << cudaGetErrorString(err) << std::endl;
        return 200;
      }

      auto end = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> duration = end - start;

      std::cout << "  Total time taken: " << duration.count() << " seconds\n";
      std::cout << "  Average time per iteration: " << duration.count() / totalIterations << " seconds\n";
      double fps = totalIterations / duration.count();
      std::cout << "  Average frames per second: " << fps << std::endl;
      if (fps < 60.0) {
        std::cout << "  *** Error: Average frames per second is less than 60" << std::endl;
      }

      // Cleanup
      for (int i = 0; i < numCameras; ++i) {
        cudaFreeHost(cpuBuffers[i]);
        cudaFree(gpuBuffers[i]);
        cudaStreamDestroy(streams[i]);
      }

    }
    std::cout << std::endl;
  }

  return ret;
}
