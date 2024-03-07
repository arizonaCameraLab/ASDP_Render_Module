/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include <iostream>
#include <string>
#include <cstdlib>
#include <thread>
#include <chrono>
#include <mutex>
#include <list>
#include <atomic>
#include <condition_variable>
#include <string.h>
#include <ASDP_Core_API.h>
#include <cuda.h>
#include <cuda_runtime.h>
using namespace asdp;

template <typename T> class SpinFreeQueue {
private:
  struct Node {
    T data;
    Node* next;
  };

  Node* head;
  Node* tail;
  size_t nodes;
  std::condition_variable cv;
  std::mutex cv_m;
  std::mutex mut;

public:
  SpinFreeQueue() {
    head = nullptr;
    tail = nullptr;
    nodes = 0;
  }

  ~SpinFreeQueue() {
    std::lock_guard<std::mutex> lk(mut);
    while (head) {
      Node* old_head = head;
      head = old_head->next;
      delete old_head;
      nodes--;
    }
  }

  void enqueue(T data) {
    {
      std::lock_guard<std::mutex> lk(mut);
      Node* new_node = new Node;
      new_node->data = data;
      new_node->next = nullptr;

      if (nodes == 0) {
        head = new_node;
        tail = new_node;
      } else {
        tail->next = new_node;
        tail = new_node;
      }

      nodes++;
    }
    cv.notify_one();
  }

  bool dequeue(T& value, const std::chrono::milliseconds& timeout) {
    if (nodes == 0) {
      std::unique_lock<std::mutex> cvlk(cv_m);
      if (!cv.wait_for(cvlk, timeout, [&] { return nodes != 0; })) {
        return false;
      }
    }

    std::lock_guard<std::mutex> lk(mut);
    if (nodes == 0) {
      return false;
    }
    value = head->data;
    Node* old_head = head;
    head = old_head->next;
    delete old_head;
    nodes--;
    if (head == nullptr) {
      tail = head;
    }
    return true;
  }

  size_t size() const {
    return nodes;
  }
};

/// @brief Structure to hold the data needed to send data to the GPU.
struct DataToSend {
  unsigned char* cpuBuffer;    ///< Copy from here
  unsigned char* gpuBuffer;    ///< Copy to here
  size_t size;                 ///< Size of the data
  cudaStream_t stream;         ///< Stream to use to for the copy and kernel run
};

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

/// @brief Separate thread to copy data to the GPU and run kernels on it.
/// @return None
static void handleDataThread(std::atomic<bool>& done, SpinFreeQueue<DataToSend>& queue)
{
  DataToSend data;
  while (!done) {
    if (queue.dequeue(data, std::chrono::milliseconds(1))) {
      cudaError_t ret = cudaMemcpyAsync(data.gpuBuffer, data.cpuBuffer, data.size,
        cudaMemcpyHostToDevice, data.stream);
      if (ret != cudaSuccess) {
        std::cerr << "Error: " << cudaGetErrorString(ret) << std::endl;
        return;
      }

      // Perform a dummy computation on the data.
      int size = data.size / 2;
      int blockSize = 256;
      int numBlocks = (size + blockSize - 1) / blockSize;
      uint16_t* data16 = reinterpret_cast<uint16_t*>(data.gpuBuffer);
      squareAndDivideKernel << <numBlocks, blockSize, 0, data.stream >> > (data16, size);
    }
  }
}

/// @brief Separate thread per camera to receive and process data from the network.
/// @param receiveSocket The socket to use to receive data.
/// @param queue The queue to use to send data to the GPU.
/// @param stream The stream to use to copy data to the GPU and run kernels.
/// @param cpuBuffer The pinned memory buffer to use to copy data to the GPU.
/// @param gpuBuffer The GPU buffer to use to copy data to the GPU.
/// @param bytesPerPacket The number of bytes per packet.
/// @param packetsPerFrame The number of packets per frame.
/// @param totalFrames The total number of frames to receive.
/// @param packetsPerGPUSend The number of packets to send to the GPU at a time.
/// @param printMutex The mutex to use to print to the console.
/// @param broken Atomic boolean to signal an error.
static void receiveDataThread(ReceiverUDP& receiveSocket, SpinFreeQueue<DataToSend>& queue,
  cudaStream_t stream, unsigned char* cpuBuffer, unsigned char* gpuBuffer,
  size_t bytesPerPacket, size_t packetsPerFrame, size_t totalFrames,
  size_t packetsPerGPUSend,
  std::mutex& printMutex, std::atomic<bool> &broken)
{
  std::vector<uint8_t> buffer(bytesPerPacket);
  unsigned packetsReceived = 0;
  std::vector<char> copyBuffer(bytesPerPacket);

  // Loop through and receive packets until we've gotten them all or an error occurs
  while (packetsReceived < packetsPerFrame * totalFrames) {
    Status status = receiveSocket.ReceiveBuffer(buffer);
    if (status != OKAY) {
      std::cerr << "Error receiving data: " << ErrorMessage(status) << std::endl;
      return;
    }

    // Verify that the data is correct and we haven't missed any packets
    if (buffer[0] != (packetsReceived % 256)) {
      std::lock_guard<std::mutex> lock(printMutex);
      std::cerr << "Error: Expected " << (packetsReceived % 256) << " but got " << (int)buffer[0] << std::endl;
      broken = true;
      return;
    }

    // Copy the memory to the relevant portion of the pinned CPU buffer.
    // Wrap back around to the start when we're done with a frame.
    size_t mod = packetsReceived % packetsPerGPUSend;
    size_t offset = mod * bytesPerPacket;
    {
      std::lock_guard<std::mutex> lock(printMutex);
    }
    memcpy(cpuBuffer + offset, buffer.data(), bytesPerPacket);

    // If we've reached the number of packets to send to the GPU, send them.
    // We do this by enqueueing the data to the queue, which will be handled by the handleDataThread.
    if (mod == packetsPerGPUSend - 1) {
      size_t framesReceived = packetsReceived / packetsPerFrame;
      size_t baseOffset = (packetsReceived - framesReceived * packetsPerFrame)
        / packetsPerGPUSend * bytesPerPacket;
      DataToSend data;
      data.cpuBuffer = cpuBuffer + baseOffset;
      data.gpuBuffer = gpuBuffer + baseOffset;
      data.size = packetsPerGPUSend * bytesPerPacket;
      data.stream = stream;
      {
        std::lock_guard<std::mutex> lock(printMutex);
      }
      queue.enqueue(data);
    }

    // Increment the number of packets received
    packetsReceived++;
  }

}

int main(int argc, char* argv[])
{
  int cameras = 25;
  float fps = 60.0;
  int secondsWorth = 10;
  int packetsPerGPUSend = 10;
  std::string IP = "localhost";
  int port = 12000;
  size_t realParams = 0;

  for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];
    if (arg == "--cameras") {
      cameras = std::stoi(argv[++i]);
    } else if (arg == "--fps") {
      if (i + 1 >= argc) {
        std::cerr << "Error: --fps must be followed by a number" << std::endl;
      }
      fps = std::stof(argv[++i]);
    } else if (arg == "--secondsWorth") {
      if (i + 1 >= argc) {
        std::cerr << "Error: --secondsWorth must be followed by a number" << std::endl;
      }
      secondsWorth = std::stoi(argv[++i]);
    } else if (arg == "--packetsPerGPUSend") {
      if (i + 1 >= argc) {
        std::cerr << "Error: --packetsPerGPUSend must be followed by a number" << std::endl;
      }
      packetsPerGPUSend = std::stoi(argv[++i]);
    } else if (arg == "--IP") {
      if (i + 1 >= argc) {
        std::cerr << "Error: --IP must be followed by a string" << std::endl;
      }
      IP = argv[++i];
    } else if (arg == "--port") {
      if (i + 1 >= argc) {
        std::cerr << "Error: --port must be followed by a number" << std::endl;
      }
      port = std::stoi(argv[++i]);
    } else if (arg[0] == '-') {
      std::cerr << "Unknown option: " << arg << std::endl;
      return 1;
    } else {
      ++realParams;
      switch (realParams) {
      case 1:
      default:
        std::cerr << "Unexpected argument: " << arg << std::endl;
        return 1;
      }
    }
  }

  std::cout << "ASDP Speed Test Receiver" << std::endl;
  std::cout << "Listens for data from the Speed_Test_Sender and checks for dropped packets" << std::endl;
  std::cout << "Run this before running the sender." << std::endl;
  std::cout << "Usage: Speed_Test_Receiver [--cameras <number>] [--fps <number>] [--secondsWorth <number>] [--IP <string>] [--port <number>]" << std::endl;
  std::cout << "       It listens on the port specified and a number above it for each camera." << std::endl;
  std::cout << "The parameters here must match those used by the sender." << std::endl;
  std::cout << std::endl;
  std::cout << "Cameras: " << cameras << std::endl;
  std::cout << "FPS: " << fps << std::endl;
  std::cout << "Seconds worth of data: " << secondsWorth << std::endl;
  std::cout << "Listening on IP:Port and following " << IP << ":" << port << std::endl;

  // Compute the total number of packets to receive, where we send 342 packets per frame.
  size_t packetsPerFrame = 342;
  size_t totalPacketsPerCamera = static_cast<size_t>(fps * secondsWorth) * packetsPerFrame;

  // We receive three lines of 1280 pixels of 2 bytes each.
  size_t bytesPerPacket = 1280 * 2 * 3;
  size_t bytesPerFrame = bytesPerPacket * packetsPerFrame;

  // Thread to handle saving data to file and associated resources
  std::thread saveThread;
  std::atomic<bool> done(false);
  SpinFreeQueue<DataToSend> queue;
  saveThread = std::thread(handleDataThread, std::ref(done), std::ref(queue));

  // Create the receive sockets.
  std::vector<ReceiverUDP> receiveSockets;
  for (unsigned i = 0; i < cameras; i++) {
    receiveSockets.push_back(ReceiverUDP(IP, port + i));
    if (receiveSockets.back().GetConstructorStatus() != OKAY) {
      std::cerr << "Error creating receive socket: " << receiveSockets.back().GetConstructorStatus() << std::endl;
      return 2;
    }
  }

  // Structures to let us clean up at the end.
  std::vector<unsigned char*> cpuBuffers;
  std::vector<unsigned char*> gpuBuffers;
  std::vector<cudaStream_t> streams;

  // Start the specified number of receiver threads.
  std::mutex printMutex;
  std::atomic<bool> broken(false);
  std::vector<std::thread> receivers;
  for (unsigned i = 0; i < cameras; i++) {
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

    std::thread receiver(receiveDataThread, std::ref(receiveSockets[i]), std::ref(queue),
      stream, cpuBuffer, gpuBuffer,
      bytesPerPacket, packetsPerFrame, static_cast<size_t>(secondsWorth * fps),
      packetsPerGPUSend, std::ref(printMutex), std::ref(broken));
    receivers.push_back(std::move(receiver));
  }

  // Wait for the threads to finish.
  for (unsigned i = 0; i < cameras; i++) {
    receivers[i].join();
  }

  // Check for any errors
  int ret = 0;
  if (broken) {
    std::cerr << "Error: Packets were dropped" << std::endl;
    ret = 3;
  } else {
    std::cout << "Success" << std::endl;
  }

  // If we have a thread, time how long it takes it to finish
  if (saveThread.joinable()) {
    size_t queueSize = queue.size();
    std::chrono::time_point<std::chrono::steady_clock> start = std::chrono::steady_clock::now();
    done = true;
    saveThread.join();
    std::chrono::time_point<std::chrono::steady_clock> end = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::lock_guard<std::mutex> lock(printMutex);
    std::cout << "Save thread had " << queueSize << " items in the queue" << std::endl;
    std::cout << "  Time to save data: " << elapsed.count() << " seconds" << std::endl;
  }

  // Clean up resources
  for (int i = 0; i < cameras; ++i) {
    cudaFreeHost(cpuBuffers[i]);
    cudaFree(gpuBuffers[i]);
    cudaStreamDestroy(streams[i]);
  }
  receivers.clear();
  receiveSockets.clear();

  return ret;
}
