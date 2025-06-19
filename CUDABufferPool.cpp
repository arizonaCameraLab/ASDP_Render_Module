/*
 * Copyright (C) 2025: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include "CUDABufferPool.h"
#include <thread>
#include <iostream>
#include <stdexcept>
#include <cuda_runtime.h>

using namespace asdp::render;

/// @brief Allocate the appropriate type based on the host flag.
static cudaError_t mallocBuffer(unsigned char** buffer, size_t size, bool host)
{
  if (host) {
    return cudaMallocHost(buffer, size);
  } else {
    return cudaMalloc(buffer, size);
  }
}

/// @brief Free the appropriate type based on the host flag.
static cudaError_t freeBuffer(unsigned char* buffer, bool host)
{
  if (host) {
    return cudaFreeHost(buffer);
  } else {
    return cudaFree(buffer);
  }
}

CUDABufferPool::CUDABufferPool(size_t bufferSize, size_t bufferCount, bool host)
  : m_host(host)
  , m_bufferSize(bufferSize)
  , m_done(false)
{
  // Fill the buffer pool with buffers.
  // Fill the free-buffer list with pointers to the buffers.
  for (size_t i = 0; i < bufferCount; ++i) {
    unsigned char* buffer = nullptr;
    cudaError_t ret = mallocBuffer(&buffer, m_bufferSize, m_host);
    if (ret != cudaSuccess) {
      std::cerr << "PinnedBufferPool::PinnedBufferPool): CUDA malloc failed: " << cudaGetErrorString(ret) << std::endl;
      throw std::runtime_error("CUDA malloc failed");
    }
    m_allBuffers.push_back(buffer);
    m_freeBuffers.push_back(buffer);
  }
}

CUDABufferPool::~CUDABufferPool()
{
  // Wait for all the buffers to be returned to the pool.  Because we are setting
  // m_done to true, no more buffers will be allocated in any threads while we're
  // waiting for the existing buffers to be returned.
  m_done = true;
  bool gotThemAll = false;
  do {
    {
      std::lock_guard<std::mutex> lock(mtx);
      gotThemAll = (m_freeBuffers.size() == m_allBuffers.size());
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  } while (!gotThemAll);

  // Clear the buffer pools while holding the lock.
  std::lock_guard<std::mutex> lock(mtx);
  for (uint8_t* buffer : m_allBuffers) {
    freeBuffer(buffer, m_host);
  }
  m_allBuffers.clear();
  m_freeBuffers.clear();
}

std::shared_ptr<uint8_t> CUDABufferPool::GetBuffer(bool allocateWhenEmpty, size_t timeoutMilli)
{
  // If we are being destroyed, then we can't return any more buffers
  if (m_done) {
    return nullptr;
  }

  // If the buffer pool is empty, then we need to allocate a new buffer and also
  // add it to the list of empty buffers or else wait for one to become available.
  mtx.lock();
  if (m_freeBuffers.empty()) {
    if (allocateWhenEmpty) {
      // Allocate a new buffer and put it on the free list.
      unsigned char* buffer = nullptr;
      cudaError_t ret = mallocBuffer(&buffer, m_bufferSize, m_host);
      if (ret != cudaSuccess) {
        std::cerr << "CUDABufferPool::GetBuffer(): CUDA malloc failed: " << cudaGetErrorString(ret) << std::endl;
        throw std::runtime_error("CUDA malloc failed");
      }
      m_allBuffers.push_back(buffer);
      m_freeBuffers.push_back(buffer);
    } else {
      // Wait until the free list is not empty, sleeping with an unlocked mutex
      // so that other threads can free buffers.
      size_t count = 0;
      while (m_freeBuffers.empty()) {
        mtx.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        // If we are being destroyed, then we can't return any more buffers.
        // We must check again here because done could have been set in the meantime.
        if (m_done || (++count > timeoutMilli)) {
          return nullptr;
        }
        mtx.lock();
      }
    }
  }

  // Get a buffer from the pool and return it to the caller.
  uint8_t* buffer = m_freeBuffers.front();
  m_freeBuffers.pop_front();

  // Done with the mutex
  mtx.unlock();

  // Make a shared_ptr that will return the buffer to the empty pool when it is destroyed.
  return std::shared_ptr<uint8_t>(buffer, [this, buffer](uint8_t*) {
      std::lock_guard<std::mutex> lock(mtx);
      m_freeBuffers.push_back(buffer);
    });
}

static void TestBufferPoolAllocationThread(CUDABufferPool* pool, double tryPeriod, int tryTimes,
  std::atomic_int *successCount, std::atomic_bool *running, bool allocateWhenEmpty)
{
  *running = true;
  std::vector< std::shared_ptr<uint8_t> > buffers;
  for (int i = 0; i < tryTimes; i++) {
    std::shared_ptr<uint8_t> buffer = pool->GetBuffer(allocateWhenEmpty, 1000);
    buffers.push_back(buffer);
    if (buffer != nullptr) {
      (*successCount)++;
    }
    std::this_thread::sleep_for(std::chrono::microseconds((int)(tryPeriod * 1e6)));
  }

  // Buffers will be cleared when the function returns.
}

std::string CUDABufferPool::Test()
{
  // Single-threaded test of the buffer pool.  Verify that the buffer pool size
  // adjusts as expected when we allocate new buffers.
  for (bool host : {false, true}) {
    CUDABufferPool pool(100, 10, host);
    if (pool.m_freeBuffers.size() != 10) {
      return "CUDABufferPool::Test() failed: pool.m_freeBuffers.size() != 10";
    }

    // Check as we allocate free buffers and then get more than are available.
    std::vector < std::shared_ptr<uint8_t> > buffers;
    for (size_t i = 0; i < 20; i++) {
      size_t expected = (i < 9) ? 9 - i : 0;
      std::shared_ptr<uint8_t> buffer = pool.GetBuffer(true, 1000);
      buffers.push_back(buffer);
      if (pool.m_freeBuffers.size() != expected) {
        return "Allocation failed: pool.m_freeBuffers.size() != " + std::to_string(expected);
      }
    }

    // Try to get another buffer without allocating and ensure that we time out.
    std::shared_ptr<uint8_t> buffer = pool.GetBuffer(false, 10);
    if (buffer != nullptr) {
      return "Allocation failed: pool.GetBuffer(false, 10) returned a buffer";
    }

    // Check after we return all buffers.
    buffers.clear();
    if (pool.m_freeBuffers.size() != 20) {
      return "Freeing failed: pool.m_freeBuffers.size() != 20";
    }
  }

  // Multi-threaded test of the buffer pool.  Verify that the GetBuffer() method
  // returns nullptr when the pool is being destroyed and that the destructor
  // waits for all buffers to be returned to the pool.
  for (bool host : {false, true}) {
    CUDABufferPool*pool = new CUDABufferPool(100, 10, host);
    std::atomic_int count = 0;
    std::atomic_bool running(false);

    // Start a thread that will allocate one buffer per tenth of a second, asking for ten buffers
    std::thread getThread(TestBufferPoolAllocationThread, pool, 0.1, 10, &count, &running, true);
    while (!running) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // Wait half a second and then destroy the pool.  The thread should continue to run
    // and try to allocate buffers, but the pool should not allocate any more buffers.
    std::this_thread::sleep_for(std::chrono::microseconds(500000));
    delete pool;
    getThread.join();

    // Verify that the thread allocated some but not all of the buffers.
    if ((count == 0) || (count == 10)) {
      return "Multi-threaded test failed: worked = " + std::to_string(count);
    }
  }

  /// Test that the read thread waits rather than allocating new buffers when it exhausts a pool.
  /// Also test that it gets buffers that are returned.
  for (bool host : {false, true}) {
    CUDABufferPool*pool = new CUDABufferPool(100, 10, host);
    std::atomic_int count = 0;
    std::atomic_bool running(false);

    // Start a thread that will allocate one buffer per tenth of a second, asking for ten buffers
    // but will not allocate new buffers, so will hang.
    std::thread getThread(TestBufferPoolAllocationThread, pool, 0.1, 10, &count, &running, false);
    while (!running) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // Wait half a second and then allocate a buffer.  The thread should continue to run
    // and try to allocate buffers, but the pool should not allocate all required buffers.
    std::this_thread::sleep_for(std::chrono::microseconds(500000));
    std::shared_ptr<uint8_t> buf = pool->GetBuffer(false, 1000);

    // Wait a full second, and the thread should have allocated all buffers but should only have
    // allocated 9 buffers.
    std::this_thread::sleep_for(std::chrono::seconds(1));
    if (count != 9) {
      return "Multi-threaded non-allocating test failed: count = " + std::to_string(count);
    }

    // Now return the entry and wait half a second.  The thread should have allocated the last buffer.
    buf.reset();
    std::this_thread::sleep_for(std::chrono::microseconds(500000));
    if (count != 10) {
      return "Multi-threaded non-allocating test failed: count = " + std::to_string(count);
    }

    // Clean up.
    delete pool;
    getThread.join();
  }

  // Everything worked.
  return "";
}
