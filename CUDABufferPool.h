/*
 * Copyright (C) 2025: Arizona Board of Regents on Behalf of the University of Arizona
 */

#pragma once

/**
* @file CUDABufferPool.h
* @brief Apache Strap-Down Pilotage utility class to provide a pre-allocated pool of CUDA buffers (CPU or GPU side).
*
* @author ReliaSolve.
* @date September 18th, 2024.
*/

#include <vector>
#include <list>
#include <memory>
#include <mutex>
#include <cstdint>
#include <string>
#include <atomic>

namespace asdp {
  namespace render {

    /// @brief Manages a thread-safe pre-allocated pool of CUDA pinned CPU buffers or GPU buffers.
    class CUDABufferPool {
    public:
      /// @brief Constructs a buffer pool with the given buffer size and initial number of buffers.
      /// @param bufferSize The size of each buffer in bytes.
      /// @param bufferCount The initial number of buffers in the pool.
      /// @PARAM host If true, the buffers will be allocated in pinned memory on the host (CPU) side.
      ///        Otherwise, they will be allocated on the GPU side.
      /// @throw std::runtime_error if the buffer pool cannot be created.
      CUDABufferPool(size_t bufferSize, size_t bufferCount, bool host);

      /// @brief Destroys the buffer pool after waiting for all outstanding buffers to return to the pool.
      ~CUDABufferPool();

      /// @brief Returns a buffer from the pool, or nullptr if the pool is being destroyed.
      /// @details Returns a buffer from the pool, creating a new buffer if necessary.
      /// When the shared_ptr is destroyed, the buffer is automatically returned to the pool.
      /// The nullptr is returned if the pool is being destroyed.
      /// This method is thread-safe.
      /// @param allocateWhenEmpty Should we allocate more buffers when the pool is empty? Otherwise, wait
      /// for buffers to be returned to the pool.
      /// @param timeoutMilli The maximum time to wait for a buffer to be returned to the pool, in milliseconds.
      /// @return A buffer from the pool, or nullptr if the pool is being destroyed.
      /// @throw std::runtime_error if a buffer cannot be created.
      std::shared_ptr<uint8_t> GetBuffer(bool allocateWhenEmpty, size_t timeoutMilli);

      /// @brief Test the CUDABufferPool class.
      /// @return Empty string on success, descriptive error message on failure.
      static std::string Test();

    private:
      /// Whether we're allocating buffers on the host (CPU) side or the GPU side.
      bool m_host;

      /// The size of each buffer in bytes.
      size_t m_bufferSize;

      /// A list of all buffers thave have been allocated, whether they are free or have been loaned out.
      std::list<uint8_t*> m_allBuffers;

      /// A list of pointers to buffers that are free (have not been loaned out).
      std::list<uint8_t*> m_freeBuffers;

      /// Mutex to guard access to our data structures to make this class thread-safe.
      std::mutex mtx;

      /// Set to true when we're in the process of destroying the pool.  Prevents new
      /// buffers from being allocated.
      std::atomic_bool m_done;
    };

  } // namespace render
} // namespace asdp
