/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

 /**
  * @file ImageQueue.h
  * @brief Apache Strap-Down Pilotage Render/ImageQueue class header file.
  *
  * @author ReliaSolve.
  * @date April 29, 2024.
  */

#pragma once
#include <mutex>
#include <list>
#include <memory>
#include <string>
#include <ASDP_Core_API.h>

#ifdef WIN32
#include <Windows.h>
#endif
#include <GL/gl.h>

namespace asdp {
  namespace render {

    /// @brief Stores an OpenGL texture ID and the time the image was read into the system.
    struct ImageData {
      /// @brief Destructor deletes the texture when the ImageData is destroyed.
      virtual ~ImageData();

      /// @brief Image data stored in an OpenGL texture.
      /// @details This struct takes "ownership" of the texture, in the sense that
      /// the texture will be deleted when the ImageData is destroyed.  It should be
      /// created as a shared_ptr and passed around as a shared_ptr to avoid deleting
      /// the texture while it is still being used.
      GLuint texture = 0;

      /// @brief Time the middle of the image was read into the system.
      /// @details This time is used to determine how much to shift the image based
      /// on the change in pose of the helicopter since the image was taken.
      asdp::Time imageCenterTime;

      /// @brief Duration of the image capture in microseconds.
      /// @details This is the scan-out time of the image, from the time the first pixel is read
      /// until the time the last pixel is read. For a global shutter camera, this will be zero.
      uint32_t imageDurationMicroseconds = 0;

      /// @brief The gain used to capture the image.
      float gain = 0.0f;

      /// @brief The exposure time used to capture the image.
      float exposure = 0.0f;
    };

    /// @brief Thread-safe access to pool of images along with times they were created.
    /// @details This class is used to store images and their creation times.  It is used
    /// by a creator thread to store images and by a consumer thread to lock groups of
    /// image pointers and then unlock them when they are done using them.
    /// The creator thread pulls the oldest image off the queue and replaces it with a new image.
    /// Remember to make a different OpenGL context for each thread that will be using
    /// this class and have them share texture resources.
    class ImageQueue {
    public:
      ImageQueue() = default;
      virtual ~ImageQueue() = default;

      /// @brief Add an image to the queue in time-sorted order.
      /// @param[in] image Image to add to the queue.
      void InsertImage(std::shared_ptr<ImageData> image);

      /// @brief Pop the oldest image off the queue, to be overwritten with new data.
      /// @return Shared pointer to the oldest image in the queue.  Null pointer if the
      /// queue has no unlocked elements.  The entry is removed from the queue.
      std::shared_ptr<ImageData> GetOldestImage();

      /// @brief Lock the newest images in the queue and return their pointers.
      /// @return Shared pointers to the newest images in the queue.
      /// Returns a short or empty list if there are not enough elements.
      std::list< std::shared_ptr<ImageData> > LockNewestImages(size_t count = 1);

      /// @brief Unlock an image that was previously locked.
      /// @param[in] image Image to unlock.
      /// @return True if the image was successfully unlocked, false if the image was not found
      /// or is not locked.
      bool UnlockImage(std::shared_ptr<ImageData> image);

      /// @brief Test function to test the ImageQueue class.
      /// @return Empty string on success, string with error message on failure.
      static std::string Test();

      /// @brief Get the number of images in the queue.
      size_t size() const;

    protected:
      /// @brief Mutex to protect access to the image queue.
      mutable std::mutex m_mutex;

      /// @brief Element in the image queue and its reference count.
      struct ImageQueueElement {
        std::shared_ptr<ImageData> image;     ///< Image data.
        unsigned refCount = 0;                ///< Reference count for the image.
      };

      /// @brief The images associated with this queue.
      std::list<ImageQueueElement> m_images;
    };

  } // namespace render
} // namespace asdp