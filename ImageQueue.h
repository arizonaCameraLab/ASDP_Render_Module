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
      asdp::Time imageCenterTime = {};
    };

    /// @brief Thread-safe access to pool of images along with times they were created.
    /// @details This class is used to store images and their creation times.  It is used
    /// by a creator thread to store images and by a consumer thread to retrieve image pointers.
    /// The consumer thread will get a pointer to the newest image without pulling it off
    /// the queue and it releases the shared pointer when it is done with the image.
    /// The creator thread pulls the oldest image off the queue
    /// and replaces it with a new image, which becomes the one to be rendered.
    /// This class will not pop the last image off the queue to avoid pulling the rug out
    /// from under a consumer thread that is still using the image.
    /// Remember to make a different OpenGL context for each thread that will be using
    /// this class and have them share texture resources.
    class ImageQueue {
    public:
      ImageQueue() = default;
      virtual ~ImageQueue() = default;

      /// @brief Add an image to the queue as the next to be rendered.
      /// @details This function adds an image to the queue.  The image becomes the
      /// newest image in the queue, the one that will be rendered next.
      /// @param[in] image Image to add to the queue.
      void AddNewestImage(std::shared_ptr<ImageData> image);

      /// @brief Get the oldest image in the queue, to be overwritten and then rendered.
      /// @return Shared pointer to the oldest image in the queue.  Null pointer if the
      /// queue has less than two elements.  The entry is removed from the queue.  Returns
      /// a null pointer if the queue is empty.
      std::shared_ptr<ImageData> PopOldestImage();

      /// @brief Get the newest image in the queue, to be rendered.
      /// @return Shared pointer to the newest image in the queue.
      /// The entry is removed from the queue.  Returns a null pointer if the queue is empty.
      std::shared_ptr<ImageData> PopNewestImage();

      /// @brief Add an image to the queue as the oldest, first to be replaced.
      /// @details This function adds an image to the queue.  The image becomes the
      /// newest image in the queue, the one that will be rendered next.
      /// @param[in] image Image to add to the queue.
      void AddOldestImage(std::shared_ptr<ImageData> image);

      /// @brief Test function to test the ImageQueue class.
      /// @return Empty string on success, string with error message on failure.
      static std::string Test();

      /// @brief Get the number of images in the queue.
      size_t size() const;

    protected:
      /// @brief Mutex to protect access to the image queue.
      mutable std::mutex m_mutex;

      /// @brief The images associated with this queue.
      std::list< std::shared_ptr<ImageData> > m_images;
    };

  } // namespace render
} // namespace asdp