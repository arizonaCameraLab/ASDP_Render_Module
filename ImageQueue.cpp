/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include <ImageQueue.h>
using namespace asdp::render;

ImageData::~ImageData()
{
  if (texture != 0) {
    glDeleteTextures(1, &texture);
  }
}

void ImageQueue::AddNewestImage(std::shared_ptr<ImageData> image)
{
  std::shared_ptr<ImageData> imagePtr = image;

  // Add it to the front of the queue
  std::lock_guard<std::mutex> lock(m_mutex);
  m_images.push_front(imagePtr);
}

std::shared_ptr<ImageData> ImageQueue::PopOldestImage()
{
  std::lock_guard<std::mutex> lock(m_mutex);
  if (m_images.size() == 0) {
    return nullptr;
  } else {
    std::shared_ptr<ImageData> image = m_images.back();
    m_images.pop_back();
    return image;
  }
}

std::shared_ptr<ImageData> ImageQueue::PopNewestImage()
{
  std::lock_guard<std::mutex> lock(m_mutex);
  if (m_images.size() == 0) {
    return nullptr;
  } else {
    std::shared_ptr<ImageData> image = m_images.front();
    m_images.pop_front();
    return image;
  }
}

void ImageQueue::AddOldestImage(std::shared_ptr<ImageData> image)
{
  std::shared_ptr<ImageData> imagePtr = image;

  // Add it to the back of the queue
  std::lock_guard<std::mutex> lock(m_mutex);
  m_images.push_back(imagePtr);
}

size_t ImageQueue::size() const
{
  std::lock_guard<std::mutex> lock(m_mutex);
  return m_images.size();
}

std::string ImageQueue::Test()
{
  // Create an image queue
  ImageQueue imageQueue;

  // Verify that the queue size is 0 at the start
  if (imageQueue.size() != 0) {
    return "Queue size is not 0 at the start.";
  }

  // Verify that we can't get the oldest and newest image from an empty queue
  std::shared_ptr<ImageData> oldestImage = imageQueue.PopOldestImage();
  if (oldestImage != nullptr) {
    return "Got oldest image from empty queue.";
  }
  std::shared_ptr<ImageData> newestImage = imageQueue.PopNewestImage();
  if (newestImage != nullptr) {
    return "Got newest image from empty queue.";
  }

  // Create an image
  std::shared_ptr<ImageData> image = std::make_shared<ImageData>();

  // Add the image to the queue
  imageQueue.AddNewestImage(image);

  // Get the newest image from the queue
  newestImage = imageQueue.PopNewestImage();
  if (newestImage == nullptr) {
    return "Failed to get newest image from queue.";
  }

  // We should fail to get the oldest image from the queue because there is are no images in the queue
  oldestImage = imageQueue.PopOldestImage();
  if (oldestImage != nullptr) {
    return "Incorrectly able to get oldest image from queue.";
  }

  // Add two images to the queue
  imageQueue.AddNewestImage(std::make_shared<ImageData>());
  imageQueue.AddNewestImage(std::make_shared<ImageData>());

  // Verify that the queue size is 2
  if (imageQueue.size() != 2) {
    return "Queue size is not 2 after adding two images.";
  }

  // Get the newest and oldest images from the queue.
  // Verify that the newest image is not the same as the oldest image
  std::shared_ptr<ImageData> newestImage2 = imageQueue.PopNewestImage();
  std::shared_ptr<ImageData> oldestImage2 = imageQueue.PopOldestImage();
  if (oldestImage2 == nullptr) {
    return "Failed to get oldest image from queue with queue length 2.";
  }
  if (newestImage2.get() == oldestImage2.get()) {
    return "Newest image is the same as the oldest image.";
  }

  return "";
}
