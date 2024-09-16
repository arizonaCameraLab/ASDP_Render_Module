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

std::shared_ptr<ImageData> ImageQueue::GetRenderImage()
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

void ImageQueue::ReturnRenderImage(std::shared_ptr<ImageData> image)
{
  std::shared_ptr<ImageData> imagePtr = image;

  // Add it to the front of the queue if it is newer than the image that is there, or the back of the queue otherwise
  std::lock_guard<std::mutex> lock(m_mutex);
  if (image->imageCenterTime > m_images.front()->imageCenterTime) {
    m_images.push_front(std::move(imagePtr));
  } else {
    m_images.push_back(std::move(imagePtr));
  }
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
  std::shared_ptr<ImageData> newestImage = imageQueue.GetRenderImage();
  if (newestImage != nullptr) {
    return "Got newest image from empty queue.";
  }

  // Create an image
  std::shared_ptr<ImageData> image = std::make_shared<ImageData>();

  // Add the image to the queue
  imageQueue.AddNewestImage(image);

  // Get the newest image from the queue
  newestImage = imageQueue.GetRenderImage();
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
  std::shared_ptr<ImageData> newestImage2 = imageQueue.GetRenderImage();
  std::shared_ptr<ImageData> oldestImage2 = imageQueue.PopOldestImage();
  if (oldestImage2 == nullptr) {
    return "Failed to get oldest image from queue with queue length 2.";
  }
  if (newestImage2.get() == oldestImage2.get()) {
    return "Newest image is the same as the oldest image.";
  }

  // Push two images onto the queue whose times are different and then ensure that
  // the image that is re-added to the queue is put in the correct location (front when
  // it is newer, back when it is older).
  std::shared_ptr<ImageData> image1 = std::make_shared<ImageData>();
  std::shared_ptr<ImageData> image2 = std::make_shared<ImageData>();
  std::shared_ptr<ImageData> image3 = std::make_shared<ImageData>();
  image1->imageCenterTime.seconds = 1;
  image2->imageCenterTime.seconds = 2;
  image3->imageCenterTime.seconds = 3;

  imageQueue.AddNewestImage(image2);
  imageQueue.AddNewestImage(image3);
  std::shared_ptr<ImageData> renderImage = imageQueue.GetRenderImage();
  imageQueue.ReturnRenderImage(renderImage);
  if (imageQueue.m_images.front().get() != image3.get()) {
    return "Failed to put newer image in front of queue.";
  }
  imageQueue.ReturnRenderImage(image1);
  if (imageQueue.m_images.back().get() != image1.get()) {
    return "Failed to put older image in back of queue.";
  }

  return "";
}
