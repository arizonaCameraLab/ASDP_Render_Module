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

std::shared_ptr<ImageData> ImageQueue::GetOldestImage()
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

std::list< std::shared_ptr<ImageData> > ImageQueue::GetNewestImages(size_t count)
{
  std::list< std::shared_ptr<ImageData> > images;
  std::lock_guard<std::mutex> lock(m_mutex);
  while (!m_images.empty() && (count > 0)) {
    images.push_back(m_images.front());
    m_images.pop_front();
    --count;
  }
  return images;
}

void ImageQueue::InsertImage(std::shared_ptr<ImageData> image)
{
  std::shared_ptr<ImageData> imagePtr = image;

  // Insert the image in time-sorted order, with the newest at the fron and the oldest
  // at the back.
  std::lock_guard<std::mutex> lock(m_mutex);
  auto element = m_images.begin();
  while (element != m_images.end() && (*element)->imageCenterTime > image->imageCenterTime) {
    ++element;
  }
  m_images.insert(element, std::move(imagePtr));
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
  std::shared_ptr<ImageData> oldestImage = imageQueue.GetOldestImage();
  if (oldestImage != nullptr) {
    return "Got oldest image from empty queue.";
  }
  auto newestImages = imageQueue.GetNewestImages();
  if (newestImages.size() != 0) {
    return "Got newest image from empty queue.";
  }

  // Create an image
  std::shared_ptr<ImageData> image = std::make_shared<ImageData>();

  // Add the image to the queue
  imageQueue.InsertImage(image);

  // Get the newest image from the queue
  newestImages = imageQueue.GetNewestImages();
  if (newestImages.size() != 1) {
    return "Failed to get newest image from queue.";
  }

  // We should fail to get the oldest image from the queue because there is are no images in the queue
  oldestImage = imageQueue.GetOldestImage();
  if (oldestImage != nullptr) {
    return "Incorrectly able to get oldest image from queue.";
  }

  // Add two images to the queue
  imageQueue.InsertImage(std::make_shared<ImageData>());
  imageQueue.InsertImage(std::make_shared<ImageData>());

  // Verify that the queue size is 2
  if (imageQueue.size() != 2) {
    return "Queue size is not 2 after adding two images.";
  }

  // Get the newest and oldest images from the queue.
  // Verify that the newest image is not the same as the oldest image
  auto newestImages2 = imageQueue.GetNewestImages();
  std::shared_ptr<ImageData> oldestImage2 = imageQueue.GetOldestImage();
  if (oldestImage2 == nullptr) {
    return "Failed to get oldest image from queue with queue length 2.";
  }
  if (newestImages2.empty() || (newestImages2.front().get() == oldestImage2.get())) {
    return "Newest image is empty or the same as the oldest image.";
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

  imageQueue.InsertImage(image2);
  imageQueue.InsertImage(image3);
  auto renderImages = imageQueue.GetNewestImages();
  if (renderImages.size() != 1) {
    return "Failed to get newest image from queue with queue length 3.";
  }
  imageQueue.InsertImage(renderImages.front());
  if (imageQueue.m_images.front().get() != image3.get()) {
    return "Failed to put newer image in front of queue.";
  }
  imageQueue.InsertImage(image1);
  if (imageQueue.m_images.back().get() != image1.get()) {
    return "Failed to put older image in back of queue.";
  }

  // Test getting multiple newest images from the queue
  renderImages = imageQueue.GetNewestImages(3);
  if (renderImages.size() != 3) {
    return "Failed to get 3 newest images from queue.";
  }

  return "";
}
