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
    if (m_images.back().refCount > 0) {
      return nullptr;
    } else {
      std::shared_ptr<ImageData> image = m_images.back().image;
      m_images.pop_back();
      return image;
    }
  }
}

std::list< std::shared_ptr<ImageData> > ImageQueue::LockNewestImages(size_t count)
{
  std::list< std::shared_ptr<ImageData> > images;
  std::lock_guard<std::mutex> lock(m_mutex);
  auto it = m_images.begin();
  while ((it != m_images.end()) && (count > 0)) {
    images.push_back(it->image);
    it->refCount++;
    --count;
    ++it;
  }
  return images;
}

void ImageQueue::InsertImage(std::shared_ptr<ImageData> image)
{
  ImageQueueElement ins;
  ins.image = image;

  // Insert the image in time-sorted order, with the newest at the front and the oldest
  // at the back.
  std::lock_guard<std::mutex> lock(m_mutex);
  auto element = m_images.begin();
  while (element != m_images.end() && element->image->imageCenterTime >= image->imageCenterTime) {
    ++element;
  }
  m_images.insert(element, std::move(ins));
}

bool ImageQueue::UnlockImage(std::shared_ptr<ImageData> image)
{
  std::lock_guard<std::mutex> lock(m_mutex);
  for (auto& element : m_images) {
    if (element.image == image) {
      if (element.refCount > 0) {
        element.refCount--;
        return true;
      }
    }
  }
  return false;
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
  auto newestImages = imageQueue.LockNewestImages();
  if (newestImages.size() != 0) {
    return "Got newest image from empty queue.";
  }

  // Create an image
  std::shared_ptr<ImageData> image = std::make_shared<ImageData>();

  // Add the image to the queue
  imageQueue.InsertImage(image);

  // Lock the newest image in the queue
  newestImages = imageQueue.LockNewestImages();
  if (newestImages.size() != 1) {
    return "Failed to lock newest image in queue.";
  }

  // We should fail to get the oldest image from the queue because there
  // are no unlocked images in the queue.
  oldestImage = imageQueue.GetOldestImage();
  if (oldestImage != nullptr) {
    return "Incorrectly able to get oldest image from queue.";
  }

  // Unlock the newest image in the queue
  if (!imageQueue.UnlockImage(newestImages.front())) {
    return "Failed to unlock newest image in queue.";
  }

  // Add another image to the queue
  imageQueue.InsertImage(std::make_shared<ImageData>());

  // Verify that the queue size is 2
  if (imageQueue.size() != 2) {
    return "Queue size is not 2 after adding two images.";
  }

  // Lock the newest and get the oldest image from the queue.
  // Verify that the newest image is not the same as the oldest image
  auto newestImages2 = imageQueue.LockNewestImages();
  std::shared_ptr<ImageData> oldestImage2 = imageQueue.GetOldestImage();
  if (oldestImage2 == nullptr) {
    return "Failed to get oldest image from queue with queue length 2.";
  }
  if (newestImages2.empty() || (newestImages2.front().get() == oldestImage2.get())) {
    return "Newest image is empty or the same as the oldest image.";
  }
  if (!imageQueue.UnlockImage(newestImages2.front())) {
    return "Failed to unlock newest image in second queue.";
  }

  // Clear the queue by popping all images.
  if (imageQueue.GetOldestImage() == nullptr) {
    return "Failed to pop image from queue.";
  }
  if (imageQueue.size() != 0) {
    return "Queue size is not 0 after popping all images.";
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
  std::shared_ptr<ImageData> renderImage = imageQueue.GetOldestImage();
  if (renderImage == nullptr) {
    return "Failed to get image from queue with queue length 3.";
  }
  imageQueue.InsertImage(renderImage);
  if (imageQueue.m_images.front().image.get() != image3.get()) {
    return "Failed to put newer image in front of queue.";
  }
  imageQueue.InsertImage(image1);
  if (imageQueue.m_images.back().image.get() != image1.get()) {
    return "Failed to put older image in back of queue.";
  }

  // Test locking multiple newest images in the queue
  std::list< std::shared_ptr<ImageData> > renderImages = imageQueue.LockNewestImages(3);
  if (renderImages.size() != 3) {
    return "Failed to lock 3 newest images from queue.";
  }

  // Test double locking the same images.
  renderImages = imageQueue.LockNewestImages(3);
  if (renderImages.size() != 3) {
    return "Failed to lock 3 newest images from queue.";
  }
  for (auto const & image : imageQueue.m_images) {
    if (image.refCount != 2) {
      return "Failed to double lock images in queue.";
    }
  }

  // Test unlocking just once
  for (auto const & image : renderImages) {
    if (!imageQueue.UnlockImage(image)) {
      return "Failed to unlock image in queue.";
    }
  }
  for (auto const& image : imageQueue.m_images) {
    if (image.refCount != 1) {
      return "Failed to singe unlock images in queue.";
    }
  }

  return "";
}
