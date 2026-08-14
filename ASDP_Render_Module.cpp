/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

// This is a client that connects to the first server it encounters and runs a Render Module.

/**
 * @file ASDP_Render_Module.cpp
 * @brief Apache Strap-Down Pilotage Render Module.
 *
* @author ReliaSolve.
* @date May 20th, 2024.
*/

#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include <map>
#include <set>
#include <mutex>
#include <thread>
#include <string>
#include <filesystem>
#include <vector>
#include <list>
#include <atomic>
#include <memory>
#include <algorithm>
#include <ASDP_Core_API.h>
#include <ASDP_SpinFreeQueue.hpp>
#include <ASDP_BufferPool.h>
#include <ASDP_ClockSynchronizer.h>
#include "CUDABufferPool.h"
#include <nlohmann/json.hpp>
#include <GL/glew.h>
#include <ToneMap.h>
#include <RenderTimingInfo.h>
#include <CameraRenderInfo.h>
#include <Composite.h>
#include <Display.h>
#include <CPUDataToTextureHandler.h>
#include <PoseAdjuster.h>
#include <DepthEstimator.h>
#include <ImageStatistics.h>
#include <RangeEstimator.h>
#include <Calibration_Helpers.h>
#include <PointCorrespondences.h>
#include <Analysis.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>

using namespace asdp;
using namespace asdp::render;
using namespace asdp::analysis;
using json = nlohmann::json;

static std::string VERSION = "3.38.0";

/// @brief The path to the configuration file. Defined in the CMakeLists file.
std::filesystem::path g_dirPath = CONFIG_FILE_PATH;

/// @brief Structure storing information needed by the callback handlers, a pointer is passeed in userData.
struct CallbackHandlerData {
  std::string cameraConfigFileName; ///< The name of the configuration file that was read and parsed.
  std::atomic_int analysisEpoch{ 0 }; ///< The current epoch of the analysis data, incremented to reset analysis.
};
static CallbackHandlerData g_callbackHandlerData;

/// @brief Global variable set by callback handlers to tell when we're playing and pausing.
static std::atomic<bool> g_paused(false);

/// @brief Global variable to hold the timing information for the program.
static asdp::render::RenderTimingInfo g_timingInfo;

/// @brief Global variable to hold the depth estimator.
static std::shared_ptr<DepthEstimator> g_depthEstimator;

/// @brief Global variables to hold the visible and depth cameras.
static std::vector< std::shared_ptr<asdp::render::CameraRenderInfo> > g_visibleCameras, g_depthCameras;

/// @brief Global variable to hold the index of the active camera.
static std::atomic<size_t> g_activeCameraIndex(0);

/// @brief Global variable to hold the composite camera used by each display.
static std::vector< std::shared_ptr<CompositeCameras> > g_composites;

/// @brief Global variable to hold the display used to provide a context for point correspondences.
static std::shared_ptr<Display> g_pointCorrespondenceDisplay;

/// @brief Global variable to hold the point correspondences object.
static std::shared_ptr<PointCorrespondences> g_pointCorrespondences;

/// @brief Global variable to hold the display used to handle kiosk commands.
static std::shared_ptr<Display> g_kioskDisplay;

/// @brief Vector of camera pairs to use for auto-updating color offsets.
/// @todo Eventually, this should be determined from the geometry of the camera configuration.
static std::vector< std::array<uint32_t, 2> > g_cameraPairs = {
  {11, 14}, {14, 17}, {17, 20},     // Right along the center row
  {11, 8},  {8, 5},   {5, 2},       // Left along the center row
  {2, 1}, {5, 4}, {8, 7}, {11, 10}, {14, 13}, {17, 16}, {20, 19},     // Top row
  {2, 3}, {5, 6}, {8, 9}, {11, 12}, {14, 15}, {17, 18}, {20, 21}      // Bottom row
};

/// @brief Atomic shared pointer to a map from name to the current and past history of active objects.
/// Filled in by the Analysis API reading thread and used by the annotation callback.
static std::shared_ptr< std::map<std::string, AnalysisObjectOverTime> > g_currentAnalysis;

static float g_analysisFadeTimeSeconds = 1.0f;  ///< Time in seconds for analysis annotations to fade out.
static float g_analysisChanceThreshold = 0.0f; ///< Minimum chance threshold for analysis annotations to be shown.

static std::atomic<Time> g_lastCLOCK_SYNC = { 0 };  ///< The last CLOCK_SYN message time received, used to adjust analysis displays
static_assert(std::is_trivially_copyable<Time>::value, "Time must be trivially copyable to use std::atomic<Time> portably");

/// @brief Vector of Annotation objects to hold camera annotations if they are shown.
static std::vector<CompositeCameras::Annotation> g_cameraAnnotations;

/// @brief Atomic boolean to control the analysis thread.
static std::atomic_bool g_runAnalysisThread;

/// @brief Mutex to protect access to the current annotations.
static std::mutex g_annotationMutex;

/// @brief Atomic boolean to control the depth thread.
static std::atomic_bool g_runDepthThread;

/// @brief Thread for running depth estimation.
static std::thread g_depthThread;

/// @brief Callback handler to increment the active camera index.
static void IncrementActiveCamera(void* /* unused */)
{
  g_activeCameraIndex++;
  if (g_activeCameraIndex >= g_visibleCameras.size()) {
    g_activeCameraIndex = 0;
  }
  std::cout << "Incremented active camera index to " << g_activeCameraIndex.load();
  if (g_activeCameraIndex < g_visibleCameras.size()) {
    std::cout << " (ID = " << g_visibleCameras[g_activeCameraIndex]->m_ID << ")";
  }
  std::cout << std::endl;
}

/// @brief Callback handler to decrement the active camera index.
static void DecrementActiveCamera(void* /* unused */)
{
  if (g_activeCameraIndex == 0) {
    g_activeCameraIndex = g_visibleCameras.size() - 1;
  } else {
    g_activeCameraIndex--;
  }
  std::cout << "Decremented active camera index to " << g_activeCameraIndex.load();
  if (g_activeCameraIndex < g_visibleCameras.size()) {
    std::cout << " (ID = " << g_visibleCameras[g_activeCameraIndex]->m_ID << ")";
  }
  std::cout << std::endl;
}

/// @brief Callback handler to toggle play and pause.
static void ChangePlayPause(bool nowPlaying, void* /* unused */)
{
  g_paused = !nowPlaying;
  std::cout << "Toggled play/pause to: " << (g_paused ? "paused" : "playing") << std::endl;
}

/// @brief Callback handler to toggle showing camera names.
static void ShowCameraNames(bool showNames, void* /* unused */)
{
  std::lock_guard<std::mutex> lock(g_annotationMutex);
  g_cameraAnnotations.clear();
  if (showNames) {
    CompositeCameras::Annotation annotation;
    annotation.uv = { 0.5, 0.5 };       // Center of the image
    annotation.color = { 1.0f, 1.0f, 0.0f, 1.0f };  // Yellow and fully opaque
    for (auto const& cameraRenderInfo : g_visibleCameras) {
      annotation.cameraID = cameraRenderInfo->m_ID;
      annotation.label = "CamID: " + std::to_string(cameraRenderInfo->m_ID);
      g_cameraAnnotations.push_back(annotation);
    }
  }
  std::cout << "Toggled camera names to: " << (showNames ? "shown" : "hidden") << std::endl;
}

/// @brief Adjust the active camera's color offset calibration value.
/// @param offsetDelta The amount to add to the offset, positive or negative.
static void AdjustActiveCameraOffset(int offsetDelta, void* /* unused */)
{
  if (g_activeCameraIndex >= g_visibleCameras.size()) {
    std::cerr << "Error: Active camera index out of range." << std::endl;
    return;
  }
  std::shared_ptr<asdp::render::CameraRenderInfo> cri = g_visibleCameras[g_activeCameraIndex];
  if (cri == nullptr) {
    std::cerr << "Error: Active camera is null." << std::endl;
    return;
  }
  
  // Adjust the color offset and gain.
  float currentOffset, currentGain;
  cri->GetColorOffsetGain(currentOffset, currentGain);
  cri->SetColorOffsetGain(currentOffset + offsetDelta, currentGain);
  std::cout << "Adjusted camera ID " << cri->m_ID << " color offset to : " << currentOffset + offsetDelta << std::endl;
}

/// @brief Adjust the active camera's color gain calibration value.
/// @param gainDelta The amount to multiply the gain by, <1 or >1.
static void AdjustActiveCameraGain(float gainDelta, void* /* unused */)
{
  if (g_activeCameraIndex >= g_visibleCameras.size()) {
    std::cerr << "Error: Active camera index out of range." << std::endl;
    return;
  }
  std::shared_ptr<asdp::render::CameraRenderInfo> cri = g_visibleCameras[g_activeCameraIndex];
  if (cri == nullptr) {
    std::cerr << "Error: Active camera is null." << std::endl;
    return;
  }

  // Adjust the color offset and gain.
  float currentOffset, currentGain;
  cri->GetColorOffsetGain(currentOffset, currentGain);
  cri->SetColorOffsetGain(currentOffset, currentGain * gainDelta);
  std::cout << "Adjusted camera ID " << cri->m_ID << " color gain to : " << currentGain * gainDelta << std::endl;
}

static double TimeDiffMagnitude(asdp::Time t1, asdp::Time t2)
{
  asdp::Time diff;
  if (t1 > t2) {
    diff = t1 - t2;
  } else {
    diff = t2 - t1;
  }
  return diff.seconds + diff.microseconds * 1.0e-6;
}

/// @brief Get a consistent set of images from all visible cameras for color offset adjustment.
/// @return Vector of shared pointers to ImageData objects, one from each visible camera.  The
/// caller is responsible for unlocking the images when done using them by calling
/// UnlockConsistentImageSet() and passing it this return vector.
/// Note: If not enough images are available, an empty vector is returned.

static std::vector< std::shared_ptr<ImageData> > GetConsistentImageSet()
{
  std::vector< std::shared_ptr<ImageData> > imageSet;

  // Pull the first two images from each queue and then select a set of consistent ones.
  std::vector< std::list< std::shared_ptr<ImageData> > > images;
  for (auto const& cameraRenderInfo : g_visibleCameras) {
    images.push_back(cameraRenderInfo->m_imageQueue->LockNewestImages(2));
    if (images.back().size() != 2) {
      std::cerr << "GetConsistentImageSet(): Could not get all needed images, skipping frame" << std::endl;
      for (auto const& imList : images) {
        for (auto const& image : imList) {
          cameraRenderInfo->m_imageQueue->UnlockImage(image);
        }
      }
      return imageSet;
    }
  }

  // Find the time of the oldest image among the first (newest) image from
  // all cameras and then selecting from each pair the one whose time is closest to the
  // selected time.
  asdp::Time desiredTime = images[0].front()->imageCenterTime;
  for (size_t i = 1; i < images.size(); i++) {
    if (images[i].front()->imageCenterTime < desiredTime) {
      desiredTime = images[i].front()->imageCenterTime;
    }
  }

  // Find the image from each list that is closest to the desired time.  Push it into the m_images
  // array and return the other images+/ to the queue.
  for (size_t i = 0; i < images.size(); i++) {
    auto& imList = images[i];
    auto best = imList.begin();
    double bestDiff = TimeDiffMagnitude((*best)->imageCenterTime, desiredTime);
    for (auto it = imList.begin(); it != imList.end(); ++it) {
      double diff = TimeDiffMagnitude((*it)->imageCenterTime, desiredTime);
      if (diff < bestDiff) {
        best = it;
        bestDiff = diff;
      }
    }

    for (auto it = imList.begin(); it != imList.end(); ++it) {
      if (it == best) {
        // Use this image
        imageSet.push_back(*it);
      } else {
        // Unlock the images that are not selected.
        g_visibleCameras[i]->m_imageQueue->UnlockImage(*it);
      }
    }
  }

  return imageSet;
}

/// @brief Unlock a consistent set of images previously obtained by calling GetConsistentImageSet().
/// @param imageSet The vector of shared pointers to ImageData objects obtained from GetConsistentImageSet().
static void UnlockConsistentImageSet(const std::vector< std::shared_ptr<ImageData> >& imageSet)
{
  for (size_t i = 0; i < imageSet.size(); i++) {
    g_visibleCameras[i]->m_imageQueue->UnlockImage(imageSet[i]);
  }
}

/// @brief Make a vector of pairs of pixel values, one from each image, read at the specified correspondence locations.
/// @param correspondences Locations from first and second image to read.
/// @param widths Array of 2 image widths.
/// @param heights Array of 2 image heights.
/// @param imageData Array of 2 images to read data from.
static std::vector< std::array<uint16_t, 2> > GetRawPixelValues(
  const std::vector<PointCorrespondences::PointPair>& correspondences,
  std::array<uint16_t, 2> widths, std::array<uint16_t, 2> heights,
  std::array< std::shared_ptr<ImageData>, 2> imageData)
{
  // Borrow the OpenGL context so that we can read back the pixel values.
  if (!g_pointCorrespondenceDisplay || !g_pointCorrespondenceDisplay->BorrowContext()) {
    std::cerr << "GetRawPixelValues(): Error: Could not borrow OpenGL context." << std::endl;
    return {};
  }

  // Read back both images from texture memory to CPU memory.  These have already been adjusted by
  // the offset and gain on the way to being written to the texture.
  std::array<std::vector<uint16_t>, imageData.size()> imagePixels;
  GLenum ret;
  for (int i = 0; i < imagePixels.size(); i++) {
    imagePixels[i].resize(widths[i] * heights[i]);
    glBindTexture(GL_TEXTURE_2D, imageData[i]->texture);
    glGetTexImage(GL_TEXTURE_2D, 0, GL_RED, GL_UNSIGNED_SHORT, imagePixels[i].data());
#if !defined(NDEBUG)
    ret = glGetError();
    if (ret != GL_NO_ERROR) {
      std::cerr << "GetRawPixelValues(): Error: glGetTexImage() failed for image " << i
        << " with error code " << ret << std::endl;
      glBindTexture(GL_TEXTURE_2D, 0);
      g_pointCorrespondenceDisplay->ReturnContext();
      return {};
    }
#endif
  }
  glBindTexture(GL_TEXTURE_2D, 0);

  // Return the OpenGL context.
  if (!g_pointCorrespondenceDisplay || !g_pointCorrespondenceDisplay->ReturnContext()) {
    std::cerr << "GetRawPixelValues(): Error: Could not return OpenGL context." << std::endl;
    return {};
  }

  // Construct the vector of pixel value pairs, rounding each pixel location to the nearest pixel
  // location and clamping to ensure we don't read out of bounds.
  std::vector< std::array<uint16_t, imageData.size()> > rawPixels;
  for (const auto& pair : correspondences) {
    std::array<uint16_t, imageData.size()> pixelValues;
    for (int i = 0; i < imageData.size(); i++) {
      // Compute the pixel location, rounding to nearest integer and clamping to image bounds.
      int x = static_cast<int>(std::round(pair[i][0]));
      int y = static_cast<int>(std::round(pair[i][1]));
      if (x < 0) { x = 0; }
      if (x >= widths[i]) { x = widths[i] - 1; }
      if (y < 0) { y = 0; }
      if (y >= heights[i]) { y = heights[i] - 1; }
      // Read the pixel value.
      pixelValues[i] = imagePixels[i][y * widths[i] + x];
    }
    rawPixels.emplace_back(pixelValues);
  }

  return rawPixels;
}

/// @brief Compute the color offset adjustment needed for the second camera in a pair based on the first.
/// @param correspondences The point correspondences between the two cameras.
/// @param imageData1 The image data for the first camera.
/// @param imageData2 The image data for the second camera.
/// @param cri1 The camera render info for the first camera.
/// @param cri2 The camera render info for the second camera.
/// @return The new color offset for the second camera that will make the adjusted average pixel values
/// for both cameras match.
static float ComputeNewColorOffset(
  const std::vector<PointCorrespondences::PointPair>& correspondences,
  std::shared_ptr<ImageData> imageData1, std::shared_ptr<ImageData> imageData2,
  std::shared_ptr<asdp::render::CameraRenderInfo> cri1, std::shared_ptr<asdp::render::CameraRenderInfo> cri2)
{
  // Read the vector of raw values from both images, which we'll adjust and then use to compute
  // the new offset.
  std::array<uint16_t, 2> widths= { cri1->m_resolutionPixels[0], cri2->m_resolutionPixels[0] };
  std::array<uint16_t, 2> heights = { cri1->m_resolutionPixels[1], cri2->m_resolutionPixels[1] };
  std::array<std::shared_ptr<ImageData>, 2> imageDatas = {imageData1, imageData2 };
  std::vector< std::array<uint16_t, 2> > rawPixels = GetRawPixelValues(correspondences,
    widths, heights, imageDatas);

  // Find the average pixel value for each camera after adjusting for gain and offset.
  float offset1, gain1, offset2, gain2;
  cri1->GetColorOffsetGain(offset1, gain1);
  cri2->GetColorOffsetGain(offset2, gain2);
  double sum1 = 0.0, sum2 = 0.0;
  for (const auto& pixelPair : rawPixels) {
    sum1 += (pixelPair[0] + offset1) * gain1;
    sum2 += (pixelPair[1] + offset2) * gain2;
  }
  float avg1 = sum1 / rawPixels.size();
  float avg2 = sum2 / rawPixels.size();

  // Compute the new offset for the second camera to make its average match the first camera's average.
  // We want to know how much and in which direction to shift the second camera's pixel values.  The
  // second camera's offset is what is added to the raw pixel values before gain is applied, so we need to
  // subtract the needed delta divided by the gain from the current offset.
  float delta = avg2 - avg1;
  float newOffset2 = offset2 - (delta / gain2);

  return newOffset2;
}

static void AutoUpdateColorOffsets(void* /* unused */)
{
  if (!g_pointCorrespondences) {
    std::cerr << "AutoUpdateColorOffsets(): Error: No point correspondences object available." << std::endl;
    return;
  }

  // Get a consistent set of images to use for the adjustment.
  std::vector< std::shared_ptr<ImageData> > imageSet = GetConsistentImageSet();
  if (imageSet.size() != g_visibleCameras.size()) {
    UnlockConsistentImageSet(imageSet);
    std::cerr << "AutoUpdateColorOffsets(): Error: Could not get consistent image set." << std::endl;
    return;
  }

  // Update the color offsets on the second of each camera pair based on the first.  Pass the appropriate
  // image pair along with the image infos.
  // image pair along with the image infos.
  for (const auto& cameraPair : g_cameraPairs) {
    std::vector<PointCorrespondences::PointPair> correspondences =
      g_pointCorrespondences->CorrespondencesForCameraPair(cameraPair);
    if (correspondences.empty()) {
      std::cerr << "Warning: No correspondences found for camera pair: ("
        << cameraPair[0] << ", " << cameraPair[1] << "), skipping color adjustment" << std::endl;
      continue;
    }

    // Find the indices of the entries in g_visibleCameras whose camID fields match the two cameras in the pair.
    size_t index1 = SIZE_MAX, index2 = SIZE_MAX;
    for (size_t i = 0; i < g_visibleCameras.size(); i++) {
      if (g_visibleCameras[i]->m_ID == cameraPair[0]) {
        index1 = i;
      }
      if (g_visibleCameras[i]->m_ID == cameraPair[1]) {
        index2 = i;
      }
    }
    if (index1 == SIZE_MAX || index2 == SIZE_MAX) {
      std::cerr << "Warning: One or both cameras not found for camera pair: ("
        << cameraPair[0] << ", " << cameraPair[1] << "), skipping color adjustment" << std::endl;
      continue;
    }

    // Compute the offset adjustment needed.
    float newOffset = ComputeNewColorOffset(correspondences,
      imageSet[index1], imageSet[index2],
      g_visibleCameras[index1], g_visibleCameras[index2]);

    // Apply the offset adjustment to the second camera in the pair.
    std::shared_ptr<asdp::render::CameraRenderInfo> cri = g_visibleCameras[index2];
    float currentOffset, currentGain;
    cri->GetColorOffsetGain(currentOffset, currentGain);
    cri->SetColorOffsetGain(newOffset, currentGain);
  }

  // Done with the images, unlock them.
  UnlockConsistentImageSet(imageSet);
}

/// @brief Compute the color offset adjustment needed for the second camera in a pair based on the first.
/// @param correspondences The point correspondences between the two cameras.
/// @param imageData1 The image data for the first camera.
/// @param imageData2 The image data for the second camera.
/// @param cri1 The camera render info for the first camera.
/// @param cri2 The camera render info for the second camera.
/// @return The new color offset for the second camera that will make the adjusted average pixel values
/// for both cameras match.
static std::array<float, 2> ComputeNewColorOffsetGain(
  const std::vector<PointCorrespondences::PointPair>& correspondences,
  std::shared_ptr<ImageData> imageData1, std::shared_ptr<ImageData> imageData2,
  std::shared_ptr<asdp::render::CameraRenderInfo> cri1, std::shared_ptr<asdp::render::CameraRenderInfo> cri2)
{
  // Read the vector of raw values from both images, which we'll adjust and then use to compute
  // the new offset.
  std::array<uint16_t, 2> widths = { cri1->m_resolutionPixels[0], cri2->m_resolutionPixels[0] };
  std::array<uint16_t, 2> heights = { cri1->m_resolutionPixels[1], cri2->m_resolutionPixels[1] };
  std::array<std::shared_ptr<ImageData>, 2> imageDatas = { imageData1, imageData2 };
  std::vector< std::array<uint16_t, 2> > rawPixels = GetRawPixelValues(correspondences,
    widths, heights, imageDatas);

  // Transform the points to consistent space based on the current offset and gain and place them
  // into a vector of 2D points where the first element (x) is from the first camera and the second
  // element (y) is from the second camera.
  float offset1, gain1, offset2, gain2;
  cri1->GetColorOffsetGain(offset1, gain1);
  cri2->GetColorOffsetGain(offset2, gain2);
  std::vector<PointCorrespondences::Point2D> adjustedPoints;
  for (const auto& pixelPair : rawPixels) {
    PointCorrespondences::Point2D adjustedPoint;
    adjustedPoint[0] = (pixelPair[0] + offset1) * gain1;
    adjustedPoint[1] = (pixelPair[1] + offset2) * gain2;
    adjustedPoints.push_back(adjustedPoint);
  }

  // Compute the best-fit line through the points.
  float sumX = 0.0f, sumY = 0.0f, sumXY = 0.0f, sumXX = 0.0f;
  for (const auto& pt : adjustedPoints) {
    sumX += pt[0];
    sumY += pt[1];
    sumXY += pt[0] * pt[1];
    sumXX += pt[0] * pt[0];
  }
  float n = static_cast<float>(adjustedPoints.size());
  float slope = (n * sumXY - sumX * sumY) / (n * sumXX - sumX * sumX);
  float intercept = (sumY - slope * sumX) / n;

  // Compute the factor to adjust the slope by to make it 1.0 (i.e., y = x).
  // This is the amount by which we'll multiply the gain of the second camera to bring it into alignment.
  // If the slope is very different, just leave the gain unchanged because this is
  // probably due to numerical instability.
  /// @todo Consider a more robust way to determine numerical instability.
  float gainAdjustment = 1.0f / slope;
  if (slope < 0.5 || slope > 2) { gainAdjustment = 1.0f; }
  float newGain2 = gain2 * gainAdjustment;

  // Compute the new point values with the adjusted gain for the second camera to find the new offset needed.
  for (size_t i = 0; i < adjustedPoints.size(); i++) {
    adjustedPoints[i][1] = (rawPixels[i][1] + offset2) * newGain2;
  }

  // Find the average pixel value for each camera in the adjusted point set.
  float sum1 = 0.0, sum2 = 0.0;
  for (const auto& pt : adjustedPoints) {
    sum1 += pt[0];
    sum2 += pt[1];
  }
  float avg1 = sum1 / adjustedPoints.size();
  float avg2 = sum2 / adjustedPoints.size();

  // Compute the new offset for the second camera to make its average match the first camera's average.
  // We want to know how much and in which direction to shift the second camera's pixel values.  The
  // second camera's offset is what is added to the raw pixel values before gain is applied, so we need to
  // subtract the needed delta divided by the gain from the current offset.
  float delta = avg2 - avg1;
  float newOffset2 = offset2 - (delta / newGain2);

  return { newOffset2, newGain2 };
}

static void AutoUpdateColorOffsetsAndGains(void* /* unused */)
{
  if (!g_pointCorrespondences) {
    std::cerr << "AutoUpdateColorOffsetsAndGains(): Error: No point correspondences object available." << std::endl;
    return;
  }

  // Get a consistent set of images to use for the adjustment.
  std::vector< std::shared_ptr<ImageData> > imageSet = GetConsistentImageSet();
  if (imageSet.size() != g_visibleCameras.size()) {
    UnlockConsistentImageSet(imageSet);
    std::cerr << "AutoUpdateColorOffsetsAndGains(): Error: Could not get consistent image set." << std::endl;
    return;
  }

  // Update the color offsets on the second of each camera pair based on the first.  Pass the appropriate
  // image pair along with the image infos.
  // image pair along with the image infos.
  for (const auto& cameraPair : g_cameraPairs) {
    std::vector<PointCorrespondences::PointPair> correspondences =
      g_pointCorrespondences->CorrespondencesForCameraPair(cameraPair);
    if (correspondences.empty()) {
      std::cerr << "Warning: No correspondences found for camera pair: ("
        << cameraPair[0] << ", " << cameraPair[1] << "), skipping color adjustment" << std::endl;
      continue;
    }

    // Find the indices of the entries in g_visibleCameras whose camIS fields match the two cameras in the pair.
    size_t index1 = SIZE_MAX, index2 = SIZE_MAX;
    for (size_t i = 0; i < g_visibleCameras.size(); i++) {
      if (g_visibleCameras[i]->m_ID == cameraPair[0]) {
        index1 = i;
      }
      if (g_visibleCameras[i]->m_ID == cameraPair[1]) {
        index2 = i;
      }
    }
    if (index1 == SIZE_MAX || index2 == SIZE_MAX) {
      std::cerr << "Warning: One or both cameras not found for camera pair: ("
        << cameraPair[0] << ", " << cameraPair[1] << "), skipping color adjustment" << std::endl;
      continue;
    }

    // Compute the offset adjustment needed.
    std::array<float, 2> newOffsetGain = ComputeNewColorOffsetGain(correspondences,
      imageSet[index1], imageSet[index2],
      g_visibleCameras[index1], g_visibleCameras[index2]);

    // Apply the offset adjustment to the second camera in the pair.
    std::shared_ptr<asdp::render::CameraRenderInfo> cri = g_visibleCameras[index2];
    cri->SetColorOffsetGain(newOffsetGain[0], newOffsetGain[1]);
  }

  // Done with the images, unlock them.
  UnlockConsistentImageSet(imageSet);
}

/// @brief Callback handler to save the camera configuration to a file.
static void SaveCameraConfig(const std::string& filename, void* userdata)
{
  if (g_visibleCameras.empty()) {
    std::cerr << "Error: No cameras to save configuration for." << std::endl;
    return;
  }
  if (!userdata) {
    std::cerr << "Error: No user data provided for callback handler." << std::endl;
    return;
  }
  CallbackHandlerData* data = static_cast<CallbackHandlerData*>(userdata);

  // Parse the original JSON configuration file for the camera configuration directly, then replace
  // the color offsets and gains for each camera with those currenttly stored because they may have been
  // adjusted by the user.
  json cameraConfig;
  try {
    std::ifstream configFile(data->cameraConfigFileName);
    cameraConfig = json::parse(configFile);
  } catch (const std::exception& e) {
    std::cerr << "Error: Unable to read camera configuration file: " << data->cameraConfigFileName
      << ": " << e.what() << std::endl;
    std::cerr << "  (Cannot save configuration file)" << std::endl;
    return;
  }
  // Iterate through the cameras and update their color offsets and gains.
  for (auto& camera : cameraConfig["cameras"]) {
    uint16_t id = camera["id"];
    for (auto& cri : g_visibleCameras) {
      if (cri->m_ID == id) {
        float offset, gain;
        cri->GetColorOffsetGain(offset, gain);
        camera["color"]["offset"] = offset;
        camera["color"]["gain"] = gain;

        break;  // Found the camera, no need to continue.
      }
    }
  }

  // Write the updated configuration to the specified file
  try {
    std::ofstream outFile(filename);
    outFile << cameraConfig.dump(2);  // Pretty print with 2 spaces.
    outFile.close();
    std::cout << "Saved camera configuration to: " << filename << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "Error: Unable to write camera configuration file: " << filename
      << ": " << e.what() << std::endl;
  }
}


/// @brief Thread to compute the depth information for the cameras and update the meshes.
/// @details The CopyDepthInfo callback handler transfers the vertex buffers from the
/// @param timer Timer to use for getting the current time for depth estimation.
/// @param depthContext DisplayTexture to use for borrowing an OpenGL context to update the meshes.
/// CameraRenderInfo inline during rendering.
static void DepthThreadFunction(std::shared_ptr<Timer> timer, std::shared_ptr<DisplayTexture> depthContext)
{
  if (!depthContext->BorrowContext()) {
    std::cerr << "DepthThreadFunction(): Error: Could not borrow OpenGL context." << std::endl;
    return;
  }
  while (g_runDepthThread) {
    g_timingInfo.depthComputeStartTimes.push_back(std::chrono::steady_clock::now());
    // Make a snapshot of the images from all cameras at the same time and store it into
    // a custom ImageQueue that has a single entry from the same time for all of them.
    /// @todo

    Time now;
    Status status = timer->GetCoreTime(now);
    if (status != OKAY) {
      std::cerr << "Failed to get time: " << ErrorMessage(status) << std::endl;
      return;
    }
    /// @todo Consider another approach to finding the time for the estimate.
    try {
      std::string ret = g_depthEstimator->ComputeDepthEstimate(now);
      if (ret != "") {
        std::cerr << "Error computing depth estimate: " << ret << std::endl;
        return;
      } else {
        g_depthEstimator->UpdateMeshesGPU(g_visibleCameras);
      }
    } catch (const std::exception& e) {
      std::cerr << "Exception while computing depth estimate: " << e.what() << std::endl;
      return;
    }
    g_timingInfo.depthComputeEndTimes.push_back(std::chrono::steady_clock::now());

    std::this_thread::sleep_for(std::chrono::milliseconds(1));  // Sleep a bit to avoid eating a whole CPU.
  }
  depthContext->ReturnContext();
}

/// @brief Callback handler to compute depth information for the cameras.
static void CopyDepthInfo(Time renderTime, void* /* unused */)
{
  g_timingInfo.depthStartTimes.push_back(std::chrono::steady_clock::now());

  // Update the vertex buffers for all of the visible cameras with the new depth information.
  // The depth updates are computed by DepthThreadFunction, which runs in a separate thread
  // and updates the vertex buffers in the CameraRenderInfo objects inline.
  for (std::shared_ptr<asdp::render::CameraRenderInfo> cri : g_visibleCameras) {
    for (auto& composite: g_composites) {
      composite->UpdateVertexBuffer(*cri);
    }
  }

  g_timingInfo.depthEndTimes.push_back(std::chrono::steady_clock::now());
}

/// @brief Callback handler to turn on and off depth rendering on the visible cameras.
static void SetDepthRendering(bool depthRendering, void* /* unused */)
{
  for (std::shared_ptr<asdp::render::CameraRenderInfo> cri : g_visibleCameras) {
    if (depthRendering) {
      // Set to clamp to white at a distance of 200 meters, to give us some resolution below that.
      cri->m_depthScale = 1.0f / 200;
    } else {
      cri->m_depthScale = -1.0f;
    }
  }
  std::cout << "Toggled depth rendering to: " << (depthRendering ? "on" : "off") << std::endl;
}

static std::shared_ptr<Message> WaitForMessageType(std::shared_ptr<Receiver> receiver, MessageID type, float seconds)
{
  std::shared_ptr<Message> empty;   ///< We return this on failure.
  std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
  do {
    std::shared_ptr<StreamPacket> response;
    size_t offset = 0;
    Status status = receiver->ReceiveStreamPacket(0, response, offset);
    if ((status != OKAY) && (status != TIMEOUT)) {
      return empty;
    }
    if (response != nullptr) {
      std::shared_ptr<Message> message;
      status = response->GetNextMessage(message);
      if (status != OKAY) {
        return empty;
      }
      while (message != nullptr) {
        MessageID messageType;
        status = message->GetType(messageType);
        if (status != OKAY) {
          return empty;
        }
        if (messageType == type) {
          // Worked!
          return message;
        }
        status = response->GetNextMessage(message);
        if (status != OKAY) {
          return empty;
        }
      }
    }
  } while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() <= seconds);

  return empty;
}

/// @param [out] replayDone Set to true if we are at the end of replay, set to false otherwise.
static Status HandleStreamPacket(std::shared_ptr<StreamPacket> packet, std::shared_ptr<ClockSynchronizer> clockSync,
  std::shared_ptr<PoseAdjuster> poseAdjuster, bool &replayDone, std::vector<std::shared_ptr<Display>> &displays,
  std::shared_ptr<Timer> timer, Time &pausedTime)
{
  // Not done replaying unless we get a message telling us that we are.
  replayDone = false;

  // Parse all of the messages in the stream packet, handling each of them in turn.
  std::shared_ptr<Message> message;
  Status status = packet->GetNextMessage(message);
  if (OKAY != status) {
    return status;
  }
  while (message != nullptr) {
    MessageID messageType;
    status = message->GetType(messageType);
    if (OKAY != status) {
      return status;
    }

    switch (messageType) {
    case EVENT:
      {
        // Find the event type and handle it.
        MessageEvent event(*message);
        if (event.GetConstructorStatus() != OKAY) {
          return event.GetConstructorStatus();
        }
        EventID eventType;
        status = event.GetType(eventType);
        if (status != OKAY) {
          return status;
        }
        // Get the string message.
        std::string messageString;
        status = event.GetParam(messageString);
        if (status != OKAY) {
          return status;
        }
        switch (eventType) {
          case START_OF_REPLAY:
            {
              // Reset the clock-sync estimates when we start, stop, or resume replay.
              clockSync->ClearHistory();
            }
            break;
          case END_OF_REPLAY:
            {
              // Reset the clock-sync estimates when we start, stop, or resume replay.
              clockSync->ClearHistory();
            }
            break;
          case REPLAY_PAUSED:
            {
              // Store the time that we're paused at so that we can reset our clock-sync estimates
              // when we resume.
              status = message->GetTime(pausedTime);
              if (status != OKAY) {
                return status;
              }

              // Tell all of our Displays that we're paused.
              for (auto &display : displays) {
                display->SetNowPlaying(false);
              }
            }
            break;
          case REPLAY_RESUMED:
            {
              // Reset the clock-sync estimates when we start, stop, or resume replay.
              clockSync->ClearHistory();

              // Add an entry to the clock-sync estimates based on the time we were paused,
              // making the current time match it.  Then reset the history again so that this
              // phantom entry doesn't affect the estimates.  We needed to reset the history
              // before doing this so that our single entry causes the shift that we want.
              if (!clockSync->AddDataPoint(pausedTime, std::chrono::steady_clock::now())) {
                return UNEXPECTED_INTERNAL_STATE;
              }
              clockSync->ClearHistory();

              // Tell all of our Displays that we're no longer paused.
              for (auto& display : displays) {
                display->SetNowPlaying(true);
              }
            }
            break;

          case CLOCK_SYNC:
            {
              // Adjust the timer offset based on clock-sync messages.  The first message (or the first one
              // after replay resumes, or the first one after replay stops), sets the estimated offset based
              // on that single number and the relative rate to 1.0. Later ones adjust based on an average of
              // the previous ones as described in the render implementation document.
              Time messageTime;
              status = message->GetTime(messageTime);
              if (status != OKAY) {
                return status;
              }
              clockSync->AddDataPoint(messageTime, std::chrono::steady_clock::now());
              g_lastCLOCK_SYNC = messageTime;
            }
            break;

          case INVALID_OPERATION:
            {
              // If we get an invalid operation message, say so
              std::cerr << "Invalid operation message received from server: " << messageString << std::endl;
            }
            break;

          case INTERNAL_ERROR:
          {
            // If we get an internal error message, say so
            std::cerr << "Internal error message received from server: " << messageString << std::endl;
          }
          break;

          case UNRECOGNIZED_OPCODE:
          {
            // If we get an unrecognized opcode message, say so
            std::cerr << "Unrecognized opcode message received from server: " << messageString << std::endl;
          }
          break;

          default:
            break;
        }
      }
      break;
    case STATE:
      {
        // Parse the state message and keep track of anything we need to.
        MessageState state(*message);
        if (state.GetConstructorStatus() != OKAY) {
          return state.GetConstructorStatus();
        }
        uint8_t replaying;
        status = state.GetReplaying(replaying);
        if (status != OKAY) {
          return status;
        }
        //std::cout << "XXX Replaying = " << (replaying ? "true" : "false") << std::endl;

        // If we're replaying and we're at the end of replay, indicate this.
        if (replaying) {
          uint8_t endOfReplay;
          status = state.GetReplayAtEnd(endOfReplay);
          if (status != OKAY) {
            return status;
          }
          if (endOfReplay) {
            replayDone = true;
          }
        }
      }
      break;
    case POSE:
      {
        // Parse the pose message and add the pose to the adjuster.
        MessagePose pose(*message);
        if (pose.GetConstructorStatus() != OKAY) {
          return pose.GetConstructorStatus();
        }
        poseAdjuster->AddPose(pose);
      }
      break;
    default:
      // Ignore other message types.
      break;
    }

    status = packet->GetNextMessage(message);
    if (OKAY != status) {
      return status;
    }
  }

  return OKAY;
}

/// @brief Structure to hold display information
struct DisplayInfo
{
  ToneMap toneMap = ToneMap();  ///< The tone map to use.
  bool useOpenXR = false;       ///< Use OpenXR for rendering? If so, overrides all of the following.
  std::string XSightNIC = "";   ///< NIC to listen to XSight on for rendering. If not empty, overrides all of the following.
  int XSightDisplay = 1;        ///< The display to use for XSight rendering.
  std::string XSight2NIC = "";  ///< NIC to listen to XSight2 on for rendering. If not empty, overrides all of the following.
  int XSight2Display = 1;       ///< The display to use for XSight2 rendering.
  int width = 1280;             ///< The width of the display.
  int height = 1024;            ///< The height of the display.
  float hFOV = 40.0f;           ///< The horizontal field of view in degrees.
  std::string joystick = "";    ///< The joystick to use for input.
  float fps = 60.0f;            ///< The frames per second to run at.
  bool fullScreen = false;      ///< Run in full screen mode.
  int fullScreenDisplay = 0;    ///< The display to run in full screen mode on.
  std::array<float, 3> viewpointOffset = { 0.0f, 0.0f, 0.0f };  ///< The offset to apply to the viewpoint for this display, in meters.

  //======================================
  // Added by Sang Yoon to add a flag for enabling the cylindrical projection.
  bool enableCP = false;        ///< The flag to enable the cylindrical projection
  //======================================

  //======================================
  // Added by Sang Yoon to indicate if the window associated with the display is overview window, detailed view windowe, or neither.
  // Where the number of displays is greater than 1, the window that has the widest horizontal FOV is considered as an overview window,
  // and the window that has the narrowest hFOV is considered as a detailed view window.
  bool overview = false;
  bool detailed_view = false;
  //======================================
};

static std::string TimeIntervalToStringMilliseconds(std::chrono::duration<float> interval)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(9) << interval.count() * 1000;
  return oss.str();
}

static std::chrono::steady_clock::time_point LargestTimeLessThan(std::chrono::steady_clock::time_point time,
  const std::vector<std::chrono::steady_clock::time_point>& times)
{
  std::chrono::steady_clock::time_point largest = std::chrono::steady_clock::time_point::min();
  for (auto &t : times) {
    if (t < time) {
      largest = std::max(largest, t);
    }
  }
  return largest;
}

/// @brief Read a table of floats from a binary file.
/// @param filename The name of the file to read from.
/// @param numFloats The number of floats to read from the file.
/// @return A vector of floats read from the file, or an empty vector if there was an error.
std::vector<float> ReadFloatTableFromFile(const std::string& filename, size_t numFloats)
{
  std::vector<float> ret;
  std::ifstream file(filename, std::ios::binary);
  if (!file) {
    std::cerr << "Unable to open file: "<< filename << std::endl;
    return ret;
  }
  // Find the file size and compare it to the expected number of floats.
  file.seekg(0, std::ios::end);
  std::streamsize fileSize = file.tellg();
  if (fileSize != static_cast<std::streamsize>(numFloats * sizeof(float))) {
    std::cerr << "File size does not match expected number of floats: " << filename << std::endl;
    return ret;
  }
  // Read the floats from the file.
  file.seekg(0, std::ios::beg);
  ret.resize(numFloats);
  file.read(reinterpret_cast<char*>(ret.data()), numFloats * sizeof(float));
  if (!file) {
    ret.resize(0);
    std::cerr << "Error reading floats from file: " << filename << std::endl;
    return ret;
  }
  return ret;
}

/// @brief Structure to hold the NUC tables for a camera.
struct NucTables {
  float coreTemperature;            ///< The core temperature for the camera.
  float sensorTemperature;          ///< The sensor temperature for the camera.
  int imageWidth;                   ///< The width of the image for the camera.
  int imageHeight;                  ///< The height of the image for the camera.
  std::vector<float> offsetTable;   ///< The offset table for the camera.
  std::vector<float> gainTable;     ///< The gain table for the camera.
};

/// @brief Structure to hold the NUC information for all cameras.
struct NUCInfo {
  std::string temperatureType;  ///< The type of temperature to use for the NUC tables (sensor or core).
  float coldBBRTemperature;     ///< The cold blackbody reference temperature for the NUC tables.
  float hotBBRTemperature;      ///< The hot blackbody reference temperature for the NUC tables.
  float minVisibleTemperature;  ///< The minimum visible temperature for the NUC tables.
  float maxVisibleTemperature;  ///< The maximum visible temperature for the NUC tables.

  /// @brief A map from camera ID to its NUC tables.
  std::map<uint32_t, NucTables> CameraNUCTables;
};

// Read the NUC information given a directory to read it from.  This includes the information in the
// NUC.json file and the temperature tables (offset and gain) for each camera in the NUC.json file.
// Make a map from camera ID to the tables for each camera.

std::shared_ptr<NUCInfo> ReadNUCInfoFromDirectory(const std::string& nucInfoDirectory)
{
  std::shared_ptr<NUCInfo> nucInfo = std::make_shared<NUCInfo>();
  try {
    nlohmann::json nucJson;
    std::string nucJsonFileName = nucInfoDirectory + "/NUC.json";
    std::cout << "Reading NUC information from: " << nucJsonFileName << std::endl;
    std::ifstream nucJsonFile(nucJsonFileName);
    if (!nucJsonFile.is_open()) {
      std::cerr << "Failed to open NUC JSON file: " << nucJsonFileName << std::endl;
      return nullptr;
    }
    nucJsonFile >> nucJson;
    nucJsonFile.close();
    nucInfo->coldBBRTemperature = nucJson["coldBBRTemperature"];
    nucInfo->hotBBRTemperature = nucJson["hotBBRTemperature"];
    nucInfo->minVisibleTemperature = nucJson["minVisibleTemperature"];
    nucInfo->maxVisibleTemperature = nucJson["maxVisibleTemperature"];
    for (const auto& cameraEntry : nucJson["cameras"]) {
      uint32_t cameraID = cameraEntry["id"];
      std::string offsetTableFileName = nucInfoDirectory + "/" + cameraEntry["offsetFile"].get<std::string>();
      std::string gainTableFileName = nucInfoDirectory + "/" + cameraEntry["gainFile"].get<std::string>();
      std::cout << "  Reading NUC tables for camera ID " << cameraID << " from files: "
        << offsetTableFileName << " and " << gainTableFileName << std::endl;
      NucTables tables;
      int width = cameraEntry["imageWidth"];
      int height = cameraEntry["imageHeight"];
      tables.imageWidth = width;
      tables.imageHeight = height;
      tables.coreTemperature = cameraEntry["coreTemperature"];
      tables.sensorTemperature = cameraEntry["sensorTemperature"];
      tables.offsetTable = ReadFloatTableFromFile(offsetTableFileName, width * height);
      if (tables.offsetTable.size() == 0) {
        std::cerr << "Failed to read offset table from file: " << offsetTableFileName << std::endl;
        return nullptr;
      }
      tables.gainTable = ReadFloatTableFromFile(gainTableFileName, width * height);
      if (tables.gainTable.size() == 0) {
        std::cerr << "Failed to read gain table from file: " << gainTableFileName << std::endl;
        return nullptr;
      }
      nucInfo->CameraNUCTables[cameraID] = tables;
    }
  } catch (const std::exception& e) {
    std::cerr << "Error reading NUC information from directory: " << nucInfoDirectory
      << ": " << e.what() << std::endl;
    return nullptr;
  }

  return nucInfo;
}

/// @brief Callback handler to process annotations requests from the CompositeCameras.
std::vector<CompositeCameras::Annotation> AnnotationCallbackHandler(Time time, void * /* userData */)
{
  // Atomically make a copy of the current analysis objects to use for generating annotations.
  std::shared_ptr< std::map<std::string, AnalysisObjectOverTime> > analysis = std::atomic_load(&g_currentAnalysis);

  // Fill in the annotations vector to return. For each named object, select the entry whose time is in the
  // most-recent past (ignoring future reports). Adjust their poses based on any velocity since analysis time.
  // Adjust their opacity based on the fading factor times the time since analysis time.  Remove any whose
  // opacity is zero or less or whose Chance value is below threshold.
  std::vector<CompositeCameras::Annotation> annotations;
  if (analysis) {

    // Fill in an entry for each report.
    for (const auto& [name, series] : *analysis) {
      // Find the report (if any) in this series whose time is the most recent past time compared to the render time.
      AnalysisReport const* report = nullptr;
      for (auto& entry : series) {
        if (entry.Timestamp <= time) {
          report = &entry;
        } else {
          break;  // The series is in chronological order, so we can stop looking once we hit a future entry.
        }
      }
      if (report == nullptr) {
        // No past report for this object, skip it.
        continue;
      }

      CompositeCameras::Annotation annotation;

      // Determine the opacity based on time since analysis and fading factor.
      float dt = 0;
      if (report->Timestamp < time) {
        Time delta = time - report->Timestamp;
        dt = delta.seconds + delta.microseconds * 1e-6;
      }
      float opacity = 1.0f - dt / g_analysisFadeTimeSeconds;
      if (opacity <= 0.0f) {
        // Fully faded out, skip it.
        continue;
      }

      // Convert to an annotation and verify that the label is not empty.
      annotation = report->ConvertToAnnotation(g_analysisChanceThreshold, opacity);
      if (annotation.label.empty()) {
        // No label, skip it.
        continue;
      }

      // If there are both position and velocity, update the position based on time since analysis.
      if (report->Loc && report->Vel) {
        annotation.uv[0] += (*report->Vel)[0] * dt;
        annotation.uv[1] += (*report->Vel)[1] * dt;
      }

      // Add the annotation to the list.
      annotations.push_back(annotation);
    }
  }

  // Make a vector that adds the current annotations and the camera annotations.
  // Avoid concurrent access to the annotation vectors.
  std::lock_guard<std::mutex> lock(g_annotationMutex);
  annotations.insert(annotations.end(), g_cameraAnnotations.begin(), g_cameraAnnotations.end());
  return annotations;
}


/// @brief Callback handler to reset the analysis.
static void ResetAnalysis(void* userdata)
{
  if (!userdata) {
    std::cerr << "Error: No user data provided for ResetAnalysis() callback handler." << std::endl;
    return;
  }
  CallbackHandlerData* data = static_cast<CallbackHandlerData*>(userdata);
  ++data->analysisEpoch;
}

/// @brief Thread to handle analysis reports and keep the vector of current reports updated.
void HandleAnalysisThread(std::vector<std::string> analysisModuleURLs, std::shared_ptr<Timer> timer)
{
  // Make a receiver for each analysis module URL.  They start out disconnected.
  std::vector< std::shared_ptr<JSONStringReceiver> > analysisReceivers(analysisModuleURLs.size());

  // Vector of times at which we last tried to connect to each receiver, so that we can avoid trying to connect too frequently.
  std::vector< std::chrono::steady_clock::time_point > lastConnectAttemptTimes(analysisReceivers.size(),
    std::chrono::steady_clock::now() - std::chrono::seconds(1));

  // Vector of maps of analysis objects over time, one per receiver.
  std::vector< std::map<std::string, AnalysisObjectOverTime> > reportMaps(analysisReceivers.size());

  int currentEpoch = g_callbackHandlerData.analysisEpoch;  // Local copy of the current epoch to detect changes.

  std::string jsonString;
  while (g_runAnalysisThread) {

    // If the epoch has changed, clear all of the receivers and report maps to reset the analysis.
    if (g_callbackHandlerData.analysisEpoch != currentEpoch) {
      std::cout << "Resetting analysis connections." << std::endl;
      currentEpoch = g_callbackHandlerData.analysisEpoch;
      for (auto& receiver : analysisReceivers) {
        receiver.reset();
      }
      for (auto& rm : reportMaps) {
        rm.clear();
      }
    }

    std::vector<AnalysisReport> reports;
    for (size_t i = 0; i < analysisReceivers.size(); i++) {
      auto& receiver = analysisReceivers[i];
      auto& rm = reportMaps[i];

      if (!receiver) {
        if (std::chrono::steady_clock::now() - lastConnectAttemptTimes[i] > std::chrono::seconds(1)) {
          // It's been a while since we last tried to connect, try again.
          const std::string& url = analysisModuleURLs[i];
          std::shared_ptr<JSONStringReceiver> analysisModule;
          Status status = JSONStringReceiver::Create(url, analysisModule);
          if (status != OKAY) {
            lastConnectAttemptTimes[i] = std::chrono::steady_clock::now();
            std::cout << "Failed to create analysis receiver for URL: " << url << std::endl;
            continue;
          }
          analysisReceivers[i] = analysisModule;
          std::cout << "Added analysis receiver for URL: " << url << std::endl;
        }
        continue;
      }

      // Read up to 10 pending reports from this receiver to avoid starving other receivers
      // while efficiently processing a backlog of reports from a single receiver.
      size_t count = 0;
      Status status = OKAY;
      do {
        status = receiver->Receive(0.0f, jsonString);
        if (status == TIMEOUT) {
          // No more reports to read right now, stop looking for new ones.
          break;
        }
        if (status != OKAY) {
          // An error occurred, drop this receiver and try to reconnect later.
          std::cout << "Error receiving from analysis receiver, dropping receiver: " << ErrorMessage(status) << std::endl;
          receiver.reset();
          rm.clear();
          break;
        }
        // Parse the JSON string into analysis reports.
        try {
          // Convert this to a report
          AnalysisReport report(jsonString);

          // Remove ASDP_ from the beginning of the name if it's there to make it cleaner for display.
          if (report.Name.rfind("ASDP_", 0) == 0) {
            report.Name = report.Name.substr(5);
          }

          // Remove ASDP_ from the beginning of each type if it's there to make it cleaner for display.
          for (auto& c : report.Class) {
            if (c.Type) {
              if (c.Type->rfind("ASDP_", 0) == 0) {
                *c.Type = c.Type->substr(5);
              }
            }
          }

          // Add this report to the appropriate map at the back (latest in time) location.
          rm[report.Name].push_back(report);

        } catch (const std::exception& e) {
          std::cerr << "Error parsing analysis report JSON: " << e.what() << std::endl;
        }
      } while (++count < 10);

      // Remove any reports that are too old and then remove any map entries that are empty.
      Time now = g_lastCLOCK_SYNC;
      for (auto it = rm.begin(); it != rm.end();) {
        auto& series = it->second;
        // Remove old reports from the back of the series.
        while (!series.empty() && series.back().Timestamp + Time(g_analysisFadeTimeSeconds, 0) < now) {
          series.pop_back();
        }
        if (series.empty()) {
          // No more reports for this object, remove the entry from the map.
          it = rm.erase(it);
        } else {
          ++it;
        }
      }
    }

    // Update the current reports atomically.
    // Make a shared pointer to a new vector that combines all of the report vectors into one.
    std::shared_ptr< std::map<std::string, AnalysisObjectOverTime> > combinedReports =
      std::make_shared< std::map<std::string, AnalysisObjectOverTime> >();
    for (const auto& rm : reportMaps) {
      for (const auto& kv : rm) {
        // Insert them in time-sequential order. The existing list for each entry will be in increasing-time order.
        auto lastInsert = (*combinedReports)[kv.first].begin();
        for (const auto& report : kv.second) {
          // Find the first entry in the existing list whose time is greater than this report's time.
          while (lastInsert != (*combinedReports)[kv.first].end() && lastInsert->Timestamp <= report.Timestamp) {
            ++lastInsert;
          }
          // Insert this report before that entry to maintain increasing-time order.
          (*combinedReports)[kv.first].insert(lastInsert, report);
        }
      }
    }
    std::atomic_store(&g_currentAnalysis, combinedReports);

    // Sleep a bit to avoid eating the entire CPU.
    std::this_thread::sleep_for(std::chrono::microseconds(1));
  }
}

//=================================================================

/// @brief Static function to spin up all of the things related to connecting to a camera ball.

int spin_up(std::shared_ptr<CoreClient> client, int &serialNumber, std::shared_ptr<Receiver> &receiver,
  std::vector<CameraInfo> &cameras, bool &hasStorage, bool &hasTemperatures, bool &hasPoses,
  uint8_t &triggerID, uint32_t &replayStreamID,
  std::filesystem::path &configPath, std::vector< std::shared_ptr<ReceiverUDP> > &UDPReceivers,
  std::set<int> const &skipCameras, std::vector<NUCInfo> &nucInfos, int &maxCameras,
  int &lineBatchesPerGPUSend, std::shared_ptr<DisplayTexture> &displayTexture,
  std::vector<DisplayInfo> &displayInfos, double &renderAheadFrames, double &cameraFPS,
  bool &computeDepth, double &autoRangeStdBelow, double &autoRangeStdAbove, float &maxDepth,
  float &depthThreshold, std::shared_ptr<PoseAdjuster> &poseAdjuster,
  std::vector< std::shared_ptr<asdp::render::CameraRenderInfo> > &cameraRenderInfos,
  std::vector<uint32_t> &cameraIDs,
  std::vector<GLuint> &toneMapTextures, double &staticDepth, std::atomic<bool> &done,
  std::string &ip_address, bool &doStreamPoses, uint32_t &frameStride,
  std::vector<std::string> &analysisModuleURLs,
  std::shared_ptr<ClockSynchronizer> &clockSync, std::shared_ptr<Timer> &timer,
  std::shared_ptr<DisplayTexture> &depthContext, std::vector<std::thread> &copyDataToGPUThreads,
  std::thread &analysisThread,
  std::vector< std::shared_ptr< SpinFreeQueue< std::shared_ptr<DataToSendToGPU> > > > &dataQueues,
  std::vector<std::thread> &receiveDataThreads,
  bool lockRotation, bool disableLatencyCompensation
)
{
  // We're not done yet
  done = false;

  //=================================================================
  // Create a PoseAdjuster to handle helicopter motion.
  PoseAdjusterCoordinates poseAdjusterCoordinates = HELICOPTER;
  if (lockRotation) {
    poseAdjusterCoordinates = INITIAL_ORIENTATION;
  }
  poseAdjuster = std::make_shared<PoseAdjuster>(2000, poseAdjusterCoordinates,
    disableLatencyCompensation);

  std::map<uint32_t, std::string> servers;
  Status status = client->IdentifiedServers(servers);
  if (status != OKAY) {
    std::cerr << "Error: Unable to get identified servers: " << ErrorMessage(status) << std::endl;
    return 7;
  }

  if (servers.empty()) {
    std::cerr << "No servers found; be sure to run the server first." << std::endl;
    return 8;
  }
  std::cout << "Servers found: " << servers.size() << std::endl;
  for (const auto& server : servers) {
    std::cout << "  " << server.second << " (serial #" << server.first << ")" << std::endl;
  }

  // Connect to the first matching server found.
  auto serverIt = servers.begin();
  if (serialNumber >= 0) {
    serverIt = servers.find(serialNumber);
    if (serverIt == servers.end()) {
      std::cerr << "Server with serial number " << serialNumber << " not found." << std::endl;
      return 8;
    }
  }
  std::cout << "Connecting to " << serverIt->second << std::endl;
  uint16_t major, minor, patch;
  status = client->ConnectToServer(serverIt->second, major, minor, patch);
  if (status != OKAY) {
    std::cerr << "Failed to connect to server: " << ErrorMessage(status) << std::endl;
    return 9;
  }
  serialNumber = serverIt->first;
  std::cout << "  Connected to server version " << major << "." << minor << "." << patch
    << " with serial number " << serialNumber << std::endl;

  // Get the main stream receiver
  status = client->GetMainStreamReceiver(receiver);
  if (status != OKAY) {
    std::cerr << "Failed to get main stream receiver: " << ErrorMessage(status) << std::endl;
    return 10;
  }

  // Ensure that we get a state message from the server within a reasonable time.
  // Report information about the cameras that were found.
  std::shared_ptr<Message> msg = WaitForMessageType(receiver, STATE, 5.0);
  if (msg == nullptr) {
    std::cerr << "Did not get state message." << std::endl;
    return 11;
  }
  MessageState state(*msg);
  if (state.GetConstructorStatus() != OKAY) {
    std::cerr << "Failed to construct state message: " << ErrorMessage(state.GetConstructorStatus()) << std::endl;
    return 12;
  }
  status = state.GetCameras(cameras);
  std::cout << "Found " << cameras.size() << " cameras" << std::endl;
  if (cameras.size() == 0) {
    return 13;
  }
  std::vector<FeatureID> features;
  status = state.GetFeatures(features);
  if (status != OKAY) {
    std::cerr << "Failed to get features: " << ErrorMessage(status) << std::endl;
    return 1000;
  }
  hasStorage = false;
  hasTemperatures = false;
  hasPoses = false;
  for (const auto& feature : features) {
    if (feature == STORAGE_API_AVAILABLE) {
      hasStorage = true;
    } else if (feature == TEMPERATURE_API_AVAILABLE) {
      hasTemperatures = true;
    } else if (feature == POSE_API_POSITION_AVAILABLE || feature == POSE_API_ORIENTATION_AVAILABLE) {
      hasPoses = true;
    }
  }

  // Find the trigger for the first camera, which we will use to synchronize to the display.  We assume that
  // they are all using the same trigger.  We don't send triggers when we replay.
  if (cameras.size() > 0 && replayStreamID == 0) {
    triggerID = cameras[0].trigger;
  }

  // If there is a map CSV file associated with this camera, read it into the point correspondence.
  {
    std::filesystem::path mapCSVPath = g_dirPath / (std::to_string(serialNumber) + ".map.csv");
    if (std::filesystem::exists(mapCSVPath)) {
      std::cout << "Reading map CSV file: " << mapCSVPath << std::endl;
      g_pointCorrespondences = std::make_shared<PointCorrespondences>(mapCSVPath.string());
    } else {
      g_pointCorrespondences.reset();
    }
  }

  // Read the configuration file associated with the serial number for the server. Verify that
  // it has a matching serial number and number of cameras.
  configPath = g_dirPath / (std::to_string(serialNumber) + ".json");
  std::vector<CameraRenderInfo> rawCameraRenderInfos;
  try {
    rawCameraRenderInfos = asdp::render::calibration::GetCameraRenderInfos(configPath.string());
  }
  catch (const std::exception& e) {
    std::cerr << "Error reading configuration file: " << e.what() << std::endl;
    return 14;
  }
  if (cameras.size() != rawCameraRenderInfos.size()) {
    std::cerr << "Number of cameras mismatch: expected " << cameras.size() << " but got " << rawCameraRenderInfos.size() << std::endl;
    return 16;
  }
  std::cout << "Read configuration from " << configPath << std::endl;

  // Remove any cameras that we are to skip from cameras, cameraInfos and NUCInfos.
  std::vector<CameraRenderInfo> filteredCameraRenderInfos;
  std::vector<CameraInfo> filteredCameras;
  for (size_t i = 0; i < cameras.size(); i++) {
    if (skipCameras.find(i + 1) == skipCameras.end()) {
      filteredCameras.push_back(cameras[i]);
    }
    else {
      std::cout << "Skipping camera ID " << i + 1 << std::endl;
    }
    if (skipCameras.find(rawCameraRenderInfos[i].m_ID) == skipCameras.end()) {
      filteredCameraRenderInfos.push_back(rawCameraRenderInfos[i]);
    }
    for (auto& nucInfo : nucInfos) {
      // Delete the camera NUC tables if we are skipping this camera.
      if (skipCameras.find(i + 1) != skipCameras.end()) {
        nucInfo.CameraNUCTables.erase(i + 1);
        std::cout << "  Skipping NUC tables for camera ID " << i + 1 << std::endl;
      }
    }
  }
  cameras = filteredCameras;
  std::cout << "After skipping, " << cameras.size() << " cameras remain." << std::endl;
  if (cameras.size() == 0) {
    std::cerr << "No cameras remain after skipping." << std::endl;
    return 15;
  }

  // If there are more cameras than the maximum, limit the number of cameras to the maximum.
  if (maxCameras > 0 && cameras.size() > maxCameras) {
    std::cout << "Limiting number of cameras to " << maxCameras << std::endl;
    cameras.resize(maxCameras);
    // Cannot use resize() here because there is not a default constructor for CameraInfo.
    while (filteredCameraRenderInfos.size() > maxCameras) {
      filteredCameraRenderInfos.pop_back();
    }
  }

  // Make additional OpenGL contexts for the texture threads.
  int NUM_TEXTURE_THREADS = 2;
  if (cameras.size() > 21) {
    // We need larger batches of lines to keep up with more than 21 cameras. The jump from
    // default 110 to 330 has both cases ending at 990, which is just below the 1024 limit so will make
    // a small final batch, reducing the latency from the end of the frame receipt to texture upload.s
    // NOTE: Originally, we could keep up on Linux by bumping our number of threads to 3 and leaving
    // the line batches the same. As of 4/24, this no longer works -- but depth estimation is now
    // taking much longer than it used to.  We collapsed to a common solution of more batches because
    // it keeps a small final batch, still reducing the latency with fewer threads.
    lineBatchesPerGPUSend *= 3;
  }

  std::vector< std::shared_ptr<DisplayTexture> > displayTextures;
  for (size_t i = 0; i < NUM_TEXTURE_THREADS; i++) {
    std::shared_ptr<DisplayTexture> dt = std::make_shared<DisplayTexture>(displayTexture.get());
    displayTextures.push_back(dt);
  }

  // Construct a vector of CameraRenderInfo objects from the configuration file, adding an image
  // queue to each.
  try {
    for (const auto& info : filteredCameraRenderInfos) {

      //==================================================================================================
      // Fill in three textures for this camera, all gray and at time zero.
      // We must borrow the context from the displayTexture so that we can create the textures.
      if (!displayTexture->BorrowContext()) {
        std::cerr << "Error borrowing context from displayTexture." << std::endl;
        return 17;
      }

      unsigned int width = info.m_resolutionPixels[0];
      unsigned int height = info.m_resolutionPixels[1];
      std::vector<uint16_t> image(width * height, 32767);

      // Create the textures for the camera. Make two for each Composite to pull when it is looking
      // for the next image to render, one for the texture thread to write to, one for an image-statistics
      // class to use, and one to lie fallow.
      // Also add as many frames as needed to render ahead the number we want to.
      for (size_t i = 0; i < 2 * displayInfos.size() + 1 + 1 + 1 + ceil(renderAheadFrames); i++) {
        std::shared_ptr<ImageData> imageData = std::make_shared<ImageData>();

        unsigned int texture;
        glGenTextures(1, &texture);
        glBindTexture(GL_TEXTURE_2D, texture);
        // Set the texture wrapping parameters
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        // Set texture filtering parameters
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        // Load image into the texture
        glTexImage2D(GL_TEXTURE_2D, 0, GL_R16, width, height, 0, GL_RED, GL_UNSIGNED_SHORT, image.data());
        glBindTexture(GL_TEXTURE_2D, 0);

        imageData->texture = texture;
        info.m_imageQueue->InsertImage(imageData);
      }

      if (!displayTexture->ReturnContext()) {
        std::cerr << "Error returning context to displayTexture." << std::endl;
        return 18;
      }
      //
      //==================================================================================================

      // Push the CameraRenderInfo onto the vector.
      cameraRenderInfos.push_back(std::make_shared<CameraRenderInfo>(info));
    }
  }
  catch (const std::exception& e) {
    std::cerr << "Error parsing configuration file: " << e.what() << std::endl;
    return 19;
  }

  cameraIDs.clear();
  for (uint32_t i = 0; i < cameraRenderInfos.size(); i++) {
    cameraIDs.push_back(cameraRenderInfos[i]->m_ID);
  }

  // If the camera FPS is not set, find the minimum period for one of the cameras and use that.
  // This assumes that all cameras capture at the same frame rate.
  if (cameraFPS == 0.0 && cameras.size() > 0) {
    cameraFPS = 1.0 / cameras[0].minTriggerPeriod;
  }
  std::cout << "Camera frame rate: " << cameraFPS << " fps" << std::endl;

  // Initialize the timing information, making an entry for each camera.  We make sure that there is
  // the maximum camera ID so that we can use the camera ID as an index.
  uint32_t maxID = 0;
  for (auto ID : cameraIDs) {
    if (ID > maxID) {
      maxID = ID;
    }
  }
  g_timingInfo.SetNumCameras(maxID);

  // Separate the cameras into two groups: those with IDs less than 22 are visible cameras and those
  // with larger ones are depth-estimation cameras. Only do this if we are computing depth.
  for (size_t j = 0; j < cameraRenderInfos.size(); j++) {
    if (cameraRenderInfos[j]->m_ID > 21 && computeDepth) {
      g_depthCameras.push_back(cameraRenderInfos[j]);
    }
    else {
      g_visibleCameras.push_back(cameraRenderInfos[j]);
    }
  }

  // If we have no remaining visible cameras, we cannot continue because there would be nothing to display.
  if (g_visibleCameras.size() == 0) {
    std::cerr << "No visible cameras remain after skipping." << std::endl;
    return 20;
  }

  // If we've been asked to do standard-deviation-based auto-ranging, set that up.
  std::shared_ptr<RangeEstimator> rangeEstimator = std::make_shared<RangeEstimatorFixed>();
  if (autoRangeStdAbove != 0 || autoRangeStdBelow != 0) {
    // Make a display object that shares textures with the others.
    std::shared_ptr<Display> display = std::make_shared<DisplayTexture>(displayTexture.get());
    // Make a MeanStdGroup object to handle the statistics.
    std::shared_ptr<asdp::render::imageStatistics::MeanStdGroup> meanStdGroup =
      std::make_shared<asdp::render::imageStatistics::MeanStdGroup>(g_visibleCameras,
        display, 1.0 / cameraFPS);
    // Make a RangeEstimator object to handle the range.
    rangeEstimator = std::make_shared<RangeEstimatorStdRanges>(meanStdGroup,
      autoRangeStdBelow, autoRangeStdAbove);
  }

  // Construct a depth-estimation object if there are any depth-estimation cameras.
  // There must be sets of two camera pairs for depth estimation.
  if (g_depthCameras.size() > 0) {
    depthContext = std::make_shared<DisplayTexture>(displayTexture.get());
    bool enablingCP = false; // Whether cylindrical projection is enabled for any of the displays.
    for (const auto& displayInfo : displayInfos) {
      if (displayInfo.enableCP) {
        enablingCP = true;
        break;
      }
    }
    if (enablingCP) {
      std::cerr << "Cylindrical projection is incompatible with depth estimation." << std::endl;
      return 100;
    }

    if (g_depthCameras.size() % 2 != 0) {
      std::cerr << "Error: There must be an even number of depth-estimation cameras." << std::endl;
      return 101;
    }
    std::vector< std::array<std::shared_ptr<asdp::render::CameraRenderInfo>, 2> > cameras;
    for (size_t i = 0; i < g_depthCameras.size(); i += 2) {
      std::array<std::shared_ptr<asdp::render::CameraRenderInfo>, 2> pair = { g_depthCameras[i], g_depthCameras[i + 1] };
      cameras.push_back(pair);
    }

    if (!depthContext->BorrowContext()) {
      std::cerr << "Error borrowing context from depthContext for DepthEstimator." << std::endl;
      return 102;
    }

    // Initialize GLEW in our context. It is okay to initialize it more than once.
    glewExperimental = true;
    if (glewInit() != GLEW_OK) {
      std::cerr << "Failed to initialize GLEW before DepthTexture" << std::endl;
      return 103;
    }
    // Clear any GL error that Glew caused.  Apparently on Non-Windows
    // platforms, this can cause a spurious error 1280.
    glGetError();

    // Determine the range of depths to use for the depth estimater and then construct it.
    std::vector<float> depths(7);
    depths[depths.size() - 1] = maxDepth;
    for (int i = depths.size() - 2; i >= 0; i--) {
      depths[i] = depths[i + 1] / 2;
    }
    g_depthEstimator = std::make_shared<DepthEstimator>(cameras, rangeEstimator, poseAdjuster, float(1.0 / cameraFPS),
      g_depthCameras[0]->m_resolutionPixels[0] * 2 / 100, g_depthCameras[0]->m_resolutionPixels[1] * 2 / 100,
      depths, depthThreshold);
    std::cout << "Constructed DepthEstimator with " << cameras.size() << " camera pairs." << std::endl;

    // Compute a depth estimate to get all of the machinery set up and GLEW initialized on this thread.
    g_depthEstimator->ComputeDepthEstimate(0);

    if (!depthContext->ReturnContext()) {
      std::cerr << "Error returning context to depthContext for DepthEstimator." << std::endl;
      return 104;
    }

    // Start the depth estimation thread.
    g_runDepthThread = true;
    g_depthThread = std::thread(DepthThreadFunction, timer, depthContext);
  }

  for (size_t i = 0; i < displayInfos.size(); i++) {

    // Construct a Tone Map texture to use for rendering the cameras.
    if (!displayTexture->BorrowContext()) {
      std::cerr << "Error borrowing context from displayTexture for ToneMap." << std::endl;
      return 21;
    }
    GLuint toneMapTexture = displayInfos[i].toneMap.GenerateTexture();
    toneMapTextures.push_back(toneMapTexture);
    if (toneMapTexture == 0) {
      std::cerr << "Error generating texture for ToneMap." << std::endl;
      return 22;
    }
    if (!displayTexture->ReturnContext()) {
      std::cerr << "Error returning context to displayTexture for ToneMap." << std::endl;
      return 23;
    }

    // Construct a Composite object to render the visible cameras.  We need a separate Composite per Display so that each
    // can cache consistent camera images for the whole frame while views are being rendered.
    // Two displays cannot share a SetupRenderFrame() call because they may have different frame rates.
    // Rendering offset based on how many frames we want to render ahead.
    uint32_t renderOffsetMicroseconds = renderAheadFrames * (1000000 / cameraFPS);
    g_composites.push_back(std::make_shared<CompositeCameras>(
      g_visibleCameras, toneMapTexture, poseAdjuster, Time(1 / cameraFPS),
      renderOffsetMicroseconds,
      Time(0, 1000000 / displayInfos[i].fps), (i == 0) ? (&g_timingInfo) : nullptr,
      rangeEstimator, staticDepth, AnnotationCallbackHandler, nullptr));

    //======================================
    // Added by Sang Yoon to just pass the status of enabling the cylindrical projection (true or false) from DisplayInfos[i] to composite.
    // Note that the cylinderical projection is processed in Composite Submodule.
    g_composites[i]->m_CP_enabled = displayInfos[i].enableCP;
    //======================================

    //======================================
    // Added by Sang Yoon to just pass the status of overview and detailed view for the current display to composite.
    // Note that the overview and detailed view are handled in Composite Submodule.
    g_composites[i]->m_overview = displayInfos[i].overview;
    g_composites[i]->m_detailed_view = displayInfos[i].detailed_view;
    //======================================
  }

  // Verify that the width and height of the cameras match the width and height of the NUC information,
  // if any NUC information was provided.
  for (const auto& nucInfo : nucInfos) {
    for (const auto& nucCam : nucInfo.CameraNUCTables) {
      uint32_t cameraID = nucCam.first;
      int nucWidth = nucCam.second.imageWidth;
      int nucHeight = nucCam.second.imageHeight;
      // Find the index of the camera whose CameraRenderInfo has this ID.
      int cameraIndex = -1;
      for (int i = 0; i < static_cast<int>(cameraRenderInfos.size()); i++) {
        if (cameraRenderInfos[i]->m_ID == cameraID) {
          cameraIndex = i;
          break;
        }
      }
      if (cameraIndex < 0) {
        std::cerr << "Error: NUC information for camera ID " << cameraID << " does not match any camera in the configuration file." << std::endl;
        return 100;
      }
      if ((nucWidth != cameras[cameraIndex].width) || (nucHeight != cameras[cameraIndex].height)) {
        std::cerr << "Error: NUC information for camera ID " << cameraID << " has dimensions (" <<
          nucWidth << ", " << nucHeight << ") that do not match the camera dimensions (" <<
          cameras[cameraIndex].width << ", " << cameras[cameraIndex].height << ")." << std::endl;
        return 101;
      }
    }
  }

  // Construct shared pointers to the data structures that we'll need to do rendering, with
  // custom destructors that will clean up when the shared_ptr is destroyed.
  std::vector< std::shared_ptr<CUDABufferPool> > cpuPinnedImageBuffers;
  std::vector< std::shared_ptr<CUDABufferPool> > gpuImageBuffers;
  std::vector< std::shared_ptr<CUDABufferPool> > gpuNUCBuffers;
  std::vector< std::shared_ptr<cudaStream_t> > streams;
  UDPReceivers.clear();
  for (size_t i = 0; i < cameras.size(); i++) {
    try {
      // Preallocate pinned memory buffers for the CPU.
      cpuPinnedImageBuffers.push_back(std::make_shared<CUDABufferPool>(cameras[i].width * cameras[i].height * sizeof(uint16_t), 10, true));

      // Preallocate memory buffers for the GPU.
      gpuImageBuffers.push_back(std::make_shared<CUDABufferPool>(cameras[i].width * cameras[i].height * sizeof(uint16_t), 10, false));

      // Make a pool of GPU memory buffers for per-pixel NUC to use if it is running.
      // Pre-allocate some if we are doing per-pixel NUC, otherwise leave the vector empty.
      size_t nucBufferCount = cameras.size() * 2 * nucInfos.size();
      gpuNUCBuffers.push_back(std::make_shared<CUDABufferPool>(cameras[i].width * cameras[i].height * sizeof(float), nucBufferCount, false));
    }
    catch (std::exception& e) {
      std::cerr << "Error creating buffer pools: " << e.what() << std::endl;
      return 26;
    }

    // Create a stream for the GPU to use.
    cudaStream_t* streamPtr = new cudaStream_t;
    cudaStreamCreate(streamPtr);
    streams.push_back(std::shared_ptr<cudaStream_t>(streamPtr,
      [](cudaStream_t* ptr) { cudaStreamDestroy(*ptr); delete ptr; }
    ));

    // Create a UDP receiver for the camera.
    std::shared_ptr<ReceiverUDP> receiverUDP = std::make_shared<ReceiverUDP>(ip_address);
    if (receiverUDP->GetConstructorStatus() != OKAY) {
      std::cerr << "Error constructing ReceiverUDP: " << ErrorMessage(receiverUDP->GetConstructorStatus()) << std::endl;
      return 27;
    }
    UDPReceivers.push_back(receiverUDP);
  }

  // Make the queues to pass the NUC data tables to the receive-data threads, one for each camera -- they
  // will be nullptr if there is no NUC information for that camera.  If we don't have any NUC information at all,
  // the vector will contain nullptr for each camera.
  std::vector< std::shared_ptr< SpinFreeQueue< NUCDataPair > > > nucTableQueues;
  for (size_t i = 0; i < cameras.size(); i++) {
    std::shared_ptr< SpinFreeQueue< NUCDataPair > > nucTableQueue;
    if (nucInfos.size()) {
      nucTableQueue = std::make_shared< SpinFreeQueue< NUCDataPair > >();
    }
    nucTableQueues.push_back(nucTableQueue);
  }

  // Go ahead and allocate and fill buffers for the NUC data tables for each camera that has NUC information,
  // so that the receive-data threads can use them right away when they start receiving data from the cameras.
  /// @todo In the future, we will use temperature information to interpolate between tables.  For now we, just
  /// use the first NUCInfo.
  if (nucInfos.size() > 0) {
    const NUCInfo& nucInfo = nucInfos[0];
    for (const auto& it : nucInfo.CameraNUCTables) {
      int cameraID = it.first;
      // Find the index of the camera whose CameraRenderInfo has this ID.
      int cameraIndex = -1;
      for (int i = 0; i < static_cast<int>(cameraRenderInfos.size()); i++) {
        if (cameraRenderInfos[i]->m_ID == cameraID) {
          cameraIndex = i;
          break;
        }
      }
      const NucTables& nucTable = it.second;
      if (cameraIndex < 0) {
        std::cerr << "Error: NUC information for camera ID " << cameraID << " does not match any camera in the configuration file." << std::endl;
        return 102;
      }
      NUCDataPair nucDataPair;
      nucDataPair.gainBuffer = gpuNUCBuffers[cameraIndex]->GetBuffer(true, 0);
      nucDataPair.offsetBuffer = gpuNUCBuffers[cameraIndex]->GetBuffer(true, 0);
      if (nucDataPair.gainBuffer == nullptr || nucDataPair.offsetBuffer == nullptr) {
        std::cerr << "Error: Failed to get NUC buffers for camera ID " << cameraID << std::endl;
        return 103;
      }

      // Use a CUDA copy to transfer the NUC tables to the GPU.
      size_t numPixels = cameras[cameraIndex].width * cameras[cameraIndex].height;
      cudaError_t cerr;
      cerr = cudaMemcpy(nucDataPair.gainBuffer.get(), nucTable.gainTable.data(),
        numPixels * sizeof(float), cudaMemcpyHostToDevice);
      if (cerr != cudaSuccess) {
        std::cerr << "Error: Failed to copy gain table to GPU for camera ID " << cameraID << ": " << cudaGetErrorString(cerr) << std::endl;
        return 104;
      }
      cerr = cudaMemcpy(nucDataPair.offsetBuffer.get(), nucTable.offsetTable.data(),
        numPixels * sizeof(float), cudaMemcpyHostToDevice);
      if (cerr != cudaSuccess) {
        std::cerr << "Error: Failed to copy offset table to GPU for camera ID " << cameraID << ": " << cudaGetErrorString(cerr) << std::endl;
        return 105;
      }

      // Queue the buffers for use by the receive-data thread for this camera.
      nucTableQueues[cameraIndex]->enqueue(nucDataPair);
    }
  }

  // Make the queues to pass data between the receiver and texture threads, one for each texture thread.
  // The cameras will be spread among the threads in a round-robin fashion.
  for (size_t i = 0; i < NUM_TEXTURE_THREADS; i++) {
    dataQueues.push_back(std::make_shared< SpinFreeQueue< std::shared_ptr<DataToSendToGPU> > >());
  }

  // Launch the threads to copy data to the GPU, each having its own queue.
  for (size_t i = 0; i < NUM_TEXTURE_THREADS; i++) {
    copyDataToGPUThreads.push_back(std::thread(CopyDataToTextures, cameras[0].width, cameras[0].height, std::ref(done),
      dataQueues[i], lineBatchesPerGPUSend, displayTextures[i], std::ref(g_timingInfo.cameras)));
  }

  // Launch the data receiving threads, hooking them together using the queues and passing the texture OpenGL
  // context to it.  Round-robin the data queues among the receive-data threads.
  for (size_t i = 0; i < cameras.size(); i++) {
    receiveDataThreads.push_back(std::thread(ReceiveDataThread, std::ref(*UDPReceivers[i]), 9000,
      std::ref(done), cpuPinnedImageBuffers[i], gpuImageBuffers[i], streams[i], cameraRenderInfos[i]->m_imageQueue,
      dataQueues[i % NUM_TEXTURE_THREADS],
      &g_timingInfo.cameras[i].frameBeginTimes, &g_timingInfo.cameras[i].frameEndTimes,
      nucTableQueues[i]));
  }

  // Ask for streaming pose and temperature data.
  if (doStreamPoses && hasPoses) {
    std::cout << "Requesting pose data." << std::endl;
    status = client->SendCommandPacket(CommandPacketStreamPoses());
    if (status != OKAY) {
      std::cerr << "Failed to request pose data: " << ErrorMessage(status) << std::endl;
      return 28;
    }
  }
  if (hasTemperatures) {
    std::cout << "Requesting temperature data." << std::endl;
    status = client->SendCommandPacket(CommandPacketStreamTemperatures());
    if (status != OKAY) {
      std::cerr << "Failed to request temperature data: " << ErrorMessage(status) << std::endl;
      return 29;
    }
  }

  // Request streaming on the cameras at their maximum rates from their associated ID.
  std::cout << "Streaming every " << frameStride << " frames from " << cameraIDs.size() << " cameras" << std::endl;
  for (size_t i = 0; i < cameras.size(); i++) {
    uint32_t camID = cameraIDs[i];
    CameraInfo& camera = cameras[i];

    TriggerInfo ti;
    ti.ID = camera.trigger;
    ti.mode = 3;
    ti.period = 1 / cameraFPS;
    ti.offset = 0;
    ti.trackingFactor = 0.005;
    ti.externalID = camera.trigger;
    status = client->SendCommandPacket(CommandPacketConfigureTrigger(ti));
    if (status != OKAY) {
      std::cerr << "Failed to configure trigger: " << ErrorMessage(status) << std::endl;
      return 30;
    }
    std::cout << std::setprecision(10) << "  Configured trigger for camera " << camID << " with period " << ti.period << " seconds" << std::endl;

    // Request the camera to stream full-frame images once every frameStride frames.
    uint16_t port;
    status = UDPReceivers[i]->GetPort(port);
    if (status != OKAY) {
      std::cerr << "Failed to get port: " << ErrorMessage(status) << std::endl;
      return 31;
    }
    StreamEndpoint endpoint(ip_address, port);
    SubregionDescription region;
    region.cameraID = camID;
    region.skipFrames = frameStride - 1;
    region.startTimeSeconds = 0;
    region.startTimeMicroseconds = 0;
    region.left = 0;
    region.top = 0;
    region.right = camera.width - 1;
    region.bottom = camera.height - 1;
    status = client->SendCommandPacket(CommandPacketStreamSubregion(endpoint, region));
    if (status != OKAY) {
      std::cerr << "Failed to stream images: " << ErrorMessage(status) << std::endl;
      return 32;
    }
  }

  // If we've been asked to replay a stream, then send a request to do this.
  if (replayStreamID) {
    if (!hasStorage) {
      std::cerr << "Error: Storage API not available when replay requested." << std::endl;
      return 33;
    }
    std::cout << "Requesting replay of stream " << replayStreamID << std::endl;
    // Set the initial time to be above zero so that we never predict backwards to negative time.
    status = client->SendCommandPacket(CommandPacketStartReplay(replayStreamID, Time(10, 0)));
    if (status != OKAY) {
      std::cerr << "Failed to start replay: " << ErrorMessage(status) << std::endl;
      return 34;
    }
  }

  // Create a ClockSynchronizer that will manage adjusting the timer based on clock-sync messages.
  clockSync = std::make_shared<ClockSynchronizer>(timer);

  // If there are any analysis modules, launch a thread to service all of them, passing it the vector of module URLs.
  if (!analysisModuleURLs.empty()) {
    g_runAnalysisThread = true;
    analysisThread = std::thread(HandleAnalysisThread, analysisModuleURLs, timer);
  }

  return 0;
}

/// @brief Static function to spin down all of the things related to connecting to a camera ball.
int spin_down(std::shared_ptr<CoreClient> client, std::atomic<bool>& done,
  std::vector<CameraInfo> &cameras, std::vector<uint32_t> &cameraIDs,
  std::vector< std::shared_ptr<ReceiverUDP> > &UDPReceivers, std::string &ip_address,
  std::shared_ptr<DisplayTexture>& depthContext, std::vector<std::thread>& copyDataToGPUThreads,
  std::thread &analysisThread,
  std::vector< std::shared_ptr< SpinFreeQueue< std::shared_ptr<DataToSendToGPU> > > > &dataQueues,
  std::vector<std::thread> &receiveDataThreads, std::shared_ptr<DisplayTexture> &displayTexture,
  std::vector< std::shared_ptr<asdp::render::CameraRenderInfo> > &cameraRenderInfos,
  std::vector<GLuint> &toneMapTextures
  )
{
  // Now borrow the context from the displayTexture so that we can free up resources.
  if (!displayTexture->BorrowContext()) {
    std::cerr << "Error borrowing context from displayTexture." << std::endl;
    return 36;
  }

  // Stopping streaming on the cameras
  std::cout << "Stop streaming from " << cameraIDs.size() << " cameras" << std::endl;
  for (size_t i = 0; i < cameras.size(); i++) {
    uint32_t camID = cameraIDs[i];
    CameraInfo& camera = cameras[i];

    // Request the camera to cancel streaming.
    uint16_t port;
    Status status = UDPReceivers[i]->GetPort(port);
    if (status != OKAY) {
      std::cerr << "Failed to get port: " << ErrorMessage(status) << std::endl;
      return 31;
    }
    StreamEndpoint endpoint(ip_address, port);
    status = client->SendCommandPacket(CommandPacketCancelSubregion(camID, endpoint));
    if (status != OKAY) {
      std::cerr << "Failed to stop streaming images: " << ErrorMessage(status) << std::endl;
      return 32;
    }
  }
  cameras.clear();
  cameraIDs.clear();

  // If we have a depth thread, shut it down and then join it.
  if (depthContext) {
    g_runDepthThread = false;
    if (g_depthThread.joinable()) {
      g_depthThread.join();
    }
    depthContext.reset();
  }

  // Set done and wait for all of our GPU data threads to join.
  done = true;
  for (auto& thread : copyDataToGPUThreads) {
    if (thread.joinable()) {
      thread.join();
    }
  }
  copyDataToGPUThreads.clear();

  // Shut down any analysis thread.
  g_runAnalysisThread = false;
  if (analysisThread.joinable()) {
    analysisThread.join();
  }

  // Now that all of the buffers have been returned to the buffer queue, join our receive-data threads.
  for (auto& thread : receiveDataThreads) {
    if (thread.joinable()) {
      thread.join();
    }
  }
  receiveDataThreads.clear();

  // Shut down all of the UDP receivers now that the receiving threads have finished.
  UDPReceivers.clear();

  // Clear all remaining data from the queues now that the receivers are done.
  // All of the receiving threads will also delete this before they exit, which will remove all of the
  // references and push their buffers back onto their empty queues.
  for (auto& queue : dataQueues) {
    queue.reset();
  }
  dataQueues.clear();

  cameraRenderInfos.clear();
  glDeleteTextures(toneMapTextures.size(), toneMapTextures.data());

  // Clean up the global objects.
  g_pointCorrespondenceDisplay.reset();
  g_visibleCameras.clear();
  g_depthCameras.clear();
  g_depthEstimator.reset();
  g_composites.clear();

  // We're done with the context.
  if (!displayTexture->ReturnContext()) {
    std::cerr << "Error returning context to displayTexture." << std::endl;
    return 37;
  }

  return 0;
}

//=================================================================

static void usage(std::string name)
{
  std::cerr << "Usage: " << name << " [options] <ip_address>" << std::endl;
  std::cerr << "  <ip_address>                        The IP address to listen for servers on." << std::endl;
  std::cerr << "  Options:" << std::endl;
  std::cerr << "  --help                              Print this help message." << std::endl;
  std::cerr << "  --version                           Print the version number and quit." << std::endl;
  std::cerr << "  --serial <serial number>            The serial number of the server to connect to (default -1 means any)." << std::endl;
  std::cerr << "  --maxCameras <int>                  The maximum number of cameras to render (default 0 means all)." << std::endl;
  std::cerr << "  --skipCamera <camera ID>            Skip rendering the specified camera ID, can be used multiple times." << std::endl;
  std::cerr << "  --frameStride <frame stride>        Read one out of every this many frames. Set to 1 for every frame." << std::endl;
  std::cerr << "  --toneMap <tone map>                The tone map to use.  Options are: linear blackbody bluesky 10bit balcony" << std::endl;
  std::cerr << "  --NUCInfo <directory> <tempType>    Add the directory containing the NUC information and the temperature type to use (sensor or core)." << std::endl;
  std::cerr << "  --replay <stream id>                ID of the stream to replay (1+)." << std::endl;
  std::cerr << "  --loopReplay                        Loop the replay (default not)." << std::endl;
  std::cerr << "  --lineBatchesPerGPUSend <int>       The number of batches of lines to group (default 16 Linux, 110 Windows)" << std::endl;
  std::cerr << "  --noPoses                           Do not stream poses from the server, so no latency adjustment." << std::endl;
  std::cerr << "  --dumpTiming <file name base>       Write timing on quit to CSV files with the specified base name." << std::endl;
  std::cerr << "  --duration <seconds>                The duration to run before quitting (default 0 means run until user quits)." << std::endl;
  std::cerr << "  --kiosk <JSON filename>             Run in kiosk mode with the specified configuration file." << std::endl;
  std::cerr << "  --addAnalysis <URL>                 Add an analysis module from the specified URL (can be used multiple times)." << std::endl;
  std::cerr << "  --addDisplay                        Add another display with defaults that can be overridden" << std::endl;
  std::cerr << "  --renderAheadMicroseconds <int>     Microseconds ahead of vertical retrace to start rendering next frame (default 2500)." << std::endl;
  std::cerr << "  --triggerAheadMicroseconds <int>    Microseconds ahead of render start to trigger camera (default 22000)." << std::endl;
  std::cerr << "  --depthAheadMicroseconds <int>      Microseconds ahead of render start to copy depth info (default 4000)." << std::endl;
  std::cerr << "  --lockRotation                      Lock the rotation of the viewer to the initial helicopter pose." << std::endl;
  std::cerr << "  --disableLatencyCompensation        Disable latency compensation." << std::endl;
  std::cerr << "  --autoRangeStd <below> <above>      Adjust color range to specified standard deviations above and below the mean." << std::endl;
  std::cerr << "  --noDepth                           Do not compute depth even when stereo cameras are available." << std::endl;
  std::cerr << "  --maxDepth <float>                  Maximum depth to test for in meters (default 200)." << std::endl;
  std::cerr << "  --depthThreshold <float>            Depth threshold in squared pixel value differences (default 10.0)." << std::endl;
  std::cerr << "  --staticDepth <double>              The static depth to use for cameras without depth information (default 900.0)." << std::endl;
  std::cerr << "  --cameraFPS <frames per second>     The frames per second to run the camera at (default is maximum rate)." << std::endl;
  std::cerr << "  --enableCP                          Enable the cylindrical projection." << std::endl; // Added by Sang Yoon
  std::cerr << "  --enableOD                          Enable the display interface of overview plus detail view." << std::endl; // Added by Sang Yoon
  std::cerr << "  --openXR                            Use OpenXR for rendering. If set, overrides the following and sets lineBatchesPerGPUSend to 10000." << std::endl;
  std::cerr << "  --xSight <ip of NIC to listen on>   <display> Render to XSight on specified NIC. If set, overrides the following." << std::endl;
  std::cerr << "  --xSight2 <ip of NIC to listen on>  <display> Render to a color, smaller XSight on specified NIC. If set, overrides the following." << std::endl;
  std::cerr << "  --width <width>                     The width of the window (default 1280)." << std::endl;
  std::cerr << "  --height <height>                   The height of the window (default 1024)." << std::endl;
  std::cerr << "  --hFOV <horizontal field of view>   The horizontal field of view in degrees (default 40)." << std::endl;
  std::cerr << "  --joystick <string>                 The joystick to use for input (e.g. GLFW::0)." << std::endl;
  std::cerr << "  --fps <frames per second>           The frames per second to run at (default 60)." << std::endl;
  std::cerr << "  --fullScreen <display>              Run in full screen mode on the specified display (0+)." << std::endl;
  std::cerr << "  --viewpointOffset <x> <y> <z>       The viewpoint offset to apply to all cameras in meters." << std::endl;
};

int main(int argc, char** argv)
{
  int serialNumber = -1;        ///< The serial number of the server to connect to, -1 means any.
  int maxCameras = 0;           ///< The maximum number of cameras to render, 0 means all.
  std::set<int> skipCameras;    ///< The set of camera IDs to skip rendering.
  uint32_t frameStride = 1;     ///< Read one out of every this many frames. Set to 1 for every frame.
  std::vector<DisplayInfo> displayInfos = { DisplayInfo() }; ///< Information for each display that is to be created.
  std::string ip_address;       ///< The IP address to listen on.
  uint32_t replayStreamID = 0;  ///< The stream ID to replay, 0 for live.
  double renderAheadFrames = 0; ///< The number of frames to render ahead of the current frame, set nonzero for replay.
  bool loopReplay = false;      ///< Loop the replay when it reaches the end if this is true.
#ifdef _WIN32
  // On Windows, throughput tests when receiving data from the network show that we must be larger
  // to keep up.  Linux is more efficient here, and can handle 16 batches at a time.
  int lineBatchesPerGPUSend = 110; ///< The number of batches of lines to group for sending to the GPU.
#else
  int lineBatchesPerGPUSend = 16; ///< The number of batches of lines to group for sending to the GPU.
#endif
  bool doStreamPoses = true;      ///< Stream poses from the server, so we can adjust for latency.
  std::string dumpTimingFileName; ///< The base name for the timing files.
  unsigned triggerAheadMicroseconds = 22000;  ///< Microseconds ahead of render to trigger camera.
  unsigned depthAheadMicroseconds = 4000;     ///< Microseconds ahead of render to copy depth info.
  unsigned renderAheadMicroseconds = 2500;    ///< Microseconds ahead of vertical retrace to start rendering the frame.
  bool lockRotation = false;      ///< Lock the rotation of the viewer to the initial helicopter pose.
  bool disableLatencyCompensation = false; ///< Disable latency compensation.
  double cameraFPS = 0.0;         ///< The frames per second to run the camera at, 0 defaults to camera-specified maximum.
  double autoRangeStdBelow = 0.0; ///< Adjust color range to this many standard deviations below the mean.
  double autoRangeStdAbove = 0.0; ///< Adjust color range to this many standard deviations above the mean.
  bool computeDepth = true;       ///< Compute depth when stereo cameras are available.
  float maxDepth = 200.0f;        ///< Maximum depth to test for in meters.
  float depthThreshold = 10.0f;   ///< Depth threshold in squared pixel value differences.
  double staticDepth = 900.0;     ///< The static depth to use for cameras without depth information.
  int durationSeconds = 0;        ///< The duration to run before quitting, 0 means run until user quits.
  std::string kioskConfigFile;    ///< The name of the kiosk configuration file to use, if any.
  std::vector<NUCInfo> nucInfos;  ///< All instances of NUC information for all cameras.
  std::vector<std::string> analysisModuleURLs; ///< The URLs of analysis modules to load.
  //======================================
  // Added by Sang Yoon to add a command line argument to enable the display interface of overview plus detail view.
  bool enableOD = false;          ///< The flag to enable/disable the display interface of overview plus detail view.
  //======================================

  size_t realParams = 0;          ///< The number of non-flag parameters we've seen.

  // Parse the command line arguments, with the first non-flag argument being the
  // name of the IP address to listen on.
  for (int i = 1; i < argc; ++i) {
    if (std::string("--frameStride") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 2;
      }
      frameStride = std::stoi(argv[i]);
    }
    else if (std::string("--width") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 2;
      }
      displayInfos.back().width = std::stoi(argv[i]);
    }
    else if (std::string("--height") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 2;
      }
      displayInfos.back().height = std::stoi(argv[i]);
    }
    else if (std::string("--hFOV") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 2;
      }
      displayInfos.back().hFOV = std::stof(argv[i]);
    }
    else if (std::string("--openXR") == argv[i]) {
      displayInfos.back().useOpenXR = true;
      lineBatchesPerGPUSend = 10000;
    }
    else if (std::string("--xSight") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 2;
      }
      displayInfos.back().XSightNIC = argv[i];
      if (++i >= argc) {
        usage(argv[0]);
        return 2;
      }
      displayInfos.back().XSightDisplay = std::stoi(argv[i]);
    }
    else if (std::string("--xSight2") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 2;
      }
      displayInfos.back().XSight2NIC = argv[i];
      if (++i >= argc) {
        usage(argv[0]);
        return 2;
      }
      displayInfos.back().XSight2Display = std::stoi(argv[i]);
    }
    else if (std::string("--viewpointOffset") == argv[i]) {
      if (i + 3 >= argc) {
        usage(argv[0]);
        return 2;
      }
      displayInfos.back().viewpointOffset[0] = std::stof(argv[++i]);
      displayInfos.back().viewpointOffset[1] = std::stof(argv[++i]);
      displayInfos.back().viewpointOffset[2] = std::stof(argv[++i]);
    }
    else if (std::string("--fullScreen") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 2;
      }
      displayInfos.back().fullScreen = true;
      displayInfos.back().fullScreenDisplay = std::stoi(argv[i]);
    } else if (std::string("--fps") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 2;
      }
      displayInfos.back().fps = std::stof(argv[i]);
    } else if (std::string("--joystick") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 2;
      }
      displayInfos.back().joystick = argv[i];
    } else if (std::string("--lineBatchesPerGPUSend") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 2;
      }
      lineBatchesPerGPUSend = std::stoi(argv[i]);
    } else if (std::string("--toneMap") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 2;
      }
      if (std::string("linear") == argv[i]) {
        displayInfos.back().toneMap = ToneMap();
      } else if (std::string("10bit") == argv[i]) {
        float maxFraction = 1023.0f / 65535; // 10-bit max value as fraction of 16-bit max value.
        displayInfos.back().toneMap = ToneMap({{0.0, 0.0,0.0,0.0}, {maxFraction, 1.0,1.0,1.0}});
      } else if (std::string("blackbody") == argv[i]) {
        displayInfos.back().toneMap = ToneMapBlackbody();
      } else if (std::string("bluesky") == argv[i]) {
        displayInfos.back().toneMap = ToneMapBlueSky();
      } else if (std::string("balcony") == argv[i]) {
        displayInfos.back().toneMap = ToneMap({{0.0, 0.0,0.0,0.0}, {0.5, 0.0,0.0,0.0},{1.0, 1.0,1.0,1.0}});
      } else {
        std::cerr << "Unknown tone map: " << argv[i] << std::endl;
        return 2;
      }
    } else if (std::string("--NUCInfo") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 2;
      }
      std::shared_ptr<NUCInfo> nucInfo = ReadNUCInfoFromDirectory(argv[i]);
      if (++i >= argc) {
        usage(argv[0]);
        return 2;
      }
      if (nucInfo == nullptr) {
        std::cerr << "Failed to read NUC information from directory: " << argv[i-1] << std::endl;
        return 2;
      }
      nucInfo->temperatureType = argv[i];
      nucInfos.push_back(*nucInfo);
    } else if (std::string("--addAnalysis") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 2;
      }
      analysisModuleURLs.push_back(argv[i]);
    } else if (std::string("--addDisplay") == argv[i]) {
      displayInfos.push_back(DisplayInfo());
    } else if (std::string("--replay") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 2;
      }
      replayStreamID = std::stoi(argv[i]);
      renderAheadFrames = 4.0; // This seemed best as of 6/20/2025; 3.5 caused wobble in 25-cams, 20-25 was not better than 4.0.
    } else if (std::string("--loopReplay") == argv[i]) {
      loopReplay = true;
    } else if (std::string("--noPoses") == argv[i]) {
      doStreamPoses = false;
    } else if (std::string("--dumpTiming") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 2;
      }
      dumpTimingFileName = argv[i];
    } else if (std::string("--duration") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 2;
      }
      durationSeconds = std::stoi(argv[i]);
    } else if (std::string("--kiosk") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 2;
      }
      kioskConfigFile = argv[i];
    } else if (std::string("--renderAheadMicroseconds") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 2;
      }
      renderAheadMicroseconds = std::stoi(argv[i]);
    } else if (std::string("--triggerAheadMicroseconds") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 2;
      }
      triggerAheadMicroseconds = std::stoi(argv[i]);
    } else if (std::string("--depthAheadMicroseconds") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 2;
      }
      depthAheadMicroseconds = std::stoi(argv[i]);
    } else if (std::string("--lockRotation") == argv[i]) {
      lockRotation = true;
    } else if (std::string("--disableLatencyCompensation") == argv[i]) {
      disableLatencyCompensation = true;
    } else if (std::string("--autoRangeStd") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 2;
      }
      autoRangeStdBelow = std::stod(argv[i]);
      if (++i >= argc) {
        usage(argv[0]);
        return 2;
      }
      autoRangeStdAbove = std::stod(argv[i]);
    } else if (std::string("--noDepth") == argv[i]) {
      computeDepth = false;
    } else if (std::string("--maxDepth") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 2;
      }
      maxDepth = std::stof(argv[i]);
    } else if (std::string("--depthThreshold") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 2;
      }
      depthThreshold = std::stof(argv[i]);
    } else if (std::string("--staticDepth") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 2;
      }
      staticDepth = std::stod(argv[i]);
    } else if (std::string("--cameraFPS") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 2;
      }
      cameraFPS = std::stod(argv[i]);
    } else if (std::string("--help") == argv[i]) {
      usage(argv[0]);
      return 0;

    //======================================
    // Added by Sang Yoon to add a command line argument to enable the cylindrical projection for the current display.
    // Note that each display (or window) can use either perspective projection (default) or cylindrical projection (enabled with --enableCP).
    }
    else if (std::string("--enableCP") == argv[i]) {
        displayInfos.back().enableCP = true;
    //======================================

    //======================================
    // Added by Sang Yoon to add a command line argument to enable the display interface of overview plus detail view.
    } else if (std::string("--enableOD") == argv[i]) {
        enableOD = true;
    //======================================
    } else if (std::string("--maxCameras") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 2;
      }
      maxCameras = std::stoi(argv[i]);
    } else if (std::string("--skipCamera") == argv[i]) {
      if (++i >= argc) {
        std::cerr << "--skipCamera requires a camera ID" << std::endl;
        usage(argv[0]);
        return 2;
      }
      skipCameras.insert(std::stoi(argv[i]));
    } else if (std::string("--serial") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 2;
      }
      serialNumber = std::stoi(argv[i]);
    } else if (std::string("--version") == argv[i]) {
      std::cout << "ASDP Render Module version " << VERSION + "-" + BUILD_TYPE << std::endl;
      return 0;
    } else if (argv[i][0] == '-') {
      usage(argv[0]);
      return 1;
    } else switch (realParams++) {
    case 0:
      ip_address = argv[i];
      break;
    default:
      usage(argv[0]);
      return 2;
    }
  }
  if (realParams != 1) {
    usage(argv[0]);
    return 2;
  }

  // Verify that none of the displays have a horizontal field of view greater than 360 degrees
  // or equal to or greater than 180 degrees unless --enableCP is set for that display.
  for (size_t i = 0; i < displayInfos.size(); i++) {
    if (displayInfos[i].hFOV > 360.0f) {
      std::cerr << "Error: Display " << i << " has a horizontal field of view greater than 360 degrees." << std::endl;
      return 2;
    }
    if (displayInfos[i].hFOV <= 0.0f) {
      std::cerr << "Error: Display " << i << " has a horizontal field of view less than 0 degrees." << std::endl;
      return 2;
    }
    if (displayInfos[i].hFOV >= 180.0f && !displayInfos[i].enableCP) {
      std::cerr << "Error: Display " << i << " has a horizontal field of view equal to or greater than 180 degrees without --enableCP." << std::endl;
      return 2;
    }
  }

  //======================================
  // Added by Sang Yoon to enable the display interface of overview plus detail view.
  if (enableOD) {
    // Determine overview window and detail view window.
    // Where the number of displays is greater than 1, the widest window is considered as an overview window,
    // and the narrowest window is considered as a detail view window.
    int overview_displayID = -1; // display ID of overview window
    int detailed_view_displayID = -1; // display ID of detailed view window

    if (displayInfos.size() > 1) {
      float widest_hFOV = 0.0f;
      float narrowest_hFOV = 360.0f;
      for (size_t i = 0; i < displayInfos.size(); i++) {
        if (displayInfos[i].hFOV >= widest_hFOV) {
          widest_hFOV = displayInfos[i].hFOV;
          overview_displayID = i;
        }
        if (displayInfos[i].hFOV < narrowest_hFOV || displayInfos[i].useOpenXR) {
          narrowest_hFOV = displayInfos[i].hFOV;
          detailed_view_displayID = i;
        }
      }
      displayInfos[overview_displayID].overview = true;
      displayInfos[detailed_view_displayID].detailed_view = true;
    }
  }
  //======================================

  // If we are connected to one or more analysis modules, increment the rendering start time by 1ms.
  if (!analysisModuleURLs.empty()) {
    std::cout << "Moving rendering forwards by 1ms to allow time for analysis display." << std::endl;
    renderAheadMicroseconds += 1000;
  }

  // Run inside a block so that the destructors will be called for all objects before we exit.
  {
    std::cout << "ASDP Render Module version " << VERSION + "-" + BUILD_TYPE << " using Core API "
      << asdp::Core::GetVersion() << std::endl;

    //=================================================================
    // Open a client, specifying the IP address to listen on.
    std::shared_ptr<CoreClient> client = std::make_shared<CoreClient>(ip_address);
    if (client->GetConstructorStatus() != OKAY) {
      std::cerr << "Failed to open client: " << ErrorMessage(client->GetConstructorStatus()) << std::endl;
      return 3;
    }
    std::cout << "Listening for servers on " << ip_address << std::endl;

    // Wait for up to two seconds to allow servers to send Discovery messages.
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    std::map<uint32_t, std::string> servers;
    Status threadStatus;
    Status status;
    do {
      status = client->GetDiscoveryThreadStatus(threadStatus);
      if (status != OKAY) {
        std::cerr << "Failed to get discovery thread status: " << ErrorMessage(status) << std::endl;
        return 4;
      }
      if (threadStatus != OKAY) {
        std::cerr << "Discovery thread status: " << ErrorMessage(threadStatus) << std::endl;
        return 5;
      }
      status = client->IdentifiedServers(servers);
      if (status != OKAY) {
        std::cerr << "Failed to get identified servers: " << ErrorMessage(status) << std::endl;
        return 6;
      }
      // If we have been asked for a specific serial number, break when we have found it.
      // Otherwise, break when we have found any server.
      if (serialNumber >= 0) {
        if (servers.find(serialNumber) != servers.end()) { break; }
      } else {
        if (!servers.empty()) { break; }
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() <= 2.0);

    // Construct a DisplayTexture object to handle textures.  It will be the base object that all others will use
    // to share contexts.
    std::shared_ptr<DisplayTexture> displayTexture = std::make_shared<DisplayTexture>();

    // Construct a Display for use by the point correspondence object, if any.
    if (g_pointCorrespondences != nullptr) {
      g_pointCorrespondenceDisplay = std::make_shared<DisplayTexture>(displayTexture.get());
    }

    //=================================================================
    // If we are running in kiosk mode, parse the configuration file.
    json kioskInfo;
    if (!kioskConfigFile.empty()) {
      std::cout << "Running in kiosk mode with configuration file: " << kioskConfigFile << std::endl;
      std::ifstream kioskFile(kioskConfigFile);
      if (!kioskFile.is_open()) {
        std::cerr << "Failed to open kiosk configuration file: " << kioskConfigFile << std::endl;
        return 2;
      }
      try {
        kioskFile >> kioskInfo;
      }
      catch (const std::exception& e) {
        std::cerr << "Failed to parse kiosk configuration file: " << e.what() << std::endl;
        return 2;
      }

      // Make sure the kiosk contains an array
      if (!kioskInfo.is_array()) {
        std::cerr << "Kiosk configuration file does not contain an array." << std::endl;
        return 2;
      }

      // Make sure every entry contains a "afterSeconds", "command", and "parameters" field
      // and that "afterSeconds" is a number, "command" is a string, and "parameters" is an array.
      for (const auto& entry : kioskInfo) {
        if (!entry.contains("afterSeconds") || !entry.contains("command") || !entry.contains("parameters")) {
          std::cerr << "Kiosk configuration file entry does not contain required fields." << std::endl;
          return 2;
        }
        if (!entry["afterSeconds"].is_number() || !entry["command"].is_string() || !entry["parameters"].is_array()) {
          std::cerr << "Kiosk configuration file entry has incorrect field types." << std::endl;
          return 2;
        }
      }

      // Make the display for the kiosk
      g_kioskDisplay = std::make_shared<DisplayTexture>(displayTexture.get());
    }

    std::shared_ptr<Timer> timer;
    status = client->GetTimer(timer);
    if (status != OKAY) {
      std::cerr << "Failed to get timer: " << ErrorMessage(status) << std::endl;
      return 300;
    }

    // Keeps track of when we were paused so we can adjust our clock synchronization when resumed.
    Time pausedTime = {};

    // Spin up the client, which will connect to the server and fill in lots of information.
    std::shared_ptr<Receiver> receiver;
    std::vector<CameraInfo> cameras;
    std::vector<uint32_t> cameraIDs; ///< The camera IDs to render, in the same order as the records in the configuration file.
    std::vector< std::shared_ptr<asdp::render::CameraRenderInfo> > cameraRenderInfos;
    bool hasStorage = false;
    bool hasTemperatures = false;
    bool hasPoses = false;
    uint8_t triggerID = 0;
    std::filesystem::path configPath;
    std::vector< std::shared_ptr<ReceiverUDP> > UDPReceivers;
    std::vector<GLuint> toneMapTextures;  ///< Stores these for later deletion or replacement.
    std::atomic<bool> done{false};
    std::shared_ptr<ClockSynchronizer> clockSync;
    std::shared_ptr<DisplayTexture> depthContext;
    std::vector<std::thread> copyDataToGPUThreads;
    std::thread analysisThread;
    std::vector< std::shared_ptr< SpinFreeQueue< std::shared_ptr<DataToSendToGPU> > > > dataQueues;
    std::vector<std::thread> receiveDataThreads;
    std::shared_ptr<PoseAdjuster> poseAdjuster;
    int ret = spin_up(client, serialNumber, receiver, cameras, hasStorage, hasTemperatures, hasPoses,
      triggerID, replayStreamID, configPath, UDPReceivers, skipCameras, nucInfos, maxCameras,
      lineBatchesPerGPUSend, displayTexture, displayInfos, renderAheadFrames, cameraFPS, computeDepth,
      autoRangeStdBelow, autoRangeStdAbove, maxDepth, depthThreshold, poseAdjuster, cameraRenderInfos,
      cameraIDs,
      toneMapTextures, staticDepth, done, ip_address, doStreamPoses, frameStride, analysisModuleURLs,
      clockSync, timer, depthContext, copyDataToGPUThreads,
      analysisThread, dataQueues, receiveDataThreads, lockRotation, disableLatencyCompensation);
    if (ret != 0) {
      return ret;
    }

    // Configure an event structure to handle callbacks for the display windows.
    std::shared_ptr<EventHandlers> handlers = std::make_shared<EventHandlers>();
    handlers->ChangePlayPause = ChangePlayPause;
    if (g_depthEstimator) {
      handlers->CopyDepthInfo = CopyDepthInfo;
    }
    handlers->SetToRenderDepth = SetDepthRendering;
    handlers->IncrementActiveCamera = IncrementActiveCamera;
    handlers->DecrementActiveCamera = DecrementActiveCamera;
    handlers->AdjustActiveCameraOffset = AdjustActiveCameraOffset;
    handlers->AdjustActiveCameraGain = AdjustActiveCameraGain;
    handlers->AutoUpdateColorOffsets = AutoUpdateColorOffsets;
    handlers->AutoUpdateColorOffsetsAndGains = AutoUpdateColorOffsetsAndGains;
    handlers->SaveCameraConfig = SaveCameraConfig;
    handlers->ShowCameraNames = ShowCameraNames;
    handlers->ResetAnalysis = ResetAnalysis;
    g_callbackHandlerData.cameraConfigFileName = configPath.string();

    // Construct one or more Display objects to render the cameras.  They all share objects with the texture Display.
    std::vector<std::shared_ptr<Display>> displays;

    for (size_t i = 0; i < displayInfos.size(); i++) {

      // Only time the first listed display, to avoid race conditions
      if (displayInfos[i].useOpenXR) {
        displays.push_back(std::make_shared<DisplayOpenXR>(g_composites[i], displayTexture.get(),
          client, triggerID, triggerAheadMicroseconds, depthAheadMicroseconds, displayInfos[i].viewpointOffset,
          renderAheadMicroseconds, 1, handlers, &g_callbackHandlerData,
          (i == 0) ? (&g_timingInfo) : nullptr, replayStreamID != 0));
      } else if (!displayInfos[i].XSightNIC.empty()) {
        displays.push_back(std::make_shared<DisplayXSight>(displayInfos[i].XSightNIC, g_composites[i], displayTexture.get(),
          client, triggerID, triggerAheadMicroseconds,
          depthAheadMicroseconds, displayInfos[i].viewpointOffset,
          renderAheadMicroseconds,
          handlers, nullptr,
          (i == 0) ? (&g_timingInfo) : nullptr, replayStreamID != 0,
          displayInfos[i].XSightDisplay
        ));
      } else if (!displayInfos[i].XSight2NIC.empty()) {
        displays.push_back(std::make_shared<DisplayXSight>(displayInfos[i].XSight2NIC, g_composites[i], displayTexture.get(),
          client, triggerID, triggerAheadMicroseconds,
          depthAheadMicroseconds, displayInfos[i].viewpointOffset,
          renderAheadMicroseconds,
          handlers, nullptr,
          (i == 0) ? (&g_timingInfo) : nullptr, replayStreamID != 0,
          displayInfos[i].XSight2Display,
          1920, 1200, 50, 70.0f,
          false
        ));
      } else {
        displays.push_back(std::make_shared<DisplayWindow>("ASDP Render Module " + std::to_string(i),
          g_composites[i], client, triggerID, triggerAheadMicroseconds, depthAheadMicroseconds, displayInfos[i].viewpointOffset,
          displayInfos[i].fps, renderAheadMicroseconds,
          displayInfos[i].width, displayInfos[i].height,
          displayInfos[i].hFOV, displayInfos[i].joystick, displayTexture.get(),
          displayInfos[i].fullScreen, displayInfos[i].fullScreenDisplay, false, handlers, &g_callbackHandlerData,
          (i == 0) ? (&g_timingInfo) : nullptr, replayStreamID != 0));
      }
      if (displays.back()->GetStatus() != "") {
        std::cerr << "Error constructing Display " << i << ": " << displays.back()->GetStatus() << std::endl;
        displays.clear();
        return 25;
      }
    }

    // Render frames until someone has marked us to be done.
    bool nowPaused = false;
    bool replayDone = false;
    start = std::chrono::steady_clock::now();
    auto kioskStart = start;
    size_t kioskIndex = 0; // The index of the current kiosk entry to check.
    while (!done) {

      // See if it is time to switch to the next kiosk entry, if any.
      if (kioskIndex < kioskInfo.size()) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - kioskStart).count();
        if (elapsed >= kioskInfo[kioskIndex]["afterSeconds"]) {
          std::cout << "Activating next kiosk entry: " << kioskInfo[kioskIndex]["command"];
          for (const auto& param : kioskInfo[kioskIndex]["parameters"]) {
            std::cout << " " << param;
          }
          std::cout << std::endl;

          // Handle the command for this kiosk entry.
          std::string command = kioskInfo[kioskIndex]["command"];
          if (command == "pause") {
            if (replayStreamID) {
              client->SendCommandPacket(CommandPacketPauseReplay());
              g_paused = true;
            }
          } else if (command == "resume") {
            if (replayStreamID) {
              client->SendCommandPacket(CommandPacketResumeReplay());
              g_paused = false;
            }
          } else if (command == "toneMap") {
            if (kioskInfo[kioskIndex]["parameters"].size() != 1) {
              std::cerr << "Error: toneMap command requires one parameter." << std::endl;
            }
            std::string toneMapName = kioskInfo[kioskIndex]["parameters"][0];
            auto toneMap = ToneMap();
            if (toneMapName == "linear") {
              toneMap = ToneMap();
            } else if (toneMapName == "10bit") {
              float maxFraction = 1023.0f / 65535; // 10-bit max value as fraction of 16-bit max value.
              toneMap = ToneMap({ {0.0, 0.0,0.0,0.0}, {maxFraction, 1.0,1.0,1.0} });
            } else if (toneMapName == "blackbody") {
              toneMap = ToneMapBlackbody();
            } else if (toneMapName == "bluesky") {
              toneMap = ToneMapBlueSky();
            } else if (toneMapName == "balcony") {
              toneMap = ToneMap({ {0.0, 0.0,0.0,0.0}, {0.5, 0.0,0.0,0.0},{1.0, 1.0,1.0,1.0} });
            } else {
              std::cerr << "Error: Unknown tone map name: " << toneMapName << ", setting to linear" << std::endl;
            }
            // Fill all of the existing tonemap textures with this one.
            if (!g_kioskDisplay->BorrowContext()) {
              std::cerr << "Error borrowing context from g_kioskDisplay for ToneMap." << std::endl;
            }
            for (size_t i = 0; i < toneMapTextures.size(); i++) {
              if (!toneMap.FillTexture(toneMapTextures[i])) {
                std::cerr << "Error filling tone map texture." << std::endl;
              }
            }
            if (!g_kioskDisplay->ReturnContext()) {
              std::cerr << "Error returning context to g_kioskDisplay for ToneMap." << std::endl;
            }
          } else if (command == "autoRangeStd") {
            if (kioskInfo[kioskIndex]["parameters"].size() != 2) {
              std::cerr << "Error: autoRangeStd command requires two parameters." << std::endl;
            }
            if (!kioskInfo[kioskIndex]["parameters"][0].is_number() ||
                !kioskInfo[kioskIndex]["parameters"][1].is_number()) {
              std::cerr << "Error: autoRangeStd command parameters must be numbers." << std::endl;
            }
            float stdBelow = kioskInfo[kioskIndex]["parameters"][0].get<float>();
            float stdAbove = kioskInfo[kioskIndex]["parameters"][1].get<float>();

            std::shared_ptr<RangeEstimator> rangeEstimator;
            if (stdBelow == 0 && stdAbove == 0) {
              rangeEstimator = nullptr;
            } else {
              // Make a display object that shares textures with the others.
              std::shared_ptr<Display> display = std::make_shared<DisplayTexture>(displayTexture.get());
              // Make a MeanStdGroup object to handle the statistics.
              std::shared_ptr<asdp::render::imageStatistics::MeanStdGroup> meanStdGroup =
                std::make_shared<asdp::render::imageStatistics::MeanStdGroup>(g_visibleCameras,
                  display, 1.0 / cameraFPS);
              rangeEstimator = std::make_shared<RangeEstimatorStdRanges>(meanStdGroup, stdBelow, stdAbove);
            }
            for (auto& composite : g_composites) {
              composite->UpdateRangeEstimator(rangeEstimator);
            }
          } else if (command == "camera") {
            if (kioskInfo[kioskIndex]["parameters"].size() != 2) {
              std::cerr << "Error: camera command requires one parameter." << std::endl;
            }
            if (!kioskInfo[kioskIndex]["parameters"][0].is_number() ||
                !kioskInfo[kioskIndex]["parameters"][1].is_number()) {
              std::cerr << "Error: camera command parameters must be numbers." << std::endl;
            }
            int cameraID = kioskInfo[kioskIndex]["parameters"][0];
            int streamID = kioskInfo[kioskIndex]["parameters"][1];

            // Remove the old composite in each of the displays before we spin down so that everything
            // will be released (in particular, the CameraRenderInfo->m_imageQueue textures).
            // Wait briefly to enable any in-progress rendering to finish before we tear things down.
            for (size_t i = 0; i < displays.size(); i++) {
              displays[i]->UpdateComposite(nullptr);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            // Remove any RangeEstimator from the composites so that their resources will be released.
            for (auto& composite : g_composites) {
              composite->UpdateRangeEstimator(nullptr);
            }

            // Spin down the existing camera
            ret = spin_down(client, done, cameras, cameraIDs, UDPReceivers, ip_address, depthContext,
              copyDataToGPUThreads, analysisThread, dataQueues, receiveDataThreads, displayTexture,
              cameraRenderInfos, toneMapTextures);
            if (ret != 0) {
              return ret;
            }

            // Change the serial number and stream ID.
            serialNumber = cameraID;
            replayStreamID = streamID;

            // Spin up the new camera.
            ret = spin_up(client, serialNumber, receiver, cameras, hasStorage, hasTemperatures, hasPoses,
              triggerID, replayStreamID, configPath, UDPReceivers, skipCameras, nucInfos, maxCameras,
              lineBatchesPerGPUSend, displayTexture, displayInfos, renderAheadFrames, cameraFPS, computeDepth,
              autoRangeStdBelow, autoRangeStdAbove, maxDepth, depthThreshold, poseAdjuster, cameraRenderInfos,
              cameraIDs,
              toneMapTextures, staticDepth, done, ip_address, doStreamPoses, frameStride, analysisModuleURLs,
              clockSync, timer, depthContext, copyDataToGPUThreads,
              analysisThread, dataQueues, receiveDataThreads, lockRotation, disableLatencyCompensation);
            if (ret != 0) {
              return ret;
            }

            // Set the new composites in each of the displays.
            for (size_t i = 0; i < displays.size(); i++) {
              displays[i]->UpdateComposite(g_composites[i]);
            }
            std::cout << " Switched to camera ID " << cameraID << " and stream ID " << streamID << std::endl;
          } else {
            std::cerr << "Error: Unknown kiosk command." << std::endl;
          }

          // Go to the next (or back to the first) kiosk entry.
          kioskStart = std::chrono::steady_clock::now();
          kioskIndex++;
          if (kioskIndex >= kioskInfo.size()) {
            std::cout << "Kiosk entries completed, resetting." << std::endl;
            kioskIndex = 0;
          }
        }
      }

      // Receive and handle any message from the server, waiting at most 100ms for a
      // new packet before looping back around.
      std::shared_ptr<StreamPacket> response;
      size_t offset = 0;
      Status status = receiver->ReceiveStreamPacket(0.1, response, offset);
      if (status == OKAY) {
        status = HandleStreamPacket(response, clockSync, poseAdjuster, replayDone, displays, timer, pausedTime);
        if (status != OKAY) {
          std::cerr << "Error handling stream packet: " << ErrorMessage(status) << std::endl;
          done = true;
        }
      } else if (status != TIMEOUT) {
        std::cerr << "Error receiving stream packet: " << ErrorMessage(status) << std::endl;
        done = true;
      }

      // If we've been asked to loop replays and replay is done, request a new replay with the offset
      // at the current time.
      if (replayDone && loopReplay) {
        std::cout << "Replay done, requesting new replay." << std::endl;
        Time nowTime;
        status = timer->GetCoreTime(nowTime);
        if (status != OKAY) {
          std::cerr << "Failed to get time: " << ErrorMessage(status) << std::endl;
          done = true;
        }
        status = client->SendCommandPacket(CommandPacketStartReplay(replayStreamID, nowTime));
        if (status != OKAY) {
          std::cerr << "Failed to start replay: " << ErrorMessage(status) << std::endl;
          done = true;
        }
        replayDone = false;
      }

      // If all of our Displays have been closed (or are broken), then we're done.
      bool allClosed = true;
      for (auto& display : displays) {
        if (display->GetStatus() == "") {
          allClosed = false;
          break;
        }
      }
      if (allClosed) {
        done = true;
      }

      // If our state of play/pause has switched and we're pausing or replaying, send a command to the server.
      if (replayStreamID) {
        if (nowPaused != g_paused) {
          if (g_paused) {
            status = client->SendCommandPacket(CommandPacketPauseReplay());
          } else {
            status = client->SendCommandPacket(CommandPacketResumeReplay());
          }
          nowPaused = g_paused;
        }
      }

      // See if we have been running for longer than the requested time, and if so, mark done.
      if (durationSeconds > 0) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();
        if (elapsed >= durationSeconds) {
          std::cout << "Run time of " << durationSeconds << " seconds has elapsed, exiting." << std::endl;
          done = true;
        }
      }
    }

    ret = spin_down(client, done, cameras, cameraIDs, UDPReceivers, ip_address, depthContext,
      copyDataToGPUThreads, analysisThread, dataQueues, receiveDataThreads, displayTexture,
      cameraRenderInfos, toneMapTextures);
    if (ret != 0) {
      return ret;
    }

    // Destroy our client ONLY AFTER spin_down; we don't want to do this inside spin_down,
    // because kiosk mode needs it to remain alive when switching cameras.
    client.reset();

    // Destroy kiosk mode display context, if any.
    g_kioskDisplay.reset();

    // Delete our display devices.
    for (auto& display : displays) {
      display.reset();
    }

    // If we've been asked to dump the timing information, do so.
    if (!dumpTimingFileName.empty()) {

      // Find the maximum number of entries in any of the timing vectors.
      size_t maxEntries = 0;
      if (g_timingInfo.renderStartTimes.size() > maxEntries) {
        maxEntries = g_timingInfo.renderStartTimes.size();
      }
      if (g_timingInfo.renderSubmitTimes.size() > maxEntries) {
        maxEntries = g_timingInfo.renderSubmitTimes.size();
      }
      if (g_timingInfo.depthComputeStartTimes.size() > maxEntries) {
        maxEntries = g_timingInfo.depthComputeStartTimes.size();
      }
      if (g_timingInfo.depthComputeEndTimes.size() > maxEntries) {
        maxEntries = g_timingInfo.depthComputeEndTimes.size();
      }
      if (g_timingInfo.depthStartTimes.size() > maxEntries) {
        maxEntries = g_timingInfo.depthStartTimes.size();
      }
      if (g_timingInfo.depthEndTimes.size() > maxEntries) {
        maxEntries = g_timingInfo.depthEndTimes.size();
      }
      for (size_t i = 0; i < g_timingInfo.cameras.size(); i++) {
        if (g_timingInfo.cameras[i].frameBeginTimes.size() > maxEntries) {
          maxEntries = g_timingInfo.cameras[i].frameBeginTimes.size();
        }
        if (g_timingInfo.cameras[i].frameEndTimes.size() > maxEntries) {
          maxEntries = g_timingInfo.cameras[i].frameEndTimes.size();
        }
        if (g_timingInfo.cameras[i].textureTimes.size() > maxEntries) {
          maxEntries = g_timingInfo.cameras[i].textureTimes.size();
        }
      }

      //==================================================================================================
      // Write the raw file.
      std::string rawTimingFileName = dumpTimingFileName + ".csv";
      std::ofstream dumpTimingFile(rawTimingFileName);
      std::cout << "Dumping " << maxEntries << " raw timing information to " << rawTimingFileName << std::endl;
      dumpTimingFile << "Depth Compute Start,Depth Compute End,Depth Copy Start,Depth Copy End,Render start,Render submit";
      for (size_t i = 0; i < g_timingInfo.cameras.size(); i++) {
        dumpTimingFile << ",Camera " << i+1 << " frame begin,Camera " << i+1
          << " frame end,Camera " << i+1 << " texture complete,Camera " << i+1
          << " center time seconds, Camera " << i+1 << " center time microseconds";
      }
      dumpTimingFile << std::endl;
      dumpTimingFile << std::setprecision(20);
      for (size_t i = 0; i < maxEntries; i++) {
        if (i < g_timingInfo.depthComputeStartTimes.size()) {
          dumpTimingFile << TimeIntervalToStringMilliseconds(g_timingInfo.depthComputeStartTimes[i] - g_timingInfo.startTime);
        }
        dumpTimingFile << ",";
        if (i < g_timingInfo.depthComputeEndTimes.size()) {
          dumpTimingFile << TimeIntervalToStringMilliseconds(g_timingInfo.depthComputeEndTimes[i] - g_timingInfo.startTime);
        }
        dumpTimingFile << ",";
        if (i < g_timingInfo.depthStartTimes.size()) {
          dumpTimingFile << TimeIntervalToStringMilliseconds(g_timingInfo.depthStartTimes[i] - g_timingInfo.startTime);
        }
        dumpTimingFile << ",";
        if (i < g_timingInfo.depthEndTimes.size()) {
          dumpTimingFile << TimeIntervalToStringMilliseconds(g_timingInfo.depthEndTimes[i] - g_timingInfo.startTime);
        }
        dumpTimingFile << ",";
        if (i < g_timingInfo.renderStartTimes.size()) {
          dumpTimingFile << TimeIntervalToStringMilliseconds(g_timingInfo.renderStartTimes[i] - g_timingInfo.startTime);
        }
        dumpTimingFile << ",";
        if (i < g_timingInfo.renderSubmitTimes.size()) {
          dumpTimingFile << TimeIntervalToStringMilliseconds(g_timingInfo.renderSubmitTimes[i] - g_timingInfo.startTime);
        }
        // No comma here; we'll append them with the following
        for (size_t j = 0; j < g_timingInfo.cameras.size(); j++) {
          dumpTimingFile << ",";
          if (i < g_timingInfo.cameras[j].frameBeginTimes.size()) {
            dumpTimingFile << TimeIntervalToStringMilliseconds(g_timingInfo.cameras[j].frameBeginTimes[i] - g_timingInfo.startTime);
          }
          dumpTimingFile << ",";
          if (i < g_timingInfo.cameras[j].frameEndTimes.size()) {
            dumpTimingFile << TimeIntervalToStringMilliseconds(g_timingInfo.cameras[j].frameEndTimes[i] - g_timingInfo.startTime);
          }
          dumpTimingFile << ",";
          if (i < g_timingInfo.cameras[j].textureTimes.size()) {
            dumpTimingFile << TimeIntervalToStringMilliseconds(g_timingInfo.cameras[j].textureTimes[i] - g_timingInfo.startTime);
          }
          if (i < g_timingInfo.cameras[j].centerRenderTimes.size()) {
            dumpTimingFile << ",";
            dumpTimingFile << std::to_string(g_timingInfo.cameras[j].centerRenderTimes[i].seconds);
            dumpTimingFile << ",";
            dumpTimingFile << std::to_string(g_timingInfo.cameras[j].centerRenderTimes[i].microseconds);
          } else {
            dumpTimingFile << ",,";
          }
        }
        dumpTimingFile << std::endl;
      }
      dumpTimingFile.close();

      //==================================================================================================
      // Write the intervals file.
      std::string intervalTimingFileName = dumpTimingFileName + "_intervals.csv";
      std::ofstream intervalTimingFile(intervalTimingFileName);
      std::cout << "Dumping " << maxEntries-1 << " interval timing information to " << intervalTimingFileName << std::endl;
      intervalTimingFile << "Depth compute start,Depth compute end,Depth copy start interval,Depth copy end interval,Render start interval,Render submit interval";
      for (size_t i = 0; i < g_timingInfo.cameras.size(); i++) {
        intervalTimingFile << ",Camera " << i+1 << " frame begin interval, " << i+1 << " frame end interval,Camera"
          << i+1 << " texture complete interval,Camera " << i+1 << " center time interval";
      }
      intervalTimingFile << std::endl;
      intervalTimingFile << std::setprecision(20);
      for (size_t i = 1; i < maxEntries; i++) {
        if (i < g_timingInfo.depthComputeStartTimes.size()) {
          intervalTimingFile << TimeIntervalToStringMilliseconds(g_timingInfo.depthComputeStartTimes[i] - g_timingInfo.depthComputeStartTimes[i - 1]);
        }
        intervalTimingFile << ",";
        if (i < g_timingInfo.depthComputeEndTimes.size()) {
          intervalTimingFile << TimeIntervalToStringMilliseconds(g_timingInfo.depthComputeEndTimes[i] - g_timingInfo.depthComputeEndTimes[i - 1]);
        }
        intervalTimingFile << ",";
        if (i < g_timingInfo.depthStartTimes.size()) {
          intervalTimingFile << TimeIntervalToStringMilliseconds(g_timingInfo.depthStartTimes[i] - g_timingInfo.depthStartTimes[i - 1]);
        }
        intervalTimingFile << ",";
        if (i < g_timingInfo.depthEndTimes.size()) {
          intervalTimingFile << TimeIntervalToStringMilliseconds(g_timingInfo.depthEndTimes[i] - g_timingInfo.depthEndTimes[i - 1]);
        }
        intervalTimingFile << ",";
        if (i < g_timingInfo.renderStartTimes.size()) {
          intervalTimingFile << TimeIntervalToStringMilliseconds(g_timingInfo.renderStartTimes[i] - g_timingInfo.renderStartTimes[i - 1]);
        }
        intervalTimingFile << ",";
        if (i < g_timingInfo.renderSubmitTimes.size()) {
          intervalTimingFile << TimeIntervalToStringMilliseconds(g_timingInfo.renderSubmitTimes[i] - g_timingInfo.renderSubmitTimes[i - 1]);
        }
        // No comma here; we'll append them with the following
        for (size_t j = 0; j < g_timingInfo.cameras.size(); j++) {
          intervalTimingFile << ",";
          if (i < g_timingInfo.cameras[j].frameBeginTimes.size()) {
            intervalTimingFile << TimeIntervalToStringMilliseconds(g_timingInfo.cameras[j].frameBeginTimes[i] - g_timingInfo.cameras[j].frameBeginTimes[i - 1]);
          }
          intervalTimingFile << ",";
          if (i < g_timingInfo.cameras[j].frameEndTimes.size()) {
            intervalTimingFile << TimeIntervalToStringMilliseconds(g_timingInfo.cameras[j].frameEndTimes[i] - g_timingInfo.cameras[j].frameEndTimes[i - 1]);
          }
          intervalTimingFile << ",";
          if (i < g_timingInfo.cameras[j].textureTimes.size()) {
            intervalTimingFile << TimeIntervalToStringMilliseconds(g_timingInfo.cameras[j].textureTimes[i] - g_timingInfo.cameras[j].textureTimes[i - 1]);
          }
          intervalTimingFile << ",";
          if (i < g_timingInfo.cameras[j].centerRenderTimes.size()) {
            Time diff = g_timingInfo.cameras[j].centerRenderTimes[i] - g_timingInfo.cameras[j].centerRenderTimes[i - 1];
            double diffms = diff.seconds * 1000.0 + diff.microseconds / 1000.0;
            intervalTimingFile << std::to_string(diffms);
          }
        }
        intervalTimingFile << std::endl;
      }
      intervalTimingFile.close();

      //==================================================================================================
      // Write a summary file that describes the min and max behavior for each frame that has camera timing info.
      std::string summaryTimingFileName = dumpTimingFileName + "_summary.csv";
      std::ofstream summaryTimingFile(summaryTimingFileName);
      std::cout << "Dumping summary timing information to " << summaryTimingFileName << std::endl;
      summaryTimingFile << "Depth compute start to end,Depth copy start to end,Depth copy end to render start,Render start to submit,Render start interval,Min camera end to render"
        << ",Max camera end to render, Min camera texture to render, Max camera texture to render"
        << ",Min center interval,Max center interval" << std::endl;
      summaryTimingFile << std::setprecision(20);
      for (size_t i = 1; i < g_timingInfo.renderStartTimes.size(); i++) {
        // Find the entry in compute end times that finishes must closely before the render start time.
        // If there is one, compute the difference between the associated start time and end time and record
        // it. Otherwise, don't put anything.
        if (i < g_timingInfo.renderSubmitTimes.size()) {
          auto t = g_timingInfo.renderSubmitTimes[i];
          int index = g_timingInfo.depthComputeEndTimes.size();
          for (int j = g_timingInfo.depthComputeEndTimes.size() - 1; j >= 0; j--) {
            if (g_timingInfo.depthComputeEndTimes[j] < t) {
              index = j;
              break;
            }
          }
          if (index < g_timingInfo.depthComputeEndTimes.size()) {
            summaryTimingFile << TimeIntervalToStringMilliseconds(g_timingInfo.depthComputeEndTimes[index] - g_timingInfo.depthComputeStartTimes[index]);
          }
        }
        summaryTimingFile << ",";
        if (i < g_timingInfo.depthStartTimes.size() && i < g_timingInfo.depthEndTimes.size()) {
          summaryTimingFile << TimeIntervalToStringMilliseconds(g_timingInfo.depthEndTimes[i] - g_timingInfo.depthStartTimes[i]);
        }
        summaryTimingFile << ",";
        if (i < g_timingInfo.depthEndTimes.size() && i < g_timingInfo.renderStartTimes.size()) {
          summaryTimingFile << TimeIntervalToStringMilliseconds(g_timingInfo.renderStartTimes[i] - g_timingInfo.depthEndTimes[i]);
        }
        summaryTimingFile << ",";
        if (i < g_timingInfo.renderSubmitTimes.size()) {
          summaryTimingFile << TimeIntervalToStringMilliseconds(g_timingInfo.renderSubmitTimes[i] - g_timingInfo.renderStartTimes[i]);
        }
        summaryTimingFile << ",";
        summaryTimingFile << TimeIntervalToStringMilliseconds(g_timingInfo.renderStartTimes[i] - g_timingInfo.renderStartTimes[i - 1]);
        summaryTimingFile << ",";

        // Find the largest camera end-frame time less than the render start time for each camera, and then find the
        // minimum and maximum of these.
        std::chrono::steady_clock::time_point minTime = std::chrono::steady_clock::time_point::max();
        std::chrono::steady_clock::time_point maxTime = std::chrono::steady_clock::time_point::min();
        for (size_t j = 0; j < g_timingInfo.cameras.size(); j++) {
          std::chrono::steady_clock::time_point t = LargestTimeLessThan(g_timingInfo.renderStartTimes[i], g_timingInfo.cameras[j].frameEndTimes);
          minTime = std::min(minTime, t);
          maxTime = std::max(maxTime, t);
        }
        if (minTime == std::chrono::steady_clock::time_point::min()) {
          summaryTimingFile << ",,";
        } else {
          summaryTimingFile << TimeIntervalToStringMilliseconds(g_timingInfo.renderStartTimes[i] - maxTime) << ",";
          summaryTimingFile << TimeIntervalToStringMilliseconds(g_timingInfo.renderStartTimes[i] - minTime) << ",";
        }

        // Find the largest texture time less than the render start time for each camera, and then find the
        // minimum and maximum of these.
        minTime = std::chrono::steady_clock::time_point::max();
        maxTime = std::chrono::steady_clock::time_point::min();
        for (size_t j = 0; j < g_timingInfo.cameras.size(); j++) {
          std::chrono::steady_clock::time_point t = LargestTimeLessThan(g_timingInfo.renderStartTimes[i], g_timingInfo.cameras[j].textureTimes);
          minTime = std::min(minTime, t);
          maxTime = std::max(maxTime, t);
        }
        if (minTime == std::chrono::steady_clock::time_point::min()) {
          summaryTimingFile << "," << ",";
        } else {
          summaryTimingFile << TimeIntervalToStringMilliseconds(g_timingInfo.renderStartTimes[i] - maxTime) << ",";
          summaryTimingFile << TimeIntervalToStringMilliseconds(g_timingInfo.renderStartTimes[i] - minTime) << ",";
        }

        // Find the minimum and maximum center render intervals.
        Time minCenterInterval = { 1000000, 0 };
        Time maxCenterInterval = { 0, 0 };
        for (size_t j = 0; j < g_timingInfo.cameras.size(); j++) {
          if (g_timingInfo.cameras[j].centerRenderTimes.size() > i) {
            Time diff = g_timingInfo.cameras[j].centerRenderTimes[i] - g_timingInfo.cameras[j].centerRenderTimes[i - 1];
            if (diff < minCenterInterval) {
              minCenterInterval = diff;
            }
            if (diff > maxCenterInterval) {
              maxCenterInterval = diff;
            }
          }
        }
        summaryTimingFile << std::to_string(minCenterInterval.seconds * 1000.0 + minCenterInterval.microseconds / 1000.0);
        summaryTimingFile << ",";
        summaryTimingFile << std::to_string(maxCenterInterval.seconds * 1000.0 + maxCenterInterval.microseconds / 1000.0);
        summaryTimingFile << std::endl;
      }
      summaryTimingFile.close();
    }

  } // End of block that causes destruction of all objects before returning.

  return 0;
}
