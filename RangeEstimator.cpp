/*
 * Copyright (C) 2025: Arizona Board of Regents on Behalf of the University of Arizona
 */

 /**
  * @file RangeEstimator.h
  * @brief Apache Strap-Down Pilotage Render/Display RangeEstimator implementation file.
  *
 * @author ReliaSolve.
 * @date January 29, 2025.
 */

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <Display.h>
#include <ImageStatistics.h>

#include <algorithm>
#include "RangeEstimator.h"

using namespace asdp::render;
using namespace asdp::render::imageStatistics;

std::string RangeEstimatorFixed::GetCurrentRange(double& minVal, double& maxVal)
{
  std::lock_guard<std::mutex> lock(m_mutex);
  minVal = m_minVal;
  maxVal = m_maxVal;
  return "";
}

std::string RangeEstimatorFixed::SetCurrentRange(double minVal, double maxVal)
{
  if (minVal < 0) {
    return "minVal must be >= 0";
  }
  if (maxVal > 1) {
    return "maxVal must be <= 1";
  }
  std::lock_guard<std::mutex> lock(m_mutex);
  m_minVal = minVal;
  m_maxVal = maxVal;
  return "";
}

std::string RangeEstimatorStdRanges::GetCurrentRange(double& minVal, double& maxVal)
{
  if (m_meanStdGroup == nullptr) {
    minVal = 0.0;
    maxVal = 1.0;
    return "MeanStdGroup is null.";
  }

  double mean, stddev;
  // This call is thread-safe because the MeanStdGroup is thread-safe.
  std::string error = m_meanStdGroup->GetMeanStd(mean, stddev);
  if (!error.empty()) {
    minVal = 0.0;
    maxVal = 1.0;
    return error;
  }

  // Scale the values to the range 0-1.
  mean /= 65535.0;
  stddev /= 65535.0;

  minVal = std::max(0.0, mean - m_numStdBelow * stddev);
  maxVal = std::min(1.0, mean + m_numStdAbove * stddev);
  return "";
}

//================================================================================================
// Test helper functions and methods below.

std::string RangeEstimator::Test()
{
  //================================================================================================
  // Test the fixed range estimator.

  RangeEstimatorFixed fixed(0.1, 0.8);
  double minVal, maxVal;
  std::string error = fixed.GetCurrentRange(minVal, maxVal);
  if (!error.empty()) {
    return "RangeEstimatorFixed::GetCurrentRange failed.";
  }

  if (minVal != 0.1 || maxVal != 0.8) {
    return "RangeEstimatorFixed::GetCurrentRange returned incorrect values.";
  }

  RangeEstimatorFixed fixedDefault;
  error = fixedDefault.GetCurrentRange(minVal, maxVal);
  if (!error.empty()) {
    return "RangeEstimatorFixed::GetCurrentRange default failed.";
  }
  if (minVal != 0.0 || maxVal != 1.0) {
    return "RangeEstimatorFixed::GetCurrentRange default returned incorrect values.";
  }

  fixedDefault.SetCurrentRange(0.2, 0.7);
  error = fixedDefault.GetCurrentRange(minVal, maxVal);
  if (!error.empty()) {
    return "RangeEstimatorFixed::SetCurrentRange failed.";
  }
  if (minVal != 0.2 || maxVal != 0.7) {
    return "RangeEstimatorFixed::SetCurrentRange set incorrect values.";
  }

  //================================================================================================
  // Test the standard deviation range estimator.

  // Create a window and OpenGL context.
  if (!glfwInit()) {
    return "Could not initialize GLFW";
  }
  glfwWindowHint(GLFW_VISIBLE, false);
  std::shared_ptr<GLFWwindow> window(glfwCreateWindow(640, 480, "RangeEstimator Test", NULL, NULL), glfwDestroyWindow);
  if (!window) {
    return "Could not create GLFW window";
  }
  glfwMakeContextCurrent(window.get());

  // Initialize GLEW in our context. It is okay to initialize it more than once.
  glewExperimental = true;
  if (glewInit() != GLEW_OK) {
    return "Could not initialize GLEW";
  }
  // Clear any GL error that Glew caused.  Apparently on Non-Windows
  // platforms, this can cause a spurious error 1280.
  glGetError();

  // Make the display object that we'll use and borrow its context.
  std::shared_ptr<Display> display(new DisplayTexture());
  if (!display->BorrowContext()) {
    return "Display::BorrowContext() failed";
  }

  // Make a camera to use that has a half values of 20000 and half of 40000.
  uint16_t width = 1280;
  uint16_t height = 1024;
  DistortionNone* dNone = new DistortionNone();
  std::shared_ptr<Distortion> distortion(dNone);
  VignetteNone* vNone = new VignetteNone();
  std::shared_ptr<Vignette> vignette(vNone);

  // Make first camera.
  std::shared_ptr<ImageData> image1(new ImageData);
  std::shared_ptr<ImageQueue> queue1(new ImageQueue);
  std::shared_ptr<CameraRenderInfo> camera1(new CameraRenderInfo(
    1, { 0, 0, 0 }, { 0, 0, 0 }, { width, height }, { 90.0, 90.0 }, distortion, vignette, queue1, -1.0f));

  // Add an image to the queue that has half values of 20000 and half of 40000.
  size_t imgSize = static_cast<size_t>(width) * height;
  std::vector<uint16_t> image(imgSize, 20000);
  for (size_t i = 0; i < imgSize / 2; i++) {
    image[i] = 40000;
  }

  // Construct an OpenGL texture and copy the image into it.
  GLuint texture1;
  glGenTextures(1, &texture1);
  glBindTexture(GL_TEXTURE_2D, texture1);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_R16, width, height, 0, GL_RED, GL_UNSIGNED_SHORT, image.data());
  glBindTexture(GL_TEXTURE_2D, 0);
  image1->texture = texture1;
  queue1->InsertImage(image1);

  // Done with the display context.
  if (!display->ReturnContext()) {
    return "Display::ReturnContext() failed";
  }

  // Make a vector of cameras and construct the MeanStdGroup with a 0.1-second iteration time.
  std::vector< std::shared_ptr<CameraRenderInfo> > cameras = { camera1 };
  std::shared_ptr<MeanStdGroup> meanStdGroup = std::make_shared<MeanStdGroup>(cameras, display, 0.1);

  // Wait long enough for it to fill up.
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Construct the RangeEstimatorStdRanges with 1 standard deviation below and 2 above.
  RangeEstimatorStdRanges stdRanges(meanStdGroup, 1, 2);

  // Get the range from it.
  // We expect to be 1 standard deviation (which is 10000) below the mean (which is 30000) and 2 above:
  // low is 20000, high is 50000 after scaling up by 65535.
  error = stdRanges.GetCurrentRange(minVal, maxVal);
  if (!error.empty()) {
    return "RangeEstimatorStdRanges::GetCurrentRange failed.";
  }
  minVal = std::round(minVal * 65535);
  maxVal = std::round(maxVal * 65535);
  if (minVal != 20000 || maxVal != 50000) {
    return "RangeEstimatorStdRanges::GetCurrentRange returned incorrect values.";
  }

  return "";
}
