/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include <iostream>
#include <vector>
#include <chrono>
#include <cstdint>
#include <memory>
#include <GL/glew.h>
#include <ToneMap.h>
#include <Composite.h>
#include <ASDP_Core_API.h>
#include <GLFW/glfw3.h>

/// @brief Make an image whose brightness varies from the top of the image to the bottom.
/// @details The image will be a gradient from the minimum value at the bottom to the
/// maximum value at the top.  The image order matches that of stored images, which is
/// different from OpenGL texture order; the first pixel is at the upper left of the image.
/// OpenGL must have been initialized before calling this function.
/// @param width Width of the image.
/// @param height Height of the image.
/// @param minVal Minimum value of the image (at the bottom of the image).
/// @param maxVal Maximum value of the image (at the top of the image).
/// @return An OpenGL texture ID.
GLuint MakeTexture(int width, int height, uint16_t minVal, uint16_t maxVal)
{
  std::vector<uint16_t> image(width * height);
  float range = static_cast<float>(maxVal - minVal);
  for (int j = 0; j < height; j++) {
    float normJ = static_cast<float>(j) / static_cast<float>(height - 1);
    for (int i = 0; i < width; i++) {
      image[i + j*width] = static_cast<uint16_t>(maxVal - normJ*range);
    }
  }

  unsigned int texture;
  glGenTextures(1, &texture);
  glBindTexture(GL_TEXTURE_2D, texture);
  // Set the texture wrapping parameters
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
  // Set texture filtering parameters
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

   // Load image into texture
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, width, height, 0, GL_RED, GL_UNSIGNED_SHORT, image.data());
  glBindTexture(GL_TEXTURE_2D, 0);

  return texture;
}

int main()
{
  int windowSize = 640;
  int width = 1280;
  int height = 1024;
  int nx = 3;
  int ny = 3;
  double hFOV = 40;
  double vFOV = 32.5;
  double degreesPerSecond = 15.0;

  asdp::render::ViewRenderInfo viewRenderInfo;
  viewRenderInfo.width = windowSize;
  viewRenderInfo.height = windowSize;
  // Set the view to be +/- 60 degrees so we can see more of the scene
  viewRenderInfo.bottomHalfFOV = -60;
  viewRenderInfo.topHalfFOV = 60;
  viewRenderInfo.leftHalfFOV = -60;
  viewRenderInfo.rightHalfFOV = 60;
  std::vector<asdp::render::ViewRenderInfo> views;
  views.push_back(viewRenderInfo);

  // Initialize the library
  if (!glfwInit()) {
    std::cerr << "Failed to initialize GLFW\n";
    return -1;
  }

  // Create a windowed mode window and its OpenGL context
  GLFWwindow* window = glfwCreateWindow(windowSize, windowSize, "CompositeCameras_Test", NULL, NULL);
  if (!window) {
    std::cerr << "Failed to create GLFW window\n";
    glfwTerminate();
    return -1;
  }

  // Make the window's context current
  glfwMakeContextCurrent(window);

  // Initialize GLEW in our context. It is okay to initialize it more than once.
  glewExperimental = true;
  if (glewInit() != GLEW_OK) {
    std::cerr << "Failed to initialize GLEW" << std::endl;
    return 4;
  }
  // Clear any GL error that Glew caused.  Apparently on Non-Windows
  // platforms, this can cause a spurious error 1280.
  glGetError();

  // Construct the cameras to render.
  // Construct the image queues to render, one per camera.
  std::vector<asdp::render::CameraRenderInfo> cameras;
  for (int x = 0; x < nx; x++) {
    uint16_t minVal = static_cast<uint16_t>(x * (65535.0/3) / nx);
    for (int y = 0; y < ny; y++) {
      // Odd-numbered columns are rotated with X facing up, even with it facing down.
      // The transformations are complicated by the fact that our Euler order of operations
      // is XYZ.  We need to rotate around X by 90 or -90 degrees to point straight up or down.
      // We then need to rotate around the the new Y axis by -90 plus the desired Y rotation
      // so that the original X axis will be pointing down.  Finally, we need to rotate around
      // the new Z axis by 90 + the desired vertical rotation.
      double desiredHor = 1.01 * (x - (nx - 1) / 2.0) * vFOV;  ///< Camera is rotated portrait
      double desiredVer = 1.01 * (y - (ny - 1) / 2.0) * hFOV;  ///< Camera is rotated portrait
      std::array<double, 3> orientationDegrees;
      if (x % 2 == 0) {
        orientationDegrees[0] = 90;
        orientationDegrees[1] = -90 - desiredHor;
        orientationDegrees[2] = 90 - desiredVer;
      }
      else {
        orientationDegrees[0] = 90;
        orientationDegrees[1] = 90 - desiredHor;
        orientationDegrees[2] = -90 + desiredVer;
      }

      // Make the image for the camera.
      uint16_t maxVal = static_cast<uint16_t>(2 * (65535.0 / 3) + y * (65535.0 / 3) / ny);
      std::shared_ptr<asdp::render::ImageData> image = std::make_shared<asdp::render::ImageData>();
      image->texture = MakeTexture(width, height, minVal, maxVal);
      std::shared_ptr<asdp::render::ImageQueue> iq = std::make_shared<asdp::render::ImageQueue>();
      // We must insert two images because the renderer will grab the newest two images.
      iq->InsertImage(image);
      iq->InsertImage(image);

      // Give each camera a unique ID
      asdp::render::CameraRenderInfo camera(x * ny + y,
        std::array<double, 3>(), orientationDegrees,
        std::array<uint16_t, 2>(), std::array<double, 2>({ hFOV, vFOV }),
        nullptr, iq);

      cameras.push_back(camera);
    }
  }

  // Use the default tone-map texture for the cameras.
  asdp::render::ToneMap toneMap;
  GLuint toneMapTexture = toneMap.GenerateTexture();

  // Create a CompositeCameras object to render once the window is open and the context is active.
  std::shared_ptr<asdp::render::PoseAdjuster> poseAdjuster = std::make_shared<asdp::render::PoseAdjuster>();
  asdp::render::CompositeCameras composite(cameras, toneMapTexture, poseAdjuster, asdp::Time());

  // Loop until the user closes the window.
  std::cout << "You should see a row of three distorted dark boxes horizontally across" << std::endl;
  std::cout << "the center of the view, the first and third brighter on the left and the" << std::endl;
  std::cout << "second brighter on the right." << std::endl;
  std::cout << "Above should be brighter extensions and below should be darker ones." << std::endl;
  std::cout << "The extensions meet at dark and then bright boundaries from left to right." << std::endl;
  std::cout << "" << std::endl;
  std::cout << "Close the window to exit." << std::endl;
  auto start = std::chrono::steady_clock::now();
  while (!glfwWindowShouldClose(window)) {

    // Render here
    composite.Render(asdp::Time(), views);

    // Swap front and back buffers
    glfwSwapBuffers(window);

    // Poll for and process events
    glfwPollEvents();
  }

  // Clean up resources and exit
  glfwTerminate();
  return 0;
}
