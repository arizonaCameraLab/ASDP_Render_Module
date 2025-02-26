/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include <iostream>
#include <vector>
#include <chrono>
#include <Composite.h>
#include <ASDP_Core_API.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>

int main()
{
  int width = 640;
  int height = 640;
  double degreesPerSecond = 15.0;

  asdp::render::ViewRenderInfo viewRenderInfo;
  viewRenderInfo.width = width;
  viewRenderInfo.height = height;
  std::vector<asdp::render::ViewRenderInfo> views;
  views.push_back(viewRenderInfo);

  // Initialize the library
  if (!glfwInit()) {
    std::cerr << "Failed to initialize GLFW\n";
    return -1;
  }

  // Create a windowed mode window and its OpenGL context
  GLFWwindow* window = glfwCreateWindow(width, height, "Composite_Test", NULL, NULL);
  if (!window) {
    std::cerr << "Failed to create GLFW window\n";
    glfwTerminate();
    return -1;
  }

  // Make the window's context current
  glfwMakeContextCurrent(window);

  // Determine the coordinates of the ends of a line that starts in the upper-left pixel and goes
  // to the right for a total of 10 pixels.
  unsigned numPixels = 100;
  unsigned numValues = numPixels * 3;
  GLfloat x0, y0, x1, y1;
  asdp::render::CompositeLineRawData::ComputeVertexCoordinates(width, height, 0, 0, numPixels-1, 0,
    x0, y0, x1, y1);
  std::cout << "x0: " << x0 << " y0: " << y0 << " x1: " << x1 << " y1: " << y1 << std::endl;

  // Create a vector of RGB values to render on the line.
  std::vector<uint8_t> valuesRGB(numValues);
  for (unsigned i = 0; i < numValues; i++) {
    valuesRGB[i] = i * 5;
  }

  // Create a CompositeCube object to render once the window is open and the context is active.
  asdp::render::CompositeLineRawData composite(x0, y0, x1, y1, valuesRGB);

  // Image vector to read back into.
  std::vector<uint8_t> image(width * height * 3);

  // Loop until the user closes the window, rotating the view each frame around the +Y axis.
  std::cout << "You should see ramps of increasing-brightness colors in the upper left." << std::endl;
  std::cout << "If no error messages are printed, the read-back values match the expected values." << std::endl;
  std::cout << "Close the window to exit." << std::endl;
  auto start = std::chrono::steady_clock::now();
  while (!glfwWindowShouldClose(window)) {
    views[0].nearClip = 0.1f;
    views[0].farClip = 1000.0f;
    views[0].viewpoint[0] = -5;
    views[0].orientation[0] = 1.0f;
    views[0].orientation[1] = 0;
    views[0].orientation[2] = 0;
    views[0].orientation[3] = 0;

    // Update the composite data
    composite.UpdateValues(valuesRGB);

    // Render here
    composite.Render(asdp::Time(), views);

    // Ensure rendering is complete.
    glFinish();

    // Read back the rendered image
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, image.data());

    // Verify that the first part of the image matches our pixel values.
    // We're reading back lines from the bottom, so we need to skip to the upper line.
    size_t startByte = width * (height - 1) * 3;
    bool success = true;
    for (unsigned i = 0; i < numValues; i++) {
      if (image[startByte + i] != valuesRGB[i]) {
        std::cout << "Error: Mismatch at value " << i << ": expected " << (int)valuesRGB[i] << " got " << (int)image[startByte + i] << std::endl;
        success = false;
        break;
      }
    }
    if (!success) {
      std::cerr << "Failed to match the rendered image to the expected values." << std::endl;
      //return 1;
    }

    // Swap front and back buffers
    glfwSwapBuffers(window);

    // Poll for and process events
    glfwPollEvents();
  }

  // Clean up resources and exit
  glfwTerminate();
  return 0;
}
