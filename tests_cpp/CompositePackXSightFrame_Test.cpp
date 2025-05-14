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
#include <RangeEstimator.h>
#include <ASDP_Core_API.h>
#include <GLFW/glfw3.h>

/// @brief Make an image that is mostly black but has vertical lines that vary from dark at
/// the top to bright at the bottom.  The lines will be red, green, blue, and white and they
/// will be separated from each other by twenty pixels.  The line widths will be 1 pixel for the
/// first repetition, on an odd and even column, and 2 pixels for the second repetition --
/// again on an odd and even column.
/// @param width Width of the image.
/// @param height Height of the image.
/// @param minVal Minimum value of the image (at the top of the texture).
/// @param maxVal Maximum value of the image (at the bottom of the texture).
/// @return An OpenGL texture ID.
GLuint MakeTexture(int width, int height, uint16_t minVal, uint16_t maxVal)
{
  std::vector<int> columns = { 20, 40, 61, 81, 100,101, 121,122 };
  std::vector< std::array<double, 3> > colors = {
    { 1.0, 0, 0 },   // Red
    { 0, 1.0, 0 },   // Green
    { 0, 0, 1.0 },   // Blue
    { 1.0, 1.0, 1.0 }, // White
    { 1.0, 1.0, 1.0 }, // White
    { 1.0, 1.0, 1.0 }, // White
    { 1.0, 1.0, 1.0 }, // White
    { 1.0, 1.0, 1.0 } // White
  };

  std::vector<uint16_t> image(width * height * 3);
  float range = static_cast<float>(maxVal - minVal);
  for (int j = 0; j < height; j++) {
    float normJ = static_cast<float>(j) / static_cast<float>(height - 1);
    for (int i = 0; i < width; i++) {
      for (int c = 0; c < 3; c++) {
        int index = 3 * (i + j * width) + c;
        image[index] = 0;
        // If the column is in the list of columns, set the color to the scaled color for that column.
        for (size_t line = 0; line < columns.size(); line++) {
          if (i == columns[line]) {
            image[index] = static_cast<uint16_t>(minVal + normJ * range * colors[line][c]);
          }
        }
      }
    }
  }

  unsigned int texture;
  glGenTextures(1, &texture);
  glBindTexture(GL_TEXTURE_2D, texture);
  // Set the texture wrapping parameters
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  // Set texture filtering parameters
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

   // Load image into texture
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16, width, height, 0, GL_RGB, GL_UNSIGNED_SHORT, image.data());
  glBindTexture(GL_TEXTURE_2D, 0);

  return texture;
}

int main()
{
  int windowWidth = 640;
  int width = 2 * windowWidth;
  int height = 1024;

  // Initialize the library
  if (!glfwInit()) {
    std::cerr << "Failed to initialize GLFW\n";
    return -1;
  }

  // Create a windowed mode window and its OpenGL context
  GLFWwindow* window = glfwCreateWindow(windowWidth, height, "CompositePackXSightFrame_Test", NULL, NULL);
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

  // Create the texture to render to.
  GLuint texture = MakeTexture(width, height, 0, 65535);

  // Create a CompositePackXSightFrame object to render once the window is open and the context is active.
  asdp::render::CompositePackXSightFrame composite(texture, windowWidth);

  // Loop until the user closes the window.
  std::cout << "You should see @todo" << std::endl;
  std::cout << "the center of the view, the first and third brighter on the left and the" << std::endl;
  std::cout << "second brighter on the right." << std::endl;
  std::cout << "Above should be brighter extensions and below should be darker ones." << std::endl;
  std::cout << "The extensions meet at dark and then bright boundaries from left to right." << std::endl;
  std::cout << "A smaller box should be in the center of the view, black at bottom to white at top." << std::endl;
  std::cout << "" << std::endl;
  std::cout << "Press the space bar to toggle between the whole color range and only the" << std::endl;
  std::cout << "range 0.25 through 0.75." << std::endl;
  std::cout << "" << std::endl;
  std::cout << "Close the window to exit." << std::endl;
  auto start = std::chrono::steady_clock::now();
  bool rangeZoomed = false;
  bool spacePressed = false;
  while (!glfwWindowShouldClose(window)) {

    // Render here
    std::vector<asdp::render::ViewRenderInfo> views;
    asdp::render::ViewRenderInfo view;
    view.width = windowWidth;
    view.height = height;
    views.push_back(view);
    composite.Render(asdp::Time(), views);

    // Swap front and back buffers
    glfwSwapBuffers(window);

    // Poll for and process events
    glfwPollEvents();

    // When the space key is pressed, toggle between a range that covers the whole color range and one that
    // covers only the quarter of it above the middle.
    if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) {
      if (!spacePressed) {
        rangeZoomed = !rangeZoomed;
        std::cout << "Range " << (rangeZoomed ? "" : "not ") << "zoomed" << std::endl;
        spacePressed = true;
      }
    } else {
      spacePressed = false;
    }
  }

  // Clean up resources and exit
  glfwTerminate();
  return 0;
}
