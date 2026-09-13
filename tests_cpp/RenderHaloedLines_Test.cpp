/*
 * Copyright (C) 2025: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include <iostream>
#include <vector>
#include <chrono>
#include <memory>
#include <WindowCreation.h>
#include <RenderHaloedLines.h>

int main()
{
  int width = 640;
  int height = 640;

  // Create a windowed mode window and its OpenGL context
  std::shared_ptr<GLFWwindow> window;
  std::string error = asdp::render::CreateWindowOrContext(window, width, height, "RenderHaloedLines_Test");
  if (!error.empty()) {
    std::cerr << "Failed to create GLFW window: " << error << std::endl;
    return -1;
  }

  // Make the window's context current
  glfwMakeContextCurrent(window.get());

  try {
    // Create a RenderHaloedLines object.
    asdp::render::RenderHaloedLines renderLines;

    std::vector< std::array< std::array<float, 2>, 2> > single = {
      { { { -0.5f, 0.8f }, { 0.5f, 0.8f } } }
    };
    std::array<float, 3> green = { 0.0f, 1.0f, 0.0f };
    std::array<float, 3> red = { 1.0f, 0.0f, 0.0f };
    std::array<float, 3> black = { 0.0f, 0.0f, 0.0f };

    std::vector< std::array< std::array<float, 2>, 2> > square = {
      { { { -0.5f, -0.5f }, { 0.5f, -0.5f } } },
      { { { 0.5f, -0.5f }, { 0.5f, 0.5f } } },
      { { { 0.5f, 0.5f }, { -0.5f, 0.5f } } },
      { { { -0.5f, 0.5f }, { -0.5f, -0.5f } } }
    };

    // Loop until the user closes the window.
    std::cout << "You should see a thin translucent red line and a green square with black surround." << std::endl;
    std::cout << "Close the window to exit." << std::endl;
    while (!glfwWindowShouldClose(window.get())) {

      // Render here
      glClearColor(1.0f, 1.0f, 0.0f, 1.0f);
      glClear(GL_COLOR_BUFFER_BIT);

      renderLines.Draw(single, 1, 3, red, black, 0.5);
      renderLines.Draw(square, 2, 5, green, black);

      // Swap front and back buffers
      glfwSwapBuffers(window.get());

      // Poll for and process events
      glfwPollEvents();

      // Handle window resize, including adjusting the viewport.
      int newWidth, newHeight;
      glfwGetFramebufferSize(window.get(), &newWidth, &newHeight);
      if (newWidth != width || newHeight != height) {
        width = newWidth;
        height = newHeight;
        glViewport(0, 0, width, height);
      }
    }
  } catch (const std::exception& e) {
    std::cerr << "Failed to run the tests: " << e.what() << std::endl;
    return -1;
  }

  // Clean up resources and exit
  window.reset();
  return 0;
}
