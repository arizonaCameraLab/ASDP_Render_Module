/*
 * Copyright (C) 2025: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include <iostream>
#include <vector>
#include <chrono>
#include <memory>
#include <WindowCreation.h>
#include <RenderText.h>

int main()
{
  int width = 640;
  int height = 640;

  // Create a windowed mode window and its OpenGL context
  std::shared_ptr<GLFWwindow> window;
  std::string error = asdp::render::CreateWindowOrContext(window, width, height, "RenderText_Test");
  if (!error.empty()) {
    std::cerr << "Failed to create GLFW window: " << error << std::endl;
    return -1;
  }

  // Make the window's context current
  glfwMakeContextCurrent(window.get());

  try {
    // Create a RenderText object.
    asdp::render::RenderText renderText(width, height);

    // Loop until the user closes the window.
    std::cout << "You should see the phrase 'Hello, World' written in white in a yellow image." << std::endl;
    std::cout << "There should be a second line indented under it." << std::endl;
    std::cout << "The upper-left corner of the text should be at the center of the image." << std::endl;
    std::cout << "There should be a gray rectangle behind the text so that it is visible." << std::endl;
    std::cout << "There should be a second instance of the word 'Translucent' above it, half transparent." << std::endl;
    std::cout << "Close the window to exit." << std::endl;
    while (!glfwWindowShouldClose(window.get())) {

      // Render here
      glClearColor(1.0f, 1.0f, 0.0f, 1.0f);
      glClear(GL_COLOR_BUFFER_BIT);

      renderText.Draw("Hello, World\n  Second line indented", 0.0f, 0.0f, 1.0f, 1.0f, 1.0f);
      renderText.Draw("Translucent", 0.0f, 0.5f, 1.0f, 1.0f, 1.0f, 0.5f);

      // Swap front and back buffers
      glfwSwapBuffers(window.get());

      // Poll for and process events
      glfwPollEvents();

      // Handle window resize, including adjusting the viewport and updating RenderText.
      int newWidth, newHeight;
      glfwGetFramebufferSize(window.get(), &newWidth, &newHeight);
      if (newWidth != width || newHeight != height) {
        width = newWidth;
        height = newHeight;
        renderText.SetWindowSize(width, height);
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
