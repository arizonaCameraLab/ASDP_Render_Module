/*
 * Copyright (C) 2025: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include <iostream>
#include <vector>
#include <chrono>
#include <memory>
#include <RenderText.h>
#include <GLFW/glfw3.h>

int main()
{
  int width = 640;
  int height = 640;

  // Initialize the library
  if (!glfwInit()) {
    std::cerr << "Failed to initialize GLFW\n";
    return -1;
  }

  // Create a windowed mode window and its OpenGL context
  GLFWwindow* window = glfwCreateWindow(width, height, "RenderText_Test", NULL, NULL);
  if (!window) {
    std::cerr << "Failed to create GLFW window\n";
    glfwTerminate();
    return -1;
  }

  // Make the window's context current
  glfwMakeContextCurrent(window);

  try {
    // Create a RenderText object.
    asdp::render::RenderText renderText(width, height);

    // Loop until the user closes the window.
    std::cout << "You should see the phrase 'Hello, World' written in white in a yellow image." << std::endl;
    std::cout << "The upper-left corner of the text should be at the center of the image." << std::endl;
    std::cout << "There should be a gray rectangle behind the text so that it is visible." << std::endl;
    std::cout << "Close the window to exit." << std::endl;
    while (!glfwWindowShouldClose(window)) {

      // Render here
      glClearColor(1.0f, 1.0f, 0.0f, 1.0f);
      glClear(GL_COLOR_BUFFER_BIT);
      renderText.Draw("Hello, World", 0.0f, 0.0f, 1.0f, 1.0f, 1.0f);

      // Swap front and back buffers
      glfwSwapBuffers(window);

      // Poll for and process events
      glfwPollEvents();
    }
  } catch (const std::exception& e) {
    std::cerr << "Failed to run the tests: " << e.what() << std::endl;
    glfwTerminate();
    return -1;
  }

  // Clean up resources and exit
  glfwTerminate();
  return 0;
}
