/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include <iostream>
#include <vector>
#include <Composite.h>
#include <ASDP_Core_API.h>
#include <GLFW/glfw3.h>

int main()
{
  int width = 640;
  int height = 640;

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

  // Create a CompositeCube object to render once the window is open and the context is active.
  asdp::render::CompositeCube composite(0.5);

  // Loop until the user closes the window
  std::cout << "You should see a square of varying-brightness squares in the window." << std::endl;
  std::cout << "Close the window to exit." << std::endl;
  while (!glfwWindowShouldClose(window)) {
    // Render here
    composite.Render(asdp::Time(), views);

    // Swap front and back buffers
    glfwSwapBuffers(window);

    // Poll for and process events
    glfwPollEvents();
  }

  glfwTerminate();

  // Clean up resources and exit
  return 0;
}
