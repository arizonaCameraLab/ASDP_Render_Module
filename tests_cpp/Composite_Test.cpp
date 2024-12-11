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

  // Create a CompositeCube object to render once the window is open and the context is active.
  asdp::render::CompositeCube composite(10);

  // Loop until the user closes the window, rotating the view each frame around the +Y axis.
  std::cout << "You should see a square of varying-brightness squares in the window." << std::endl;
  std::cout << "It should be rotating at 15 degrees per second around the Y axes." << std::endl;
  std::cout << "This will make the far wall of the cube move towards the right." << std::endl;
  std::cout << "There is a much slower rotation about the X axis first that will make" << std::endl;
  std::cout << "the green wall rotate downward and the yellow up." << std::endl;
  std::cout << "The center of location is closer to the magenta wall than the red wall." << std::endl;
  std::cout << "Close the window to exit." << std::endl;
  auto start = std::chrono::steady_clock::now();
  while (!glfwWindowShouldClose(window)) {
    // Set the viewpoint here
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
    double angle = -degreesPerSecond * (elapsed / 1000.0);

    // Offset the viewpoint center and rotate around the Z axis so we can verify correct behavior.
    // Also rotate slowly around the X axis.
    views[0].nearClip = 0.1f;
    views[0].farClip = 1000.0f;
    views[0].viewpoint[0] = -5;
    float dx = glm::radians(angle/100); // Rotate 45 degrees around the X axis
    float dz = glm::radians(angle); // Rotate 30 degrees around the Z axis
    glm::quat rotationX = glm::angleAxis(dx, glm::vec3(1.0f, 0.0f, 0.0f));
    glm::quat rotationZ = glm::angleAxis(dz, glm::vec3(0.0f, 0.0f, 1.0f));
    glm::quat combinedRotation = rotationZ * rotationX;
    views[0].orientation[0] = combinedRotation.w;
    views[0].orientation[1] = combinedRotation.x;
    views[0].orientation[2] = combinedRotation.y;
    views[0].orientation[3] = combinedRotation.z;

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
