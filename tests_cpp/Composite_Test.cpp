/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include <iostream>
#include <vector>
#include <chrono>
#include <Composite.h>
#include <ASDP_Core_API.h>
#include <gfxwrapper_opengl.h>

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

  // Create a windowed mode window and its OpenGL context
  WINDOW_TITLE = "Composite_Test";
  APPLICATION_NAME = "Composite_Test";
  ksDriverInstance driverInstance{};
  ksGpuQueueInfo queueInfo{};
  ksGpuSurfaceColorFormat colorFormat{ KS_GPU_SURFACE_COLOR_FORMAT_B8G8R8A8 };
  ksGpuSurfaceDepthFormat depthFormat{ KS_GPU_SURFACE_DEPTH_FORMAT_D24 };
  ksGpuSampleCount sampleCount{ KS_GPU_SAMPLE_COUNT_1 };
  ksGpuWindow m_window{};
  if (!ksGpuWindow_Create(&m_window, &driverInstance, &queueInfo, 0, colorFormat, depthFormat,
    sampleCount, width, height, false)) {
    std::cerr << "Failed to open window\n";
    return -1;
  }

  // Make the window's context current
  ksGpuContext_SetCurrent(&m_window.context);

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
  while (KS_GPU_WINDOW_EVENT_EXIT != ksGpuWindow_ProcessEvents(&m_window)) {
    // Set the viewpoint here
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
    double angle = degreesPerSecond * (elapsed / 1000.0);

    // Offset the viewpoint center and rotate around the Y axis so we can verify correct behavior.
    // Also rotate slowly around the X axis.
    views[0].viewpoint[0] = -5;
    views[0].orientation[0] = angle/100;
    views[0].orientation[1] = angle;

    // Render here
    composite.Render(asdp::Time(), views);

    // Swap front and back buffers
    ksGpuWindow_SwapBuffers(&m_window);
  }

  // Clean up resources and exit
  return 0;
}
