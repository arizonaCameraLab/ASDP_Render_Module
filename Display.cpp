/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

#ifdef WIN32
#define _USE_MATH_DEFINES
#endif
#include <cmath>
#include <iostream>
#include <thread>
#include <gfxwrapper_opengl.h>
#include "Display.h"

using namespace asdp::render;

Display::Display(std::shared_ptr<Composite> composite,
  std::shared_ptr<CoreClient> client, uint8_t triggerID, uint32_t triggerAheadMicroseconds)
  : m_composite(composite)
  , m_client(client)
  , m_triggerID(triggerID)
  , m_offsetMicroseconds(triggerAheadMicroseconds)
  , m_done(false)
{
  if (m_client) {
    Status status = m_client->GetTimer(m_timer);
    if (status != OKAY) {
      m_timer.reset();
    }
  }
}

Display::~Display()
{
  // Call the Quit() virtual function to stop all threads and clean up resources.
  Quit();
}

bool Display::Quit()
{
  // Stop the display thread, if it is running.
  m_done = true;
  if (m_displayThread.joinable()) {
    m_displayThread.join();
  }
  m_status = "Done";

  // Clean up all resources, including those kept in shared pointers.
  m_timer.reset();
  m_composite.reset();
  m_client.reset();

  return true;
}

std::string Display::GetStatus() const
{
  return m_status;
}

bool Display::TriggerCameras(std::chrono::steady_clock::time_point when)
{
  if ((m_client == nullptr) || (m_timer == nullptr)) {
    // No client or timer, so we can't trigger the cameras.
    return true;
  }

  // Determine the time to trigger the cameras by subtracting the microseconds
  // offset from the time to trigger the cameras and then converting to Core time.
  auto sysTime = std::chrono::time_point_cast<std::chrono::microseconds>(when -
    std::chrono::microseconds(m_offsetMicroseconds));
  Time coreTime;
  Status status = m_timer->GetCoreTime(coreTime, sysTime);
  if (status != OKAY) {
    return false;
  }

  // Send a software-trigger command to the client.
  CommandPacketSoftwareTrigger packet(m_triggerID, coreTime);
  if (packet.GetConstructorStatus() != OKAY) {
    return false;
  }
  status = m_client->SendCommandPacket(packet);
  if (status != OKAY) {
    return false;
  }

  // It worked
  return true;
}

//==============================================================================
// Structures and methods for DisplayWindow class.

class asdp::render::DisplayWindow::DisplayWindowImpl {
public:
  /// Horizontal field of view in degrees.
  float m_horizontalFOVDegrees{ 40.0f };

  /// Window that will be used to display the view.
  ksGpuWindow m_window{};

  /// Views to be rendered.
  std::vector<asdp::render::ViewRenderInfo> m_views;

  /// Our main context (index 0) and shared contexts that we have produced
  std::vector<ksGpuContext> m_contexts;
};

DisplayWindow::DisplayWindow(std::shared_ptr<Composite> composite,
    std::shared_ptr<CoreClient> client, uint8_t triggerID, uint32_t triggerAheadMicroseconds,
    float horizontalFOVDegrees,
    std::string joystick, int desiredWidth, int desiredHeight, bool fullScreen, int whichDisplay)
  : Display(composite, client, triggerID, triggerAheadMicroseconds)
  , m_impl(new DisplayWindowImpl)
{
  // Store info from the constructor.
  m_impl->m_horizontalFOVDegrees = horizontalFOVDegrees;
 
  // Open the joystick if there is one.
  /// @todo

  // Add hooks for keyboard and mouse input.
  /// @todo

  // Create a windowed mode window and its OpenGL context
  ksDriverInstance driverInstance{};
  ksGpuQueueInfo queueInfo{};
  ksGpuSurfaceColorFormat colorFormat{ KS_GPU_SURFACE_COLOR_FORMAT_B8G8R8A8 };
  ksGpuSurfaceDepthFormat depthFormat{ KS_GPU_SURFACE_DEPTH_FORMAT_D24 };
  ksGpuSampleCount sampleCount{ KS_GPU_SAMPLE_COUNT_1 };
  if (!ksGpuWindow_Create(&m_impl->m_window, &driverInstance, &queueInfo, 0, colorFormat, depthFormat,
    sampleCount, desiredWidth, desiredHeight, fullScreen)) {
    m_status = "Failed to open window";
    m_impl.reset();
    return;
  }

  // Construct a single view to be used.  We base is on the actual window size and we compute a
  // field of view that is 40 degrees total horizontal and the correct aspect ratio vertical.
  ViewRenderInfo view;
  SetViewportSizeAndFOVs(view);
  m_impl->m_views.push_back(view);

  // Start the rendering thread.
  m_displayThread = std::thread(&DisplayWindow::DisplayThread, this);

  // Store our context.
  m_impl->m_contexts.push_back(m_impl->m_window.context);
}

DisplayWindow::~DisplayWindow()
{
  // Make sure we're done with our rendering state and then clean up.
  Quit();
  m_impl.reset();
}

void DisplayWindow::SetViewportSizeAndFOVs(ViewRenderInfo& viewInfo)
{
  if (m_impl == nullptr) {
    return;
  }
  viewInfo.width = m_impl->m_window.windowWidth;
  viewInfo.height = m_impl->m_window.windowHeight;
  viewInfo.leftHalfFOV = -m_impl->m_horizontalFOVDegrees / 2.0f;
  viewInfo.rightHalfFOV = m_impl->m_horizontalFOVDegrees / 2.0f;

  // The vertical field of view is based on the aspect ratio of the window.  But the aspect ratio
  // is the in-plane width divided by the in-plane height.  The horizontal and vertical fields of view
  // are based on the tangents.
  double aspectRatio = static_cast<double>(viewInfo.height) / static_cast<double>(viewInfo.width);
  double halfWidth = tan( (m_impl->m_horizontalFOVDegrees / 2.0) * (M_PI / 180.0));
  double halfHeight = halfWidth * aspectRatio;
  double halfAngle = 2.0 * atan(halfHeight) * (180.0 / M_PI);
  viewInfo.bottomHalfFOV = -halfAngle;
  viewInfo.topHalfFOV = halfAngle;
}

bool DisplayWindow::MakeContextCurrent(unsigned contextID)
{
  if (m_impl == nullptr) {
    return false;
  }

  // Make the specified context current
  if (contextID >= m_impl->m_contexts.size()) {
    return false;
  }
  ksGpuContext_SetCurrent(&m_impl->m_contexts[contextID]);

  return true;
}

unsigned DisplayWindow::ConstructSharedContext()
{
  if (m_impl == nullptr) {
    return 0;
  }
  
  ksGpuContext sharedContext{};
  if (!ksGpuContext_CreateShared(&sharedContext, &m_impl->m_contexts[0], 0)) {
    return 0;
  }
  m_impl->m_contexts.push_back(sharedContext);

  return m_impl->m_contexts.size() - 1;
}

void DisplayWindow::DisplayThread()
{
  // Make the window's context current
  MakeContextCurrent(0);

  // Loop until the display is done.
  bool windowClosed = false;
  while (!m_done) {
    // Quit when our window closes.
    if (KS_GPU_WINDOW_EVENT_EXIT != ksGpuWindow_ProcessEvents(&m_impl->m_window)) {
      m_status = "Done";
      windowClosed = true;
      break;
    }

    // Process keyboard/mouse/joystick input events and update the viewpoint
    /// @todo

    // Handle any window resizing
    SetViewportSizeAndFOVs(m_impl->m_views[0]);

    // Render here
    m_composite->Render(asdp::Time(), m_impl->m_views);

    // Swap front and back buffers
    ksGpuWindow_SwapBuffers(&m_impl->m_window);
  }

  // Close the window if needed
  if (!windowClosed) { ksGpuWindow_Destroy(&m_impl->m_window); }
}