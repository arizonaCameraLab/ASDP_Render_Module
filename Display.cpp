/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

#ifdef WIN32
#define _USE_MATH_DEFINES
#endif
#include <cmath>
#include <iostream>
#include <thread>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include "Display.h"

using namespace asdp::render;

// Static class member to ensure that GLFW is initialized and terminated when this library is
// loaded and unloaded.
class GLFWInitializer {
  public:
  GLFWInitializer() {
    if (!glfwInit()) {
      std::cerr << "asdp::Render::Display submodule: Failed to initialize GLFW" << std::endl;
    }
  }
  ~GLFWInitializer() {
    glfwTerminate();
  }
};
static GLFWInitializer initGLFW;

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
  float m_horizontalFOVDegrees{ 90.0f };

  /// Window that will be used to display the view.
  GLFWwindow *m_window = nullptr;

  /// Views to be rendered.
  std::vector<asdp::render::ViewRenderInfo> m_views;
};

DisplayWindow::DisplayWindow(std::string windowName, std::shared_ptr<Composite> composite,
    std::shared_ptr<CoreClient> client, uint8_t triggerID, uint32_t triggerAheadMicroseconds,
    int desiredWidth, int desiredHeight, float horizontalFOVDegrees,
    std::string joystick, DisplayWindow* sharedWindow,
    bool fullScreen, int desiredDisplay, bool hidden)
  : Display(composite, client, triggerID, triggerAheadMicroseconds)
  , m_impl(new DisplayWindowImpl)
{
  // Store info from the constructor.
  m_impl->m_horizontalFOVDegrees = horizontalFOVDegrees;
 

  // Construct a single view to be used.  We base is on the requested window size and we compute a
  // field of view that is 40 degrees total horizontal and the correct aspect ratio vertical.
  ViewRenderInfo view;
  SetViewportSizeAndFOVs(view, desiredWidth, desiredHeight);
  m_impl->m_views.push_back(view);

  // Detach our context so that it can be attached in the rendering thread.
  glfwMakeContextCurrent(nullptr);

  // Start the rendering thread.
  m_displayThread = std::thread(&DisplayWindow::DisplayThread, this, windowName,
    desiredWidth, desiredHeight, horizontalFOVDegrees,
    joystick, sharedWindow, fullScreen, desiredDisplay, hidden);
}

DisplayWindow::~DisplayWindow()
{
  // Make sure we're done with our rendering state and then clean up.
  Quit();
  m_impl.reset();
}

void DisplayWindow::SetViewportSizeAndFOVs(ViewRenderInfo& viewInfo, int width, int height)
{
  if (m_impl == nullptr) {
    return;
  }
  if ((width == 0) || (height == 0)) {
    glfwGetWindowSize(m_impl->m_window, &width, &height);
    viewInfo.width = width;
    viewInfo.height = height;
  }
  viewInfo.leftHalfFOV = -m_impl->m_horizontalFOVDegrees / 2.0f;
  viewInfo.rightHalfFOV = m_impl->m_horizontalFOVDegrees / 2.0f;

  // The vertical field of view is based on the aspect ratio of the window.  But the aspect ratio
  // is the in-plane width divided by the in-plane height.  The horizontal and vertical fields of view
  // are based on the tangents.
  double aspectRatio = static_cast<double>(viewInfo.height) / static_cast<double>(viewInfo.width);
  double halfWidth = tan( (m_impl->m_horizontalFOVDegrees / 2.0) * (M_PI / 180.0));
  double halfHeight = halfWidth * aspectRatio;
  double halfAngle = atan(halfHeight) * (180.0 / M_PI);
  viewInfo.bottomHalfFOV = -halfAngle;
  viewInfo.topHalfFOV = halfAngle;
}

bool DisplayWindow::MakeContextCurrent()
{
  if (m_impl == nullptr) {
    return false;
  }

  glfwMakeContextCurrent(m_impl->m_window);

  return true;
}

void DisplayWindow::DisplayThread(std::string windowName,
  int desiredWidth, int desiredHeight, float horizontalFOVDegrees,
  std::string joystick, DisplayWindow* sharedWindow,
  bool fullScreen, int desiredDisplay, bool hidden)
{
  // Create a windowed mode window and its OpenGL context.
  // This must be done in the same thread that will do the rendering so that the window events will
  // be handles properly on all architectures.
  GLFWmonitor* fullScreenMonitor = nullptr;
  if (fullScreen) {
    int count;
    GLFWmonitor** monitors = glfwGetMonitors(&count);
    if ((count == 0) || !monitors) {
      m_status = "No monitors for fullscreen";
      return;
    }
    if (desiredDisplay >= count) {
      m_status = "Invalid monitor requested (index larger than available monitors)";
      return;
    }
    fullScreenMonitor = monitors[desiredDisplay];
  }
  GLFWwindow* windowToShare = nullptr;
  if (sharedWindow) {
    windowToShare = sharedWindow->m_impl->m_window;
  }
  glfwWindowHint(GLFW_VISIBLE, !hidden);
  m_impl->m_window = glfwCreateWindow(desiredWidth, desiredHeight, windowName.c_str(), fullScreenMonitor,
    windowToShare);
  if (!m_impl->m_window) {
    m_status = "Failed to create GLFW window";
    return;
  }

  // Make the window's context current
  MakeContextCurrent();

  // Initialize GLEW in our context. It is okay to initialize it more than once.
  glewExperimental = true;
  if (glewInit() != GLEW_OK) {
    m_status = "Failed to initialize GLEW";
    return;
  }
  // Clear any GL error that Glew caused.  Apparently on Non-Windows
  // platforms, this can cause a spurious error 1280.
  glGetError();

  // Open the joystick if there is one.
  /// @todo

  // Add hooks for keyboard and mouse input.
  /// @todo

  // Make the window's context current
  if (!MakeContextCurrent()) {
    std::cerr << "DisplayWindow::DisplayThread(): could not make context current" << std::endl;
    return;
  }

  // Loop until the display is done.
  bool windowClosed = false;
  while (!m_done) {
    // Quit when our window closes.
    if (glfwWindowShouldClose(m_impl->m_window)) {
      m_composite.reset();
      m_status = "Done";
      windowClosed = true;
      break;
    }

    const char* description;
    int code = glfwGetError(&description);
    if (code != GLFW_NO_ERROR) {
      std::cerr << "GLFW error: " << code << ": " << description << std::endl;
    }

    // Process keyboard/mouse/joystick input events and update the viewpoint
    /// @todo

    // Handle any window resizing
    SetViewportSizeAndFOVs(m_impl->m_views[0]);

    // Render here
    m_composite->Render(asdp::Time(), m_impl->m_views);

    // Swap front and back buffers
    glfwSwapBuffers(m_impl->m_window);

    // Poll for and process events
    glfwPollEvents();
  }

  // Close the window if needed
  if (!windowClosed) { glfwDestroyWindow(m_impl->m_window); }
}