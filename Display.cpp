/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

#ifdef WIN32
#define _USE_MATH_DEFINES
#endif
#include <cmath>
#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
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

std::mutex Display::m_windowMutex;

//==============================================================================
// Structures and methods for Display class.

class asdp::render::Display::DisplayImpl {
public:
  /// Window that will be used to display the view.
  GLFWwindow* m_window = nullptr;

  //===========================
  // Machinery required for borrowing and returning the context from DisplayThread.
  // We need to be sure that the context is available before borrowing it, so we have
  // and atomic to track that.  We need to ensure that we don't try to use the context
  // while it is being borrowed, so we have a mutex to protect that.
  // Some operating systems (Windows, for example), require the OpenGL context to be
  // shared to be active on the thread that is creating the new window.

  // NOTE: All derived classes must set m_contextAvailable to true after the context is created
  // and read to be borrowed.
  std::atomic_bool m_contextAvailable{ false };

  // NOTE: All derived classes must lock m_contextMutex when they are using the context and must
  // periodically unlock it so that it can be borrowed to create another context that shares objects
  // with this one.  On Linux, this must be an extended period of time (not just a few instructions)
  // so that another thread can get a chance to lock the mutex.
  std::mutex m_contextMutex;
};


Display::Display(std::shared_ptr<Composite> composite,
  std::shared_ptr<CoreClient> client, uint8_t triggerID, uint32_t triggerAheadMicroseconds)
  : m_composite(composite)
  , m_client(client)
  , m_triggerID(triggerID)
  , m_offsetMicroseconds(triggerAheadMicroseconds)
  , m_done(false)
  , m_impl(new DisplayImpl)
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
  m_impl.reset();
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

bool Display::BorrowContext()
{
  if (m_impl == nullptr) {
    return false;
  }

  // Wait until the context is available.
  while (!m_impl->m_contextAvailable) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // Grab the context mutex.
  m_impl->m_contextMutex.lock();

  // Make the context current on the calling thread.
  glfwMakeContextCurrent(m_impl->m_window);

  return true;
}

bool Display::ReturnContext()
{
  if (m_impl == nullptr) {
    return false;
  }

  // Release the current context.
  glfwMakeContextCurrent(nullptr);

  // Release the context mutex.
  m_impl->m_contextMutex.unlock();

  return true;
}

//==============================================================================
// Structures and methods for DisplayWindow class.

class asdp::render::DisplayWindow::DisplayWindowImpl {
public:
  /// Horizontal field of view in degrees.
  float m_horizontalFOVDegrees {90.0f};

  /// Views to be rendered.
  std::vector<asdp::render::ViewRenderInfo> m_views;

  /// Last time we checked the keyboard, used to control motion rate.
  std::chrono::steady_clock::time_point m_lastKeyboardCheck;

  /// Time point to start rendering the next frame.
  std::chrono::steady_clock::time_point m_nextFrameTime;
};

DisplayWindow::DisplayWindow(std::string windowName, std::shared_ptr<Composite> composite,
    std::shared_ptr<CoreClient> client, uint8_t triggerID, uint32_t triggerAheadMicroseconds,
    float fps, uint32_t renderAheadMicroseconds,
    int desiredWidth, int desiredHeight, float horizontalFOVDegrees,
    std::string joystick, Display* sharedWindow,
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

  // Start the rendering thread.
  m_displayThread = std::thread(&DisplayWindow::DisplayThread, this, windowName,
    fps, renderAheadMicroseconds,
    desiredWidth, desiredHeight, horizontalFOVDegrees,
    joystick, sharedWindow, fullScreen, desiredDisplay, hidden);

  // Wait until either the context is ready or there has been a failure so that the
  // constructor does not return before the rendering thread is ready.
  while (!Display::m_impl->m_contextAvailable && (m_status == "")) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
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
    glfwGetWindowSize(Display::m_impl->m_window, &width, &height);
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

void DisplayWindow::DisplayThread(std::string windowName,
  float fps, uint32_t renderAheadMicroseconds,
  int desiredWidth, int desiredHeight, float horizontalFOVDegrees,
  std::string joystick, Display* sharedWindow,
  bool fullScreen, int desiredDisplay, bool hidden)
{
  // Hold the window mutex so that only one window can be created at a time.
  {
    std::lock_guard<std::mutex> windowLock(m_windowMutex);

    // Set the window visibility.
    glfwWindowHint(GLFW_VISIBLE, !hidden);

    // Create a windowed mode window and its OpenGL context.
    // This must be done in the same thread that will do the rendering so that the window events will
    // be handled properly on all architectures.
    // We must make the OpenGL context of the window we want to share current on this thread
    // if we are sharing it by borrowing it and then returning it once the window is open because
    // Windows requires it to be current.
    GLFWwindow* windowToShare = nullptr;
    if (sharedWindow) {
      windowToShare = sharedWindow->m_impl->m_window;
      if (!sharedWindow->BorrowContext()) {
        m_status = "Failed to borrow context from shared window";
        return;
      }
    }
    Display::m_impl->m_window = glfwCreateWindow(desiredWidth, desiredHeight, windowName.c_str(), nullptr,
      windowToShare);
    if (sharedWindow) {
      if (!sharedWindow->ReturnContext()) {
        m_status = "Failed to return context to shared window";
        return;
      }
    }

    // Verify that the window was created.
    if (!Display::m_impl->m_window) {
      m_status = "Failed to create GLFW window";
      return;
    }

    // Determine the full-screen monitor to use, if any.
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

    // If we're displaying full-screen engage that here along with specifying the refresh rate.
    if (fullScreenMonitor) {
      glfwSetWindowMonitor(Display::m_impl->m_window, fullScreenMonitor, 0, 0, desiredWidth, desiredHeight, fps);
    }

    // Make the window's context current
    glfwMakeContextCurrent(Display::m_impl->m_window);

    // Initialize GLEW in our context. It is okay to initialize it more than once.
    glewExperimental = true;
    if (glewInit() != GLEW_OK) {
      m_status = "Failed to initialize GLEW";
      return;
    }
    // Clear any GL error that Glew caused.  Apparently on Non-Windows
    // platforms, this can cause a spurious error 1280.
    glGetError();

    // Open the joystick if there is one asked for and there is one present.
    /// @todo

    // Add hooks for mouse input.
    /// @todo

    // Release the window's current context in case another Display wants to borrow it.
    glfwMakeContextCurrent(nullptr);

    // After we're done with the context for set-up and have released it, indicate that the context is available
    // for borrowing.
    Display::m_impl->m_contextAvailable = true;
  }

  // Loop until the display is done.
  bool frameCompleted = false;
  while (!m_done) {
    // Wait until it is time to render the next frame.  We must busy-wait here to avoid having our
    // thread swapped out for longer than we want.
    while (std::chrono::steady_clock::now() < m_impl->m_nextFrameTime) {
    }

    // Grab the context mutex for the duration of the loop.  Once we have it, we know
    // that the context is not active in another context.
    std::lock_guard<std::mutex> lock(Display::m_impl->m_contextMutex);

    // Make the window's context current
    glfwMakeContextCurrent(Display::m_impl->m_window);

    // Quit when our window closes.
    if (glfwWindowShouldClose(Display::m_impl->m_window)) {
      m_composite.reset();
      m_status = "Done";
      break;
    }

    // Process keyboard/mouse/joystick input events and update the viewpoint
    HandleKeyboard();
    /// @todo
    ClampViewOrienation();

    // Handle any window resizing
    SetViewportSizeAndFOVs(m_impl->m_views[0]);

    // Render here
    m_composite->Render(asdp::Time(), m_impl->m_views);

    // Swap front and back buffers and compute the next frame time.
    glfwSwapBuffers(Display::m_impl->m_window);
    m_impl->m_nextFrameTime = std::chrono::steady_clock::now() +
      std::chrono::microseconds(static_cast<long long>(1e6/fps) - renderAheadMicroseconds);

    // Poll for and process events
    glfwPollEvents();

    // Release the window's current context in case another Display wants to borrow it.
    glfwMakeContextCurrent(nullptr);
  }

  // Done with the window
  glfwDestroyWindow(Display::m_impl->m_window);
}

void DisplayWindow::HandleKeyboard()
{
  // See how long it has been since the last keyboard check.  If there has not been one,
  // then set the last check time to now and return.
  if (m_impl->m_lastKeyboardCheck == std::chrono::steady_clock::time_point()) {
    m_impl->m_lastKeyboardCheck = std::chrono::steady_clock::now();
    return;
  }
  auto now = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed = now - m_impl->m_lastKeyboardCheck;
  m_impl->m_lastKeyboardCheck = now;

  double DegreesPerSecond = 30.0;

  // Rotate to look up when the up key is pressed
  if (glfwGetKey(Display::m_impl->m_window, GLFW_KEY_UP) == GLFW_PRESS) {
    m_impl->m_views[0].orientation[0] += DegreesPerSecond * elapsed.count();
  }
  // Rotate to look down when the down key is pressed
  if (glfwGetKey(Display::m_impl->m_window, GLFW_KEY_DOWN) == GLFW_PRESS) {
    m_impl->m_views[0].orientation[0] -= DegreesPerSecond * elapsed.count();
  }
  // Rotate to look right when the right key is pressed
  if (glfwGetKey(Display::m_impl->m_window, GLFW_KEY_RIGHT) == GLFW_PRESS) {
    m_impl->m_views[0].orientation[2] -= DegreesPerSecond * elapsed.count();
  }
  // Rotate to look left when the left key is pressed
  if (glfwGetKey(Display::m_impl->m_window, GLFW_KEY_LEFT) == GLFW_PRESS) {
    m_impl->m_views[0].orientation[2] += DegreesPerSecond * elapsed.count();
  }
}

void DisplayWindow::ClampViewOrienation()
{
  if (m_impl->m_views[0].orientation[0] > 60.0) {
    m_impl->m_views[0].orientation[0] = 60.0;
  }
  if (m_impl->m_views[0].orientation[0] < -60.0) {
    m_impl->m_views[0].orientation[0] = -60.0;
  }
  if (m_impl->m_views[0].orientation[1] > 120.0) {
    m_impl->m_views[0].orientation[1] = 120.0;
  }
  if (m_impl->m_views[0].orientation[1] < -120.0) {
    m_impl->m_views[0].orientation[1] = -120.0;
  }
}

//==============================================================================
// Structures and methods for DisplayTexture class.

class asdp::render::DisplayTexture::DisplayTextureImpl {
public:
  // Nothing here, we re-use base-class objects for everything we need.
};

DisplayTexture::DisplayTexture(Display* sharedWindow)
  : Display(std::shared_ptr<CompositeCube>(), std::shared_ptr<CoreClient>(), 0, 0)
  , m_impl(new DisplayTextureImpl)
{
  // Hold the window mutex so that only one window can be created at a time.
  std::lock_guard<std::mutex> windowLock(m_windowMutex);

  // Set the window to be hidden.
  glfwWindowHint(GLFW_VISIBLE, false);

  // Construct our context, borrowing the context of the shared window so that it will be
  // active on our context (required for Windows).
  GLFWwindow* windowToShare = nullptr;
  if (sharedWindow != nullptr) {
    if (!sharedWindow->BorrowContext()) {
      m_status = "Failed to borrow context from shared window";
      return;
    }
    windowToShare = sharedWindow->m_impl->m_window;
  }
  Display::m_impl->m_window = glfwCreateWindow(100, 100, "", nullptr, windowToShare);
  if (sharedWindow != nullptr) {
    if (!sharedWindow->ReturnContext()) {
      m_status = "Failed to return context to shared window";
      return;
    }
  }

  // Verify that the window was created.
  if (!Display::m_impl->m_window) {
    m_status = "Failed to create GLFW window";
    return;
  }

  // Make the window's context current
  glfwMakeContextCurrent(Display::m_impl->m_window);

  // Initialize GLEW in our context. It is okay to initialize it more than once.
  glewExperimental = true;
  if (glewInit() != GLEW_OK) {
    m_status = "Failed to initialize GLEW";
    return;
  }
  // Clear any GL error that Glew caused.  Apparently on Non-Windows
  // platforms, this can cause a spurious error 1280.
  glGetError();

  // Release the window's current context in case another Display wants to borrow it.
  glfwMakeContextCurrent(nullptr);

  // After we're done with the context for set-up and have released it, indicate that the context is available
  // for borrowing.
  Display::m_impl->m_contextAvailable = true;
}

DisplayTexture::~DisplayTexture()
{
  // Make sure we're done with our rendering state and then clean up.
  Quit();
  m_impl.reset();
}
