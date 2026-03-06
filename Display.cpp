/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

#ifdef WIN32
#define _USE_MATH_DEFINES
#else
#include <arpa/inet.h>		// For ntohl()
#endif
#include <cmath>
#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include <map>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtx/matrix_decompose.hpp>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include "Display.h"

using namespace asdp::render;

namespace asdp {
  namespace render {
/// @brief Static class member to ensure that GLFW is initialized and terminated when this library is
/// loaded and unloaded.
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
  } // namespace render
} // namespace asdp}

/// Ensure that we only create one window at a time.
std::mutex Display::m_windowMutex;

//==============================================================================
// Structures and methods for Display class.

/// @brief Implementation details for the Display class to hide #includes from users of the Display class.
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
  // and ready to be borrowed.
  std::atomic_bool m_contextAvailable{ false };

  // NOTE: All derived classes must lock m_contextMutex when they are using the context and must
  // periodically unlock it so that it can be borrowed to create another context that shares objects
  // with this one.  On Linux, this must be an extended period of time (not just a few instructions)
  // so that another thread can get a chance to lock the mutex.
  std::mutex m_contextMutex;
};

Display::Display(std::shared_ptr<Composite> composite,
  std::shared_ptr<CoreClient> client, uint8_t triggerID, uint32_t triggerAheadMicroseconds,
  uint32_t depthAheadMicroseconds, std::array<float, 3> viewpointOffset,
  std::shared_ptr<EventHandlers> handlers, void* userData)
  : m_viewpointOffset(viewpointOffset)
  , m_composite(composite)
  , m_eventHandlers(handlers)
  , m_userData(userData)
  , m_nowPlaying(true)
  , m_showCameraNames(false)
  , m_client(client)
  , m_triggerID(triggerID)
  , m_offsetMicroseconds(triggerAheadMicroseconds)
  , m_depthAheadMicroseconds(depthAheadMicroseconds)
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

void Display::SetNowPlaying(bool nowPlaying)
{
  m_nowPlaying = nowPlaying;
}

std::string Display::GetStatus() const
{
  return m_status;
}

bool Display::TriggerCameras(std::chrono::steady_clock::time_point when)
{
  if ((m_client == nullptr) || (m_timer == nullptr) || (m_triggerID == 0)) {
    // No client or timer, so we can't trigger the cameras.
    return true;
  }

  // Determine the time to trigger the cameras by subtracting the microseconds
  // offset from the time to trigger the cameras and then converting to Core time.
  std::chrono::steady_clock::time_point sysTime = when - std::chrono::microseconds(m_offsetMicroseconds);
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

/// @brief Implementation details for the DisplayWindow class to hide #includes from users of the DisplayWindow class.
class asdp::render::DisplayWindow::DisplayWindowImpl {
public:
  /// Horizontal field of view in degrees.
  float m_horizontalFOVDegrees {90.0f};

  /// Views to be rendered.
  std::vector<asdp::render::ViewRenderInfo> m_views;

  /// Angles of rotation in degrees based on keyboard and/or joystick input.
  /// rotation is around the original Z axis, then the original Z axis.
  float m_rotationZDegrees {0.0f};
  float m_rotationXDegrees = {0.0f};

  /// Last time we checked the keyboard, used to control motion rate.
  std::chrono::steady_clock::time_point m_lastKeyboardCheck;

  /// Is the left mouse button pressed?
  bool m_leftMouseButtonPressed = false;

  /// The cursor position when the left mouse button was pressed.
  double m_mousePressedX = 0.0;
  double m_mousePressedY = 0.0;

  /// Last time we adjusted due to the mouse, used to control motion rate.
  std::chrono::steady_clock::time_point m_lastMouseMotion;

  /// Time point to start rendering the next frame.
  std::chrono::steady_clock::time_point m_nextRenderTime;

  /// Time point to start computing the depth for the next frame;
  std::chrono::steady_clock::time_point m_nextDepthTime;

  /// Time point in the middle of the next frame we will render.
  std::chrono::steady_clock::time_point m_nextFrameTime;

  /// Whether the space bar was pressed during the last loop, used to toggle play/pause.
  bool m_spacePressed = false;

  /// Whether or no the 'd' key was pressed during the last loop, used to toggle depth computation.
  bool m_dPressed = false;
  bool m_displayingDepth = false;

  /// Whether or not the '[' key was pressed during the last loop, used to ajust active camera index.
  bool m_leftBracketPressed = false;

  /// Whether or not the ']' key was pressed during the last loop, used to ajust active camera index.
  bool m_rightBracketPressed = false;

  // Whether or not the 's' key was pressed during the last loop, used to save camera configuration.
  bool m_sPressed = false;

  /// Whether or not the 'g' key was pressed during the last loop, used to toggle auto-calibration.
  bool m_gPressed = false;

  /// Whether or not the 'o' key was pressed during the last loop, used to toggle overview plus detail view.
  bool m_oPressed = false;

  /// Whether or not the 'c' key was pressed during the last loop, used to toggle camera names display.
  bool m_cPressed = false;

  /// Index of the joystick to use, or -1 if no joystick is to be used.
  int m_glfwJoystickIndex = -1;

  /// Name of joysticks that should be flipping in the Y axis.
  std::vector<std::string> m_flipYJoysticks = { "Logitech Extreme 3D", "Logitech Logitech Extreme 3D" };

  /// Scale of the joystick input in Y axis, flipped if the joystick is on the list above.
  float m_joystickScaleY = 1.0f;

  //======================================
  // Added by Sang Yoon to bind trigger button(s) of a game pad to pausing/resuming action
  bool m_triggerPressed = false;
  //======================================
};

DisplayWindow::DisplayWindow(std::string windowName, std::shared_ptr<Composite> composite,
    std::shared_ptr<CoreClient> client, uint8_t triggerID, uint32_t triggerAheadMicroseconds,
    uint32_t depthAheadMicroseconds, std::array<float, 3> viewpointOffset,
    float fps, uint32_t renderAheadMicroseconds,
    int desiredWidth, int desiredHeight, float horizontalFOVDegrees,
    std::string joystick, Display* sharedWindow,
    bool fullScreen, int desiredDisplay, bool hidden,
    std::shared_ptr<EventHandlers> handlers, void* userData,
    RenderTimingInfo* timingInfo, bool replaying)
  : Display(composite, client, triggerID, triggerAheadMicroseconds, depthAheadMicroseconds, viewpointOffset, handlers, userData)
  , m_timingInfo(timingInfo)
  , m_replaying(replaying)
  , m_impl(new DisplayWindowImpl)

{
  // Check our parameters.
  if ((desiredWidth <= 0) || (desiredHeight <= 0) || (horizontalFOVDegrees <= 0.0f)) {
    m_status = "Invalid window size or field of view";
    return;
  }

  // Store info from the constructor.
  m_impl->m_horizontalFOVDegrees = horizontalFOVDegrees;

  // Construct a single view to be used.  We base is on the requested window size and we compute a
  // field of view that is 40 degrees total horizontal and the correct aspect ratio vertical.
  ViewRenderInfo view;
  SetViewportSizeAndFOVs(view, desiredWidth, desiredHeight);
  view.viewpoint = m_viewpointOffset;
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
  double halfWidth = tan(glm::radians(m_impl->m_horizontalFOVDegrees / 2.0));
  double halfHeight = halfWidth * aspectRatio;
  double halfAngle = glm::degrees(atan(halfHeight));

  //======================================
  // Added by Sang Yoon to calculate a vertical FOV for cylindrical projection
  if (this->m_composite->m_CP_enabled)
      halfAngle = m_impl->m_horizontalFOVDegrees / 2.0 * aspectRatio;
  //======================================

  viewInfo.bottomHalfFOV = -halfAngle;
  viewInfo.topHalfFOV = halfAngle;
}

void DisplayWindow::DisplayThread(std::string windowName,
  float fps, uint32_t renderAheadMicroseconds,
  int desiredWidth, int desiredHeight, float horizontalFOVDegrees,
  std::string joystick, Display* sharedWindow,
  bool fullScreen, int desiredDisplay, bool hidden)
{
  {
    {
      // Hold the window mutex so that only one window can be created at a time.
      std::lock_guard<std::mutex> windowLock(m_windowMutex);

      // Set the window visibility.
      glfwWindowHint(GLFW_VISIBLE, !hidden);

      // Tell it not to iconify full-screen windows that lose focus.
      glfwWindowHint(GLFW_AUTO_ICONIFY, GLFW_FALSE);

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

    // Open the joystick if there is one asked for and there is one present.
    // We currently only support GLFW-based joysticks, which are specified by the string
    // "GLFW::#".  The # is the number of the joystick to open, starting with 0.  GLFW
    // joysticks are alwasy open, so we just need to record which one to use if it is
    // present.
    if (!joystick.empty() && (joystick.substr(0, 6) == "GLFW::")) {
      int joyNum = std::stoi(joystick.substr(6));
      if (glfwJoystickPresent(joyNum)) {
        m_impl->m_glfwJoystickIndex = joyNum;
        // See if we should flip the Y-axis value.
        const char* joystickName = glfwGetJoystickName(joyNum);
        if (std::find(m_impl->m_flipYJoysticks.begin(), m_impl->m_flipYJoysticks.end(),
          glfwGetJoystickName(joyNum)) != m_impl->m_flipYJoysticks.end()) {
          m_impl->m_joystickScaleY = -1.0f;
        }
      }
    }

    // Grab the context mutex for the duration of the setup.  Once we have it, we know
    // that the context is not active in another thread.
    // DO NOT do any GLFW calls while holding the context -- it causes rare hangs on Linux.
    std::lock_guard<std::mutex> lock(Display::m_impl->m_contextMutex);
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
    glFinish();
    Display::m_impl->m_contextAvailable = true;
  }

  // Loop until the display is done.
  bool frameCompleted = false;
  auto lastJoystickCheck = std::chrono::steady_clock::now();
  while (!m_done) {

    // Poll for and process events without a context to avoid multiple threads trying to get
    // the context at the same time.
    glfwPollEvents();

    // Determine the scan-out time of the frame (center of the image).
    Time renderTime;
    m_timer->GetCoreTime(renderTime, std::chrono::steady_clock::now());
    if (!m_replaying) {
      double frameTime = 1.0 / fps;
      double middleOfNextFrameOffset = frameTime / 2.0 + renderAheadMicroseconds / 1e6;
      uint32_t seconds = static_cast<uint32_t>(middleOfNextFrameOffset);
      uint32_t microseconds = (middleOfNextFrameOffset - seconds) * 1e6;
      renderTime += Time(seconds, microseconds);
    }

    // Adjust render time if we're paused.
    if (m_pauseTime) {
      renderTime = *m_pauseTime;
    }

    // Wait until it is time to compute depth for the next frame.  We must busy-wait here to avoid having our
    // thread swapped out for longer than we want.
    while (std::chrono::steady_clock::now() < m_impl->m_nextDepthTime) {
    }
    if (m_eventHandlers && m_eventHandlers->ComputeDepth) {
      // Grab the context mutex for the duration of the depth calculations.  Once we have it, we know
      // that the context is not active in another thread.
      // Make the window's context current.
      // DO NOT do any GLFW calls while holding the context -- it causes rare hangs on Linux.
      std::lock_guard<std::mutex> lock(Display::m_impl->m_contextMutex);
      glfwMakeContextCurrent(Display::m_impl->m_window);

      GLenum err = glGetError();
      if (err != GL_NO_ERROR) {
        std::cerr << "OpenGL error before checking whether to call ComputeDepth: " << err << std::endl;
      }
      m_eventHandlers->ComputeDepth(renderTime, m_userData);

      // Release the window's current context in case another Display wants to borrow it.
      glfwMakeContextCurrent(nullptr);
    }

    // Wait until it is time to render the next frame.  We must busy-wait here to avoid having our
    // thread swapped out for longer than we want.
    while (std::chrono::steady_clock::now() < m_impl->m_nextRenderTime) {
    }

    // Quit when our window closes.
    if (glfwWindowShouldClose(Display::m_impl->m_window)) {
      m_composite.reset();
      m_status = "Done";
      break;
    }

    // Process keyboard/mouse/joystick input events and update the viewpoint

    //======================================
    // Added by Sang Yoon to add key/joystick mappings for closing windows (Q or ESCAPE key on keyboard),
    // resetting viewer's orientation (R key on keyboard or A key on Xbox controller),
    // and pausing/resuming replaying (Left or Right trigger button on Xbox controller)

    // Adding key mappings for closing windows
    if (glfwGetKey(Display::m_impl->m_window, GLFW_KEY_Q) == GLFW_PRESS
        || glfwGetKey(Display::m_impl->m_window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        m_composite.reset();
        m_status = "Done";
        break;
    }

    // Adding key mapping for resetting viewer orientation
    if (glfwGetKey(Display::m_impl->m_window, GLFW_KEY_R) == GLFW_PRESS) {
        m_impl->m_rotationXDegrees = 0.0f;
        m_impl->m_rotationZDegrees = 0.0f;
    }

    if (m_impl->m_glfwJoystickIndex >= 0) {
      // Adding joystick button mapping for resetting viewer's orientation
      int btnCount;
      const unsigned char* btns = glfwGetJoystickButtons(m_impl->m_glfwJoystickIndex, &btnCount);
      if (btnCount > 0) {
        if (btns[0] == GLFW_PRESS) { // 0: A button, 1: B button, 2: X button, 3: Y button for Xbox Controller
          m_impl->m_rotationXDegrees = 0.0f;
          m_impl->m_rotationZDegrees = 0.0f;
        }
      }

      // Adding joystick triggers mapping for pausing/resuming replaying
      int axisCount;
      const float* axes = glfwGetJoystickAxes(m_impl->m_glfwJoystickIndex, &axisCount);
      bool triggerPressed = false;
      float left_trigger = -1.0;
      float right_trigger = -1.0;

      if (axisCount >= 6) {
#ifdef WIN32
        left_trigger = axes[4];
        right_trigger = axes[5];
#else
        // In Linux, the index number for the left trigger is different from that in Windows.
        left_trigger = axes[2];
        right_trigger = axes[5];
#endif
      }
#ifdef WIN32
      else if (axisCount >= 5) {
          left_trigger = axes[4];
#else
      else if (axisCount >= 3) {
          left_trigger = axes[2];
#endif
      }
      if (left_trigger == 1.0 || right_trigger == 1.0)
          triggerPressed = true;
      if (triggerPressed && !m_impl->m_triggerPressed) {
        if (m_eventHandlers && m_eventHandlers->ChangePlayPause) {
          m_eventHandlers->ChangePlayPause(!m_nowPlaying, m_userData);
        }
      }
      m_impl->m_triggerPressed = triggerPressed;
    }
    //======================================

    HandleKeyboard();
    HandleMouse();
    if (m_impl->m_glfwJoystickIndex >= 0) {
      auto now = std::chrono::steady_clock::now();
      std::chrono::duration<double> elapsed = now - lastJoystickCheck;
      lastJoystickCheck = now;

      int axisCount;
      const float* axes = glfwGetJoystickAxes(m_impl->m_glfwJoystickIndex, &axisCount);
      if (axisCount >= 2) {
        if (fabs(axes[0]) > 0.2) {
          m_impl->m_rotationZDegrees -= 90.0f * elapsed.count() * axes[0];
        }
        if (fabs(axes[1]) > 0.2) {
          m_impl->m_rotationXDegrees -= 90.0f * elapsed.count() * axes[1] * m_impl->m_joystickScaleY;
        }

        //======================================
        // Added by Sang Yoon to add the right joystick mappings for viewer's orientation change
#ifdef WIN32
        if (axisCount >= 4) {
          float x_axis = axes[2];
          float y_axis = axes[3];
#else
        // In Linux, the index numbers for the right joystick are different from those in Windows.
        if (axisCount >= 5) {
          float x_axis = axes[3];
          float y_axis = axes[4];
#endif
          if (fabs(x_axis) > 0.2) {
            m_impl->m_rotationZDegrees -= 90.0f * elapsed.count() * x_axis;
          }
          if (fabs(y_axis) > 0.2) {
            m_impl->m_rotationXDegrees -= 90.0f * elapsed.count() * y_axis * m_impl->m_joystickScaleY;
          }
        }
        //======================================
      }
    }

    // Ensure that the view orientation stays within bounds.
    ComputeAndClampViewOrientation();

    // Handle any window resizing
    SetViewportSizeAndFOVs(m_impl->m_views[0]);

    // Trigger the cameras, saying that we need the data now. The base class will handle offsetting
    // by the specified transmission/processing time as passed to its constructor by the client.
    TriggerCameras(std::chrono::steady_clock::now());

    // Record the render start time if we have a place to put it.
    if (m_timingInfo) {
      m_timingInfo->renderStartTimes.push_back(std::chrono::steady_clock::now());
    }

    // Grab the context mutex for the duration of the loop.  Once we have it, we know
    // that the context is not active in another thread.
    // Make the window's context current.
    // DO NOT do any GLFW calls while holding the context -- it causes rare hangs on Linux.
    std::lock_guard<std::mutex> lock(Display::m_impl->m_contextMutex);
    glfwMakeContextCurrent(Display::m_impl->m_window);

    m_composite->Render(renderTime, m_impl->m_views);

    // Record the render submit time if we have a place to put it.
    if (m_timingInfo) {
      m_timingInfo->renderSubmitTimes.push_back(std::chrono::steady_clock::now());
    }

    // Swap front and back buffers and wait for it to complete, then compute the next frame time.
    glfwSwapBuffers(Display::m_impl->m_window);
    glFinish();
    m_impl->m_nextRenderTime = std::chrono::steady_clock::now() +
      std::chrono::microseconds(static_cast<long long>(1e6/fps) - renderAheadMicroseconds);
    m_impl->m_nextDepthTime = m_impl->m_nextRenderTime - std::chrono::microseconds(m_depthAheadMicroseconds);
    // Half way through the next frame, which is when we want the geometry adjusted for.
    m_impl->m_nextFrameTime = std::chrono::steady_clock::now() +
      std::chrono::microseconds(static_cast<long long>(1e6/fps)*3/2);

    // Release the window's current context in case another Display wants to borrow it.
    glfwMakeContextCurrent(nullptr);
  }

  // Done with the window
  glfwDestroyWindow(Display::m_impl->m_window);
}

void DisplayWindow::SetNowPlaying(bool nowPlaying)
{
  // Call the parent-class method to set the now-playing state.
  Display::SetNowPlaying(nowPlaying);

  // Set the pause time based on whether we are now playing so that
  // we don't extrapolate forward in time while paused.
  if (!m_nowPlaying) {
    m_pauseTime = std::make_unique<Time>();
    m_timer->GetCoreTime(*m_pauseTime, std::chrono::steady_clock::now());
  } else {
    m_pauseTime.reset();
  }
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
    m_impl->m_rotationXDegrees += DegreesPerSecond * elapsed.count();
  }

  // Rotate to look down when the down key is pressed
  if (glfwGetKey(Display::m_impl->m_window, GLFW_KEY_DOWN) == GLFW_PRESS) {
    m_impl->m_rotationXDegrees -= DegreesPerSecond * elapsed.count();
  }

  // Rotate to look right when the right key is pressed
  if (glfwGetKey(Display::m_impl->m_window, GLFW_KEY_RIGHT) == GLFW_PRESS) {
    m_impl->m_rotationZDegrees -= DegreesPerSecond * elapsed.count();
  }

  // Rotate to look left when the left key is pressed
  if (glfwGetKey(Display::m_impl->m_window, GLFW_KEY_LEFT) == GLFW_PRESS) {
    m_impl->m_rotationZDegrees += DegreesPerSecond * elapsed.count();
  }

  // Toggle play/pause when the space key is pressed (once per press/release cycle).
  bool spacePressed = (glfwGetKey(Display::m_impl->m_window, GLFW_KEY_SPACE) == GLFW_PRESS);
  if (spacePressed && !m_impl->m_spacePressed) {
    if (m_eventHandlers && m_eventHandlers->ChangePlayPause) {
      m_eventHandlers->ChangePlayPause(!m_nowPlaying, m_userData);
    }
  }
  m_impl->m_spacePressed = spacePressed;

  // Toggle depth computation when the 'd' key is pressed (once per press/release cycle).
  bool dPressed = (glfwGetKey(Display::m_impl->m_window, GLFW_KEY_D) == GLFW_PRESS);
  if (dPressed && !m_impl->m_dPressed) {
    m_impl->m_displayingDepth = !m_impl->m_displayingDepth;
    if (m_eventHandlers && m_eventHandlers->SetToRenderDepth) {
      m_eventHandlers->SetToRenderDepth(m_impl->m_displayingDepth, m_userData);
    }
  }
  m_impl->m_dPressed = dPressed;

  // Adjust the active camera index when the '[' or ']' keys are pressed.
  bool leftBracketPressed = (glfwGetKey(Display::m_impl->m_window, GLFW_KEY_LEFT_BRACKET) == GLFW_PRESS);
  if (leftBracketPressed && !m_impl->m_leftBracketPressed) {
    if (m_eventHandlers && m_eventHandlers->DecrementActiveCamera) {
      m_eventHandlers->DecrementActiveCamera(m_userData);
    }
  }
  m_impl->m_leftBracketPressed = leftBracketPressed;
  bool rightBracketPressed = (glfwGetKey(Display::m_impl->m_window, GLFW_KEY_RIGHT_BRACKET) == GLFW_PRESS);
  if (rightBracketPressed && !m_impl->m_rightBracketPressed) {
    if (m_eventHandlers && m_eventHandlers->IncrementActiveCamera) {
      m_eventHandlers->IncrementActiveCamera(m_userData);
    }
  }
  m_impl->m_rightBracketPressed = rightBracketPressed;

  // Adjust the camera offset for the active camera while the '-' (decrement) or '=' (increment) keys are pressed.
  bool minusPressed = (glfwGetKey(Display::m_impl->m_window, GLFW_KEY_MINUS) == GLFW_PRESS);
  bool equalPressed = (glfwGetKey(Display::m_impl->m_window, GLFW_KEY_EQUAL) == GLFW_PRESS);
  int increment = 0;
  if (minusPressed) {
    increment = -1;
  } else if (equalPressed) {
    increment = 1;
  }
  if (increment != 0) {
    if (m_eventHandlers && m_eventHandlers->AdjustActiveCameraOffset) {
      m_eventHandlers->AdjustActiveCameraOffset(increment, m_userData);
    }
  }

  // Adjust the camera gain for the active camera while the ',' (decrement) or '.' (increment) keys are pressed.
  bool periodPressed = (glfwGetKey(Display::m_impl->m_window, GLFW_KEY_PERIOD) == GLFW_PRESS);
  bool commaPressed = (glfwGetKey(Display::m_impl->m_window, GLFW_KEY_COMMA) == GLFW_PRESS);
  float incrementGain = 1;
  if (periodPressed) {
    incrementGain *= 1.001f;
  } else if (commaPressed) {
    incrementGain /= 1.001f;
  }
  if (incrementGain != 1) {
    if (m_eventHandlers && m_eventHandlers->AdjustActiveCameraGain) {
      m_eventHandlers->AdjustActiveCameraGain(incrementGain, m_userData);
    }
  }

  bool gPressed = (glfwGetKey(Display::m_impl->m_window, GLFW_KEY_G) == GLFW_PRESS);
  if (gPressed && !m_impl->m_gPressed) {
    if (m_eventHandlers && m_eventHandlers->AutoUpdateColorOffsetsAndGains) {
      m_eventHandlers->AutoUpdateColorOffsetsAndGains(m_userData);
    }
  }
  m_impl->m_gPressed = gPressed;

  bool oPressed = (glfwGetKey(Display::m_impl->m_window, GLFW_KEY_O) == GLFW_PRESS);
  if (oPressed && !m_impl->m_oPressed) {
    if (m_eventHandlers && m_eventHandlers->AutoUpdateColorOffsets) {
      m_eventHandlers->AutoUpdateColorOffsets(m_userData);
    }
  }
  m_impl->m_oPressed = oPressed;

  // If the 's' key is pressed, send an event asking to save the current configuration file.
  bool sPressed = (glfwGetKey(Display::m_impl->m_window, GLFW_KEY_S) == GLFW_PRESS);
  if (sPressed && !m_impl->m_sPressed) {
    if (m_eventHandlers && m_eventHandlers->SaveCameraConfig) {
      m_eventHandlers->SaveCameraConfig("adjusted_camera_config.json", m_userData);
    }
  }
  m_impl->m_sPressed = sPressed;

  // If the 'c' key is pressed, toggle the display of camera names.
  bool cPressed = (glfwGetKey(Display::m_impl->m_window, GLFW_KEY_C) == GLFW_PRESS);
  if (cPressed && !m_impl->m_cPressed) {
    m_showCameraNames = !m_showCameraNames;
    if (m_eventHandlers && m_eventHandlers->ShowCameraNames) {
      m_eventHandlers->ShowCameraNames(m_showCameraNames, m_userData);
    }
  }
  m_impl->m_cPressed = cPressed;
}

void DisplayWindow::HandleMouse()
{
  // If the mouse button has not been pressed and it now is pressed, set the mouse pressed
  // position to the current position.
  if (!m_impl->m_leftMouseButtonPressed && (glfwGetMouseButton(Display::m_impl->m_window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS)) {
    m_impl->m_leftMouseButtonPressed = true;
    glfwGetCursorPos(Display::m_impl->m_window, &m_impl->m_mousePressedX, &m_impl->m_mousePressedY);
    m_impl->m_lastMouseMotion = std::chrono::steady_clock::now();
    return;
  }

  // If the mouse button is not now pressed, clear the pressed flag and we're done.
  if (glfwGetMouseButton(Display::m_impl->m_window, GLFW_MOUSE_BUTTON_LEFT) != GLFW_PRESS) {
    m_impl->m_leftMouseButtonPressed = false;
    return;
  }

  // Find the current mouse position and delta from the pressed position.  Scale by half the window height
  // and the maximum rate and the time in seconds and adjust the viewpoint accordingly.
  double xpos, ypos;
  glfwGetCursorPos(Display::m_impl->m_window, &xpos, &ypos);
  double deltaX = xpos - m_impl->m_mousePressedX;
  double deltaY = ypos - m_impl->m_mousePressedY;

  // Scale the deltas to be a fraction of half the window height (scale them both by the same amount).
  int width, height;
  glfwGetWindowSize(Display::m_impl->m_window, &width, &height);
  deltaX /= (height / 2.0);
  deltaY /= (height / 2.0);

  auto now = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed = now - m_impl->m_lastMouseMotion;
  m_impl->m_lastMouseMotion = now;

  // Handle the mouse movement with the button held down
  // We scale the deltas and increment the rotation angles so long as the button is held down.
  double MaxDegreesPerSecond = 45.0;
  m_impl->m_rotationZDegrees -= MaxDegreesPerSecond * deltaX * elapsed.count();
  m_impl->m_rotationXDegrees -= MaxDegreesPerSecond * deltaY * elapsed.count();
}

void DisplayWindow::ComputeAndClampViewOrientation()
{
  // Clamp the rotation angles to reasonable values.
  if (m_impl->m_rotationXDegrees > 60.0) {
    m_impl->m_rotationXDegrees = 60.0;
  }
  if (m_impl->m_rotationXDegrees < -60.0) {
    m_impl->m_rotationXDegrees = -60.0;
  }
  if (m_impl->m_rotationZDegrees > 120.0) {
    m_impl->m_rotationZDegrees = 120.0;
  }
  if (m_impl->m_rotationZDegrees < -120.0) {
    m_impl->m_rotationZDegrees = -120.0;
  }

  // Compute the orientation Quarternion by building two different rotation
  // matrices and applying them in the correct order.
  float rotationZRadians = glm::radians(m_impl->m_rotationZDegrees);
  float rotationXRadians = glm::radians(m_impl->m_rotationXDegrees);

  // Create rotation matrices
  // Combine the rotations: first Z, then X
  /// @todo Consider doing this with just quaternions and axis-angles.
  glm::mat4 rotationZ = glm::rotate(glm::mat4(1.0f), rotationZRadians, glm::vec3(0.0f, 0.0f, 1.0f));
  glm::mat4 rotationX = glm::rotate(rotationZ, rotationXRadians, glm::vec3(1.0f, 0.0f, 0.0f));

  glm::mat4 combinedRotation = rotationX;

  // Decompose the combined rotation matrix to get the quaternion.
  glm::vec3 scale, translation, skew;
  glm::vec4 perspective;
  glm::quat orientation;
  glm::decompose(combinedRotation, scale, orientation, translation, skew, perspective);

  // Store the quaternion.
  m_impl->m_views[0].orientation[0] = orientation.w;
  m_impl->m_views[0].orientation[1] = orientation.x;
  m_impl->m_views[0].orientation[2] = orientation.y;
  m_impl->m_views[0].orientation[3] = orientation.z;
}

//==============================================================================
// Structures and methods for DisplayTexture class.

/// @brief Implementation details for the DisplayTexture class to hide #includes from users of the DisplayTexture class.
class asdp::render::DisplayTexture::DisplayTextureImpl {
public:
  // Nothing here, we re-use base-class objects for everything we need.
};

DisplayTexture::DisplayTexture(Display* sharedWindow)
  : Display(std::shared_ptr<CompositeCube>(), std::shared_ptr<CoreClient>(), 0, 0, 0)
  , m_impl(new DisplayTextureImpl)
{
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
  }

  // Verify that the window was created.
  if (!Display::m_impl->m_window) {
    m_status = "Failed to create GLFW window";
    return;
  }

  // Grab the context mutex for the duration of the setup.  Once we have it, we know
  // that the context is not active in another thread.
  // Make the window's context current.
  // DO NOT do any GLFW calls while holding the context -- it causes rare hangs on Linux.
  std::lock_guard<std::mutex> lock(Display::m_impl->m_contextMutex);
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
  glFinish();
  Display::m_impl->m_contextAvailable = true;
}

DisplayTexture::~DisplayTexture()
{
  // Make sure we're done with our rendering state and then clean up.
  Quit();
  m_impl.reset();
}

#ifdef USE_OPENXR
 #include "pch.h"
 #include "common.h"
 #include "check.h"
 #ifdef _WIN32
  #define GLFW_EXPOSE_NATIVE_WIN32
  #define GLFW_EXPOSE_NATIVE_WGL
  #include <GLFW/glfw3native.h>
 #endif

//==============================================================================
// Structures and methods for DisplayOpenXR class.

//============================================================================================
// Helper functions and definitions.

namespace Math {
  namespace Pose {
    XrPosef Identity() {
      XrPosef t{};
      t.orientation.w = 1;
      return t;
    }

    XrPosef Translation(const XrVector3f& translation) {
      XrPosef t = Identity();
      t.position = translation;
      return t;
    }

    XrPosef RotateCCWAboutYAxis(float radians, XrVector3f translation) {
      XrPosef t = Identity();
      t.orientation.x = 0.f;
      t.orientation.y = std::sin(radians * 0.5f);
      t.orientation.z = 0.f;
      t.orientation.w = std::cos(radians * 0.5f);
      t.position = translation;
      return t;
    }
  }  // namespace Pose
}  // namespace Math

namespace Side {
  const int LEFT = 0;
  const int RIGHT = 1;
  const int COUNT = 2;
}  // namespace Side

/// @brief Implementation details for the DisplayOpenXR class to hide #includes from users of the DisplayOpenXR class.
class asdp::render::DisplayOpenXR::DisplayOpenXRImpl {
public:
  DisplayOpenXRImpl(asdp::render::DisplayOpenXR* display)
    : m_display(display)
    , m_lastGrabbedState(false)
  {
  }
  DisplayOpenXR* m_display = nullptr;

  XrInstance m_instance{ XR_NULL_HANDLE };
  XrSession m_session{ XR_NULL_HANDLE };
  XrSpace m_appSpace{ XR_NULL_HANDLE };
  XrSystemId m_systemId{ XR_NULL_SYSTEM_ID };
  XrFormFactor m_formFactor{ XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY };
  XrViewConfigurationType m_viewConfigType{ XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO };
  XrEnvironmentBlendMode m_environmentBlendMode{ XR_ENVIRONMENT_BLEND_MODE_OPAQUE };
  int m_verbosity{ 0 };

  /// @todo Change these to match the desired behavior.
  struct Options {
    std::string GraphicsPlugin;
    XrFormFactor FormFactor{ XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY };
    XrViewConfigurationType ViewConfiguration{ XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO };
    XrEnvironmentBlendMode EnvironmentBlendMode{ XR_ENVIRONMENT_BLEND_MODE_OPAQUE };
    std::string AppSpace{ "Local" };
  } m_options;

#ifdef XR_USE_PLATFORM_WIN32
  XrGraphicsBindingOpenGLWin32KHR m_graphicsBinding{ XR_TYPE_GRAPHICS_BINDING_OPENGL_WIN32_KHR };
#elif defined(XR_USE_PLATFORM_XLIB)
  XrGraphicsBindingOpenGLXlibKHR m_graphicsBinding{ XR_TYPE_GRAPHICS_BINDING_OPENGL_XLIB_KHR };
#elif defined(XR_USE_PLATFORM_XCB)
  XrGraphicsBindingOpenGLXcbKHR m_graphicsBinding{ XR_TYPE_GRAPHICS_BINDING_OPENGL_XCB_KHR };
#elif defined(XR_USE_PLATFORM_WAYLAND)
  XrGraphicsBindingOpenGLWaylandKHR m_graphicsBinding{ XR_TYPE_GRAPHICS_BINDING_OPENGL_WAYLAND_KHR };
#endif
  GLFWwindow* m_contextWindow{ nullptr };

  // Application's current lifecycle state according to the runtime
  XrSessionState m_sessionState{ XR_SESSION_STATE_UNKNOWN };
  bool m_sessionRunning{ false };

  XrEventDataBuffer m_eventDataBuffer{};

  struct InputState {
    XrActionSet actionSet{ XR_NULL_HANDLE };
    XrAction grabAction{ XR_NULL_HANDLE };
    XrAction poseAction{ XR_NULL_HANDLE };
    XrAction vibrateAction{ XR_NULL_HANDLE };
    XrAction quitAction{ XR_NULL_HANDLE };
    std::array<XrPath, Side::COUNT> handSubactionPath = {};
    std::array<XrSpace, Side::COUNT> handSpace = {};
    std::array<float, Side::COUNT> handScale = { {1.0f, 1.0f} };
    std::array<XrBool32, Side::COUNT> handActive = {};
  };
  InputState m_input;

  struct Swapchain {
    XrSwapchain handle;
    int32_t width;
    int32_t height;
  };

  std::vector<XrViewConfigurationView> m_configViews;
  std::vector<Swapchain> m_swapchains;
  std::map<XrSwapchain, std::vector<XrSwapchainImageBaseHeader*>> m_swapchainImages;
  std::vector<XrView> m_views;
  int64_t m_colorSwapchainFormat{ -1 };

  std::list<std::vector<XrSwapchainImageOpenGLKHR>> m_swapchainImageBuffers;
  GLuint m_swapchainFramebuffer{ 0 };

  // Map color buffer to associated depth buffer. This map is populated on demand.
  std::map<uint32_t, uint32_t> m_colorToDepthMap;

#ifdef XR_USE_PLATFORM_WIN32
  PFN_xrConvertTimeToWin32PerformanceCounterKHR m_xrConvertTimeToWin32PerformanceCounterKHR = nullptr;
#endif

  /// Keep track of the last grabbed state so that we can toggle the play/pause state when it is triggered.
  bool m_lastGrabbedState;

  /// Pointer to time when we started pausing, nullptr if not pausing.
  std::unique_ptr<Time> m_pauseTime;

  /// Stored estimated frame duration, initialize based on 90fps.
  XrDuration m_frameDurationNS{ XrDuration(1.0e9/90) };

  void OpenXRCreateInstance();
  void OpenXRInitializeSystem(Display* sharedWindow);
  void OpenGLInitializeDevice(Display* sharedWindow, XrInstance instance, XrSystemId systemId);
  void OpenXRInitializeSession();
  void OpenXRInitializeActions();
  XrReferenceSpaceCreateInfo GetXrReferenceSpaceCreateInfo(const std::string& referenceSpaceTypeStr);
  void OpenXRCreateSwapchains();
  int64_t OpenGLSelectColorSwapchainFormat(const std::vector<int64_t>& runtimeFormats);
  std::vector<XrSwapchainImageBaseHeader*> OpenGLAllocateSwapchainImageStructs(
    uint32_t capacity, const XrSwapchainCreateInfo& /*swapchainCreateInfo*/);
  XrEventDataBaseHeader* OpenXRTryReadNextEvent();
  void OpenXRHandleSessionStateChangedEvent(
    const XrEventDataSessionStateChanged& stateChangedEvent, bool* exitRenderLoop,
    bool* requestRestart);
  void OpenXRPollEvents(bool* exitRenderLoop, bool* requestRestart);
  void OpenXRPollActions();
  bool OpenXRRenderLayer(XrTime predictedDisplayTime,
    std::vector<XrCompositionLayerProjectionView>& projectionLayerViews,
    XrCompositionLayerProjection& layer);
  void OpenXRRenderFrame();
  void OpenGLInitializeResources();
  uint32_t OpenGLGetDepthTexture(uint32_t colorTexture);
  void OpenGLTearDown();
  void OpenXRTearDown();
};

void asdp::render::DisplayOpenXR::DisplayOpenXRImpl::OpenXRCreateInstance()
{
#ifdef XR_USE_PLATFORM_WIN32
  CHECK_HRCMD(CoInitializeEx(nullptr, COINIT_MULTITHREADED));
#endif

  CHECK(m_instance == XR_NULL_HANDLE);

  // Create union of extensions required by OpenGL.
  std::vector<const char*> extensions = { XR_KHR_OPENGL_ENABLE_EXTENSION_NAME };
#ifdef XR_USE_PLATFORM_WIN32
  extensions.push_back(XR_KHR_WIN32_CONVERT_PERFORMANCE_COUNTER_TIME_EXTENSION_NAME);
#endif

  XrInstanceCreateInfo createInfo{ XR_TYPE_INSTANCE_CREATE_INFO };
  createInfo.next = nullptr;  // Needs to be set on Android.
  createInfo.enabledExtensionCount = (uint32_t)extensions.size();
  createInfo.enabledExtensionNames = extensions.data();

  strcpy(createInfo.applicationInfo.applicationName, "asdp::render::DisplayOpenXR");
  createInfo.applicationInfo.apiVersion = XR_CURRENT_API_VERSION;

  CHECK_XRCMD(xrCreateInstance(&createInfo, &m_instance));

#ifdef XR_USE_PLATFORM_WIN32
  // Load the Windows performance timer extension function
  xrGetInstanceProcAddr(m_instance, "xrConvertTimeToWin32PerformanceCounterKHR",
    reinterpret_cast<PFN_xrVoidFunction*>(&m_xrConvertTimeToWin32PerformanceCounterKHR));

  if (!m_xrConvertTimeToWin32PerformanceCounterKHR) {
    std::cerr << "Warning: DisplayOpenXR() Failed to load xrConvertTimeToWin32PerformanceCounterKHR function." << std::endl;
  }
#endif
}

void asdp::render::DisplayOpenXR::DisplayOpenXRImpl::OpenXRInitializeSystem(Display* sharedWindow)
{
  CHECK(m_instance != XR_NULL_HANDLE);
  CHECK(m_systemId == XR_NULL_SYSTEM_ID);

  m_formFactor = m_options.FormFactor;
  m_viewConfigType = m_options.ViewConfiguration;
  m_environmentBlendMode = m_options.EnvironmentBlendMode;

  XrSystemGetInfo systemInfo{ XR_TYPE_SYSTEM_GET_INFO };
  systemInfo.formFactor = m_formFactor;
  CHECK_XRCMD(xrGetSystem(m_instance, &systemInfo, &m_systemId));

  if (m_verbosity >= 2) std::cout << "Using system " << m_systemId
    << " for form factor " << to_string(m_formFactor) << std::endl;
  CHECK(m_instance != XR_NULL_HANDLE);
  CHECK(m_systemId != XR_NULL_SYSTEM_ID);

  // The graphics API can initialize the graphics device now that the systemId and instance
  // handle are available.
  OpenGLInitializeDevice(sharedWindow, m_instance, m_systemId);
}

void asdp::render::DisplayOpenXR::DisplayOpenXRImpl::OpenGLInitializeDevice(Display* sharedWindow, XrInstance instance, XrSystemId systemId)
{
  // Extension function must be loaded by name
  PFN_xrGetOpenGLGraphicsRequirementsKHR pfnGetOpenGLGraphicsRequirementsKHR = nullptr;
  CHECK_XRCMD(xrGetInstanceProcAddr(instance, "xrGetOpenGLGraphicsRequirementsKHR",
    reinterpret_cast<PFN_xrVoidFunction*>(&pfnGetOpenGLGraphicsRequirementsKHR)));

  XrGraphicsRequirementsOpenGLKHR graphicsRequirements{ XR_TYPE_GRAPHICS_REQUIREMENTS_OPENGL_KHR };
  CHECK_XRCMD(pfnGetOpenGLGraphicsRequirementsKHR(instance, systemId, &graphicsRequirements));

  {
    // Hold the window mutex so that only one window can be created at a time.
    std::lock_guard<std::mutex> windowLock(m_windowMutex);

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
        THROW("DisplayOpenXR::DisplayOpenXRImpl::OpenGLInitializeDevice(): Failed to borrow context from shared window");
        return;
      }
    }

    // Open a window that we will use to get a context that we will use to hand to OpenXR as needed.
    // This is a bit of a hack, but it is the only way to get a context that we can use with OpenXR.
    // We will use the context from this window to create the OpenXR session.
    // Set the window to be not hidden so that it will always be cleaned up and won't leave a zombie
    // GL object that keeps us from opening new OpenXR apps.
    glfwWindowHint(GLFW_VISIBLE, true);
    m_contextWindow = glfwCreateWindow(100, 100, "ASDP_Render_Module OpenXR OpenGL Window to get context", nullptr, windowToShare);
    if (sharedWindow) {
      if (!sharedWindow->ReturnContext()) {
        THROW("OpenGLInitializeDevice(): Failed to return context to shared window");
        return;
      }
    }
    // Verify that the window was created.
    if (!m_contextWindow) {
      THROW("DisplayOpenXR::DisplayOpenXRImpl::OpenGLInitializeDevice(): Failed to create GLFW window");
      return;
    }
    // Grab the context mutex.  Once we have it, we know that the context is not active in another thread.
    // Make the window's context current
    m_display->Display::m_impl->m_contextMutex.lock();
    glfwMakeContextCurrent(m_contextWindow);
  }

  // Determine the OpenGL version.
  GLint major = 0;
  GLint minor = 0;
  glGetIntegerv(GL_MAJOR_VERSION, &major);
  glGetIntegerv(GL_MINOR_VERSION, &minor);

  const XrVersion desiredApiVersion = XR_MAKE_VERSION(major, minor, 0);
  if (graphicsRequirements.minApiVersionSupported > desiredApiVersion) {
    THROW("DisplayOpenXR::DisplayOpenXRImpl::OpenGLInitializeDevice(): Runtime does not support desired Graphics API and/or version");
  }
#ifdef XR_USE_PLATFORM_WIN32
  m_graphicsBinding.hDC = GetDC(glfwGetWin32Window(m_contextWindow));
  m_graphicsBinding.hGLRC = glfwGetWGLContext(m_contextWindow);
#elif defined(XR_USE_PLATFORM_XLIB)
  THROW("DisplayOpenXR::DisplayOpenXRImpl::OpenGLInitializeDevice(): Xlib not implemented here");
#elif defined(XR_USE_PLATFORM_XCB)
  THROW("DisplayOpenXR::DisplayOpenXRImpl::OpenGLInitializeDevice(): XCB not implemented here");
#elif defined(XR_USE_PLATFORM_WAYLAND)
  THROW("DisplayOpenXR::DisplayOpenXRImpl::OpenGLInitializeDevice(): Wayland not implemented here");
#endif

  /** @todo Can enable this for debugging
  glEnable(GL_DEBUG_OUTPUT);
  glDebugMessageCallback(
    [](GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar* message,
      const void* userParam) {
        std::cout << "GL Debug: " << std::string(message, 0, length) << std::endl;
    },
    nullptr);
  */
  OpenGLInitializeResources();
}

XrReferenceSpaceCreateInfo asdp::render::DisplayOpenXR::DisplayOpenXRImpl::GetXrReferenceSpaceCreateInfo(const std::string& referenceSpaceTypeStr)
{
  XrReferenceSpaceCreateInfo referenceSpaceCreateInfo{ XR_TYPE_REFERENCE_SPACE_CREATE_INFO };
  referenceSpaceCreateInfo.poseInReferenceSpace = Math::Pose::Identity();
  if (EqualsIgnoreCase(referenceSpaceTypeStr, "View")) {
    referenceSpaceCreateInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_VIEW;
  } else if (EqualsIgnoreCase(referenceSpaceTypeStr, "ViewFront")) {
    // Render head-locked 2m in front of device.
    referenceSpaceCreateInfo.poseInReferenceSpace = Math::Pose::Translation({ 0.f, 0.f, -2.f }),
      referenceSpaceCreateInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_VIEW;
  } else if (EqualsIgnoreCase(referenceSpaceTypeStr, "Local")) {
    referenceSpaceCreateInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_LOCAL;
  } else if (EqualsIgnoreCase(referenceSpaceTypeStr, "Stage")) {
    referenceSpaceCreateInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_STAGE;
  } else if (EqualsIgnoreCase(referenceSpaceTypeStr, "StageLeft")) {
    referenceSpaceCreateInfo.poseInReferenceSpace = Math::Pose::RotateCCWAboutYAxis(0.f, { -2.f, 0.f, -2.f });
    referenceSpaceCreateInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_STAGE;
  } else if (EqualsIgnoreCase(referenceSpaceTypeStr, "StageRight")) {
    referenceSpaceCreateInfo.poseInReferenceSpace = Math::Pose::RotateCCWAboutYAxis(0.f, { 2.f, 0.f, -2.f });
    referenceSpaceCreateInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_STAGE;
  } else if (EqualsIgnoreCase(referenceSpaceTypeStr, "StageLeftRotated")) {
    referenceSpaceCreateInfo.poseInReferenceSpace = Math::Pose::RotateCCWAboutYAxis(3.14f / 3.f, { -2.f, 0.5f, -2.f });
    referenceSpaceCreateInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_STAGE;
  } else if (EqualsIgnoreCase(referenceSpaceTypeStr, "StageRightRotated")) {
    referenceSpaceCreateInfo.poseInReferenceSpace = Math::Pose::RotateCCWAboutYAxis(-3.14f / 3.f, { 2.f, 0.5f, -2.f });
    referenceSpaceCreateInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_STAGE;
  } else {
    throw std::invalid_argument(Fmt("Unknown reference space type '%s'", referenceSpaceTypeStr.c_str()));
  }
  return referenceSpaceCreateInfo;
}

void asdp::render::DisplayOpenXR::DisplayOpenXRImpl::OpenXRInitializeSession()
{
  CHECK(m_instance != XR_NULL_HANDLE);
  CHECK(m_session == XR_NULL_HANDLE);

  {
    if (m_verbosity >= 2) std::cout << Fmt("Creating session...") << std::endl;

    XrSessionCreateInfo createInfo{ XR_TYPE_SESSION_CREATE_INFO };
    createInfo.next = reinterpret_cast<const XrBaseInStructure*>(&m_graphicsBinding);
    createInfo.systemId = m_systemId;
    CHECK_XRCMD(xrCreateSession(m_instance, &createInfo, &m_session));
  }

  OpenXRInitializeActions();
  // Do not need unless we want other than helicopter space:  OpenXRCreateVisualizedSpaces();

  {
    XrReferenceSpaceCreateInfo referenceSpaceCreateInfo = GetXrReferenceSpaceCreateInfo(m_options.AppSpace);
    CHECK_XRCMD(xrCreateReferenceSpace(m_session, &referenceSpaceCreateInfo, &m_appSpace));
  }
}

/// @todo Change the behaviors by modifying the action bindings.

void asdp::render::DisplayOpenXR::DisplayOpenXRImpl::OpenXRInitializeActions()
{
  // Create an action set.
  {
    XrActionSetCreateInfo actionSetInfo{ XR_TYPE_ACTION_SET_CREATE_INFO };
    strcpy_s(actionSetInfo.actionSetName, "gameplay");
    strcpy_s(actionSetInfo.localizedActionSetName, "Gameplay");
    actionSetInfo.priority = 0;
    CHECK_XRCMD(xrCreateActionSet(m_instance, &actionSetInfo, &m_input.actionSet));
  }

  // Get the XrPath for the left and right hands - we will use them as subaction paths.
  CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/left", &m_input.handSubactionPath[Side::LEFT]));
  CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/right", &m_input.handSubactionPath[Side::RIGHT]));

  // Create actions.
  {
    // Create an input action for grabbing objects with the left and right hands.
    XrActionCreateInfo actionInfo{ XR_TYPE_ACTION_CREATE_INFO };
    actionInfo.actionType = XR_ACTION_TYPE_FLOAT_INPUT;
    strcpy_s(actionInfo.actionName, "grab_object");
    strcpy_s(actionInfo.localizedActionName, "Grab Object");
    actionInfo.countSubactionPaths = uint32_t(m_input.handSubactionPath.size());
    actionInfo.subactionPaths = m_input.handSubactionPath.data();
    CHECK_XRCMD(xrCreateAction(m_input.actionSet, &actionInfo, &m_input.grabAction));

    // Create an input action getting the left and right hand poses.
    actionInfo.actionType = XR_ACTION_TYPE_POSE_INPUT;
    strcpy_s(actionInfo.actionName, "hand_pose");
    strcpy_s(actionInfo.localizedActionName, "Hand Pose");
    actionInfo.countSubactionPaths = uint32_t(m_input.handSubactionPath.size());
    actionInfo.subactionPaths = m_input.handSubactionPath.data();
    CHECK_XRCMD(xrCreateAction(m_input.actionSet, &actionInfo, &m_input.poseAction));

    // Create output actions for vibrating the left and right controller.
    actionInfo.actionType = XR_ACTION_TYPE_VIBRATION_OUTPUT;
    strcpy_s(actionInfo.actionName, "vibrate_hand");
    strcpy_s(actionInfo.localizedActionName, "Vibrate Hand");
    actionInfo.countSubactionPaths = uint32_t(m_input.handSubactionPath.size());
    actionInfo.subactionPaths = m_input.handSubactionPath.data();
    CHECK_XRCMD(xrCreateAction(m_input.actionSet, &actionInfo, &m_input.vibrateAction));

    // Create input actions for quitting the session using the left and right controller.
    // Since it doesn't matter which hand did this, we do not specify subaction paths for it.
    // We will just suggest bindings for both hands, where possible.
    actionInfo.actionType = XR_ACTION_TYPE_BOOLEAN_INPUT;
    strcpy_s(actionInfo.actionName, "quit_session");
    strcpy_s(actionInfo.localizedActionName, "Quit Session");
    actionInfo.countSubactionPaths = 0;
    actionInfo.subactionPaths = nullptr;
    CHECK_XRCMD(xrCreateAction(m_input.actionSet, &actionInfo, &m_input.quitAction));
  }

  std::array<XrPath, Side::COUNT> selectPath;
  std::array<XrPath, Side::COUNT> squeezeValuePath;
  std::array<XrPath, Side::COUNT> squeezeForcePath;
  std::array<XrPath, Side::COUNT> squeezeClickPath;
  std::array<XrPath, Side::COUNT> posePath;
  std::array<XrPath, Side::COUNT> hapticPath;
  std::array<XrPath, Side::COUNT> menuClickPath;
  std::array<XrPath, Side::COUNT> bClickPath;
  std::array<XrPath, Side::COUNT> triggerValuePath;
  CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/left/input/select/click", &selectPath[Side::LEFT]));
  CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/right/input/select/click", &selectPath[Side::RIGHT]));
  CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/left/input/squeeze/value", &squeezeValuePath[Side::LEFT]));
  CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/right/input/squeeze/value", &squeezeValuePath[Side::RIGHT]));
  CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/left/input/squeeze/force", &squeezeForcePath[Side::LEFT]));
  CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/right/input/squeeze/force", &squeezeForcePath[Side::RIGHT]));
  CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/left/input/squeeze/click", &squeezeClickPath[Side::LEFT]));
  CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/right/input/squeeze/click", &squeezeClickPath[Side::RIGHT]));
  CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/left/input/grip/pose", &posePath[Side::LEFT]));
  CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/right/input/grip/pose", &posePath[Side::RIGHT]));
  CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/left/output/haptic", &hapticPath[Side::LEFT]));
  CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/right/output/haptic", &hapticPath[Side::RIGHT]));
  CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/left/input/menu/click", &menuClickPath[Side::LEFT]));
  CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/right/input/menu/click", &menuClickPath[Side::RIGHT]));
  CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/left/input/b/click", &bClickPath[Side::LEFT]));
  CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/right/input/b/click", &bClickPath[Side::RIGHT]));
  CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/left/input/trigger/value", &triggerValuePath[Side::LEFT]));
  CHECK_XRCMD(xrStringToPath(m_instance, "/user/hand/right/input/trigger/value", &triggerValuePath[Side::RIGHT]));
  // Suggest bindings for KHR Simple.
  {
    XrPath khrSimpleInteractionProfilePath;
    CHECK_XRCMD(
      xrStringToPath(m_instance, "/interaction_profiles/khr/simple_controller", &khrSimpleInteractionProfilePath));
    std::vector<XrActionSuggestedBinding> bindings{ {// Fall back to a click input for the grab action.
                                                    {m_input.grabAction, selectPath[Side::LEFT]},
                                                    {m_input.grabAction, selectPath[Side::RIGHT]},
                                                    {m_input.poseAction, posePath[Side::LEFT]},
                                                    {m_input.poseAction, posePath[Side::RIGHT]},
                                                    {m_input.quitAction, menuClickPath[Side::LEFT]},
                                                    {m_input.quitAction, menuClickPath[Side::RIGHT]},
                                                    {m_input.vibrateAction, hapticPath[Side::LEFT]},
                                                    {m_input.vibrateAction, hapticPath[Side::RIGHT]}} };
    XrInteractionProfileSuggestedBinding suggestedBindings{ XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING };
    suggestedBindings.interactionProfile = khrSimpleInteractionProfilePath;
    suggestedBindings.suggestedBindings = bindings.data();
    suggestedBindings.countSuggestedBindings = (uint32_t)bindings.size();
    CHECK_XRCMD(xrSuggestInteractionProfileBindings(m_instance, &suggestedBindings));
  }
  // Suggest bindings for the Oculus Touch.
  {
    XrPath oculusTouchInteractionProfilePath;
    CHECK_XRCMD(
      xrStringToPath(m_instance, "/interaction_profiles/oculus/touch_controller", &oculusTouchInteractionProfilePath));
    std::vector<XrActionSuggestedBinding> bindings{ {{m_input.grabAction, squeezeValuePath[Side::LEFT]},
                                                    {m_input.grabAction, squeezeValuePath[Side::RIGHT]},
                                                    {m_input.poseAction, posePath[Side::LEFT]},
                                                    {m_input.poseAction, posePath[Side::RIGHT]},
                                                    {m_input.quitAction, menuClickPath[Side::LEFT]},
                                                    {m_input.vibrateAction, hapticPath[Side::LEFT]},
                                                    {m_input.vibrateAction, hapticPath[Side::RIGHT]}} };
    XrInteractionProfileSuggestedBinding suggestedBindings{ XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING };
    suggestedBindings.interactionProfile = oculusTouchInteractionProfilePath;
    suggestedBindings.suggestedBindings = bindings.data();
    suggestedBindings.countSuggestedBindings = (uint32_t)bindings.size();
    CHECK_XRCMD(xrSuggestInteractionProfileBindings(m_instance, &suggestedBindings));
  }
  // Suggest bindings for the Vive Controller.
  {
    XrPath viveControllerInteractionProfilePath;
    CHECK_XRCMD(
      xrStringToPath(m_instance, "/interaction_profiles/htc/vive_controller", &viveControllerInteractionProfilePath));
    std::vector<XrActionSuggestedBinding> bindings{ {{m_input.grabAction, triggerValuePath[Side::LEFT]},
                                                    {m_input.grabAction, triggerValuePath[Side::RIGHT]},
                                                    {m_input.poseAction, posePath[Side::LEFT]},
                                                    {m_input.poseAction, posePath[Side::RIGHT]},
                                                    {m_input.quitAction, menuClickPath[Side::LEFT]},
                                                    {m_input.quitAction, menuClickPath[Side::RIGHT]},
                                                    {m_input.vibrateAction, hapticPath[Side::LEFT]},
                                                    {m_input.vibrateAction, hapticPath[Side::RIGHT]}} };
    XrInteractionProfileSuggestedBinding suggestedBindings{ XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING };
    suggestedBindings.interactionProfile = viveControllerInteractionProfilePath;
    suggestedBindings.suggestedBindings = bindings.data();
    suggestedBindings.countSuggestedBindings = (uint32_t)bindings.size();
    CHECK_XRCMD(xrSuggestInteractionProfileBindings(m_instance, &suggestedBindings));
  }

  // Suggest bindings for the Valve Index Controller.
  {
    XrPath indexControllerInteractionProfilePath;
    CHECK_XRCMD(
      xrStringToPath(m_instance, "/interaction_profiles/valve/index_controller", &indexControllerInteractionProfilePath));
    std::vector<XrActionSuggestedBinding> bindings{ {{m_input.grabAction, squeezeForcePath[Side::LEFT]},
                                                    {m_input.grabAction, squeezeForcePath[Side::RIGHT]},
                                                    {m_input.poseAction, posePath[Side::LEFT]},
                                                    {m_input.poseAction, posePath[Side::RIGHT]},
                                                    {m_input.quitAction, bClickPath[Side::LEFT]},
                                                    {m_input.quitAction, bClickPath[Side::RIGHT]},
                                                    {m_input.vibrateAction, hapticPath[Side::LEFT]},
                                                    {m_input.vibrateAction, hapticPath[Side::RIGHT]}} };
    XrInteractionProfileSuggestedBinding suggestedBindings{ XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING };
    suggestedBindings.interactionProfile = indexControllerInteractionProfilePath;
    suggestedBindings.suggestedBindings = bindings.data();
    suggestedBindings.countSuggestedBindings = (uint32_t)bindings.size();
    CHECK_XRCMD(xrSuggestInteractionProfileBindings(m_instance, &suggestedBindings));
  }

  // Suggest bindings for the Microsoft Mixed Reality Motion Controller.
  {
    XrPath microsoftMixedRealityInteractionProfilePath;
    CHECK_XRCMD(xrStringToPath(m_instance, "/interaction_profiles/microsoft/motion_controller",
      &microsoftMixedRealityInteractionProfilePath));
    std::vector<XrActionSuggestedBinding> bindings{ {{m_input.grabAction, squeezeClickPath[Side::LEFT]},
                                                    {m_input.grabAction, squeezeClickPath[Side::RIGHT]},
                                                    {m_input.poseAction, posePath[Side::LEFT]},
                                                    {m_input.poseAction, posePath[Side::RIGHT]},
                                                    {m_input.quitAction, menuClickPath[Side::LEFT]},
                                                    {m_input.quitAction, menuClickPath[Side::RIGHT]},
                                                    {m_input.vibrateAction, hapticPath[Side::LEFT]},
                                                    {m_input.vibrateAction, hapticPath[Side::RIGHT]}} };
    XrInteractionProfileSuggestedBinding suggestedBindings{ XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING };
    suggestedBindings.interactionProfile = microsoftMixedRealityInteractionProfilePath;
    suggestedBindings.suggestedBindings = bindings.data();
    suggestedBindings.countSuggestedBindings = (uint32_t)bindings.size();
    CHECK_XRCMD(xrSuggestInteractionProfileBindings(m_instance, &suggestedBindings));
  }
  XrActionSpaceCreateInfo actionSpaceInfo{ XR_TYPE_ACTION_SPACE_CREATE_INFO };
  actionSpaceInfo.action = m_input.poseAction;
  actionSpaceInfo.poseInActionSpace.orientation.w = 1.f;
  actionSpaceInfo.subactionPath = m_input.handSubactionPath[Side::LEFT];
  CHECK_XRCMD(xrCreateActionSpace(m_session, &actionSpaceInfo, &m_input.handSpace[Side::LEFT]));
  actionSpaceInfo.subactionPath = m_input.handSubactionPath[Side::RIGHT];
  CHECK_XRCMD(xrCreateActionSpace(m_session, &actionSpaceInfo, &m_input.handSpace[Side::RIGHT]));

  XrSessionActionSetsAttachInfo attachInfo{ XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO };
  attachInfo.countActionSets = 1;
  attachInfo.actionSets = &m_input.actionSet;
  CHECK_XRCMD(xrAttachSessionActionSets(m_session, &attachInfo));
}

int64_t asdp::render::DisplayOpenXR::DisplayOpenXRImpl::OpenGLSelectColorSwapchainFormat(const std::vector<int64_t>& runtimeFormats)
{
  // List of supported color swapchain formats.
  constexpr int64_t SupportedColorSwapchainFormats[] = {
      GL_RGB10_A2,
      GL_RGBA16F,
      // The two below should only be used as a fallback, as they are linear color formats without enough bits for color
      // depth, thus leading to banding.
      GL_RGBA8,
      GL_RGBA8_SNORM,
  };

  auto swapchainFormatIt =
    std::find_first_of(runtimeFormats.begin(), runtimeFormats.end(), std::begin(SupportedColorSwapchainFormats),
      std::end(SupportedColorSwapchainFormats));
  if (swapchainFormatIt == runtimeFormats.end()) {
    THROW("No runtime swapchain format supported for color swapchain");
  }

  return *swapchainFormatIt;
}

std::vector<XrSwapchainImageBaseHeader*> asdp::render::DisplayOpenXR::DisplayOpenXRImpl::OpenGLAllocateSwapchainImageStructs(
  uint32_t capacity, const XrSwapchainCreateInfo& /*swapchainCreateInfo*/)
{
  // Allocate and initialize the buffer of image structs (must be sequential in memory for xrEnumerateSwapchainImages).
  // Return back an array of pointers to each swapchain image struct so the consumer doesn't need to know the type/size.
  std::vector<XrSwapchainImageOpenGLKHR> swapchainImageBuffer(capacity);
  std::vector<XrSwapchainImageBaseHeader*> swapchainImageBase;
  for (XrSwapchainImageOpenGLKHR& image : swapchainImageBuffer) {
    image.type = XR_TYPE_SWAPCHAIN_IMAGE_OPENGL_KHR;
    swapchainImageBase.push_back(reinterpret_cast<XrSwapchainImageBaseHeader*>(&image));
  }

  // Keep the buffer alive by moving it into the list of buffers.
  m_swapchainImageBuffers.push_back(std::move(swapchainImageBuffer));

  return swapchainImageBase;
}

void asdp::render::DisplayOpenXR::DisplayOpenXRImpl::OpenXRCreateSwapchains()
{
  // Create the swapchains for the views.
  CHECK(m_session != XR_NULL_HANDLE);
  CHECK(m_swapchains.empty());
  CHECK(m_configViews.empty());

  // Read graphics properties for preferred swapchain length and logging.
  XrSystemProperties systemProperties{ XR_TYPE_SYSTEM_PROPERTIES };
  CHECK_XRCMD(xrGetSystemProperties(m_instance, m_systemId, &systemProperties));

  // Log system properties.
  if (m_verbosity >= 1) {
    std::cout <<
      Fmt("System Properties: Name=%s VendorId=%d", systemProperties.systemName, systemProperties.vendorId)
      << std::endl;
    std::cout << Fmt("System Graphics Properties: MaxWidth=%d MaxHeight=%d MaxLayers=%d",
      systemProperties.graphicsProperties.maxSwapchainImageWidth,
      systemProperties.graphicsProperties.maxSwapchainImageHeight,
      systemProperties.graphicsProperties.maxLayerCount)
      << std::endl;
    std::cout << Fmt("System Tracking Properties: OrientationTracking=%s PositionTracking=%s",
      systemProperties.trackingProperties.orientationTracking == XR_TRUE ? "True" : "False",
      systemProperties.trackingProperties.positionTracking == XR_TRUE ? "True" : "False")
      << std::endl;
  }

  // Note: No other view configurations exist at the time this code was written. If this
  // condition is not met, the project will need to be audited to see how support should be
  // added.
  CHECK_MSG(m_viewConfigType == XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, "Unsupported view configuration type");

  // Query and cache view configuration views.
  uint32_t viewCount;
  CHECK_XRCMD(xrEnumerateViewConfigurationViews(m_instance, m_systemId, m_viewConfigType, 0, &viewCount, nullptr));
  m_configViews.resize(viewCount, { XR_TYPE_VIEW_CONFIGURATION_VIEW });
  CHECK_XRCMD(xrEnumerateViewConfigurationViews(m_instance, m_systemId, m_viewConfigType, viewCount, &viewCount,
    m_configViews.data()));

  // Create and cache view buffer for xrLocateViews later.
  m_views.resize(viewCount, { XR_TYPE_VIEW });

  // Create the swapchain and get the images.
  if (viewCount > 0) {
    // Select a swapchain format.
    uint32_t swapchainFormatCount;
    CHECK_XRCMD(xrEnumerateSwapchainFormats(m_session, 0, &swapchainFormatCount, nullptr));
    std::vector<int64_t> swapchainFormats(swapchainFormatCount);
    CHECK_XRCMD(xrEnumerateSwapchainFormats(m_session, (uint32_t)swapchainFormats.size(), &swapchainFormatCount,
      swapchainFormats.data()));
    CHECK(swapchainFormatCount == swapchainFormats.size());
    m_colorSwapchainFormat = OpenGLSelectColorSwapchainFormat(swapchainFormats);

    // Print swapchain formats and the selected one.
    {
      std::string swapchainFormatsString;
      for (int64_t format : swapchainFormats) {
        const bool selected = format == m_colorSwapchainFormat;
        swapchainFormatsString += " ";
        if (selected) {
          swapchainFormatsString += "[";
        }
        swapchainFormatsString += std::to_string(format);
        if (selected) {
          swapchainFormatsString += "]";
        }
      }
      if (m_verbosity >= 1) std::cout << Fmt("Swapchain Formats: %s", swapchainFormatsString.c_str()) << std::endl;
    }

    // Create a swapchain for each view.
    for (uint32_t i = 0; i < viewCount; i++) {
      const XrViewConfigurationView& vp = m_configViews[i];
      if (m_verbosity >= 1) {
        std::cout <<
          Fmt("Creating swapchain for view %d with dimensions Width=%d Height=%d SampleCount=%d", i,
            vp.recommendedImageRectWidth, vp.recommendedImageRectHeight, vp.recommendedSwapchainSampleCount)
          << std::endl;
      }

      // Create the swapchain.
      XrSwapchainCreateInfo swapchainCreateInfo{ XR_TYPE_SWAPCHAIN_CREATE_INFO };
      swapchainCreateInfo.arraySize = 1;
      swapchainCreateInfo.format = m_colorSwapchainFormat;
      swapchainCreateInfo.width = vp.recommendedImageRectWidth;
      swapchainCreateInfo.height = vp.recommendedImageRectHeight;
      swapchainCreateInfo.mipCount = 1;
      swapchainCreateInfo.faceCount = 1;
      swapchainCreateInfo.sampleCount = vp.recommendedSwapchainSampleCount;
      swapchainCreateInfo.usageFlags = XR_SWAPCHAIN_USAGE_SAMPLED_BIT | XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT;
      Swapchain swapchain;
      swapchain.width = swapchainCreateInfo.width;
      swapchain.height = swapchainCreateInfo.height;
      CHECK_XRCMD(xrCreateSwapchain(m_session, &swapchainCreateInfo, &swapchain.handle));

      m_swapchains.push_back(swapchain);

      uint32_t imageCount;
      CHECK_XRCMD(xrEnumerateSwapchainImages(swapchain.handle, 0, &imageCount, nullptr));
      // XXX This should really just return XrSwapchainImageBaseHeader*
      std::vector<XrSwapchainImageBaseHeader*> swapchainImages =
        OpenGLAllocateSwapchainImageStructs(imageCount, swapchainCreateInfo);
      CHECK_XRCMD(xrEnumerateSwapchainImages(swapchain.handle, imageCount, &imageCount, swapchainImages[0]));

      m_swapchainImages.insert(std::make_pair(swapchain.handle, std::move(swapchainImages)));
    }
  }
}

// Return event if one is available, otherwise return null.
XrEventDataBaseHeader* asdp::render::DisplayOpenXR::DisplayOpenXRImpl::OpenXRTryReadNextEvent()
{
  // It is sufficient to clear the just the XrEventDataBuffer header to
  // XR_TYPE_EVENT_DATA_BUFFER
  XrEventDataBaseHeader* baseHeader = reinterpret_cast<XrEventDataBaseHeader*>(&m_eventDataBuffer);
  *baseHeader = { XR_TYPE_EVENT_DATA_BUFFER };
  const XrResult xr = xrPollEvent(m_instance, &m_eventDataBuffer);
  if (xr == XR_SUCCESS) {
    if (baseHeader->type == XR_TYPE_EVENT_DATA_EVENTS_LOST) {
      const XrEventDataEventsLost* const eventsLost = reinterpret_cast<const XrEventDataEventsLost*>(baseHeader);
      if (m_verbosity > 0) std::cerr << Fmt("%d events lost", eventsLost) << std::endl;
    }

    return baseHeader;
  }
  if (xr == XR_EVENT_UNAVAILABLE) {
    return nullptr;
  }
  THROW_XR(xr, "xrPollEvent");
}

void asdp::render::DisplayOpenXR::DisplayOpenXRImpl::OpenXRHandleSessionStateChangedEvent(
  const XrEventDataSessionStateChanged& stateChangedEvent, bool* exitRenderLoop,
  bool* requestRestart)
{
  const XrSessionState oldState = m_sessionState;
  m_sessionState = stateChangedEvent.state;

  if (m_verbosity >= 1) {
    std::cout << Fmt("XrEventDataSessionStateChanged: state %s->%s session=%lld time=%lld", to_string(oldState),
      to_string(m_sessionState), stateChangedEvent.session, stateChangedEvent.time)
      << std::endl;
  }

  if ((stateChangedEvent.session != XR_NULL_HANDLE) && (stateChangedEvent.session != m_session)) {
    std::cerr << "XrEventDataSessionStateChanged for unknown session" << std::endl;
    return;
  }

  switch (m_sessionState) {
  case XR_SESSION_STATE_READY: {
    CHECK(m_session != XR_NULL_HANDLE);
    XrSessionBeginInfo sessionBeginInfo{ XR_TYPE_SESSION_BEGIN_INFO };
    sessionBeginInfo.primaryViewConfigurationType = m_viewConfigType;
    CHECK_XRCMD(xrBeginSession(m_session, &sessionBeginInfo));
    m_sessionRunning = true;
    break;
  }
  case XR_SESSION_STATE_STOPPING: {
    CHECK(m_session != XR_NULL_HANDLE);
    m_sessionRunning = false;
    CHECK_XRCMD(xrEndSession(m_session))
      break;
  }
  case XR_SESSION_STATE_EXITING: {
    *exitRenderLoop = true;
    // Do not attempt to restart because user closed this session.
    *requestRestart = false;
    break;
  }
  case XR_SESSION_STATE_LOSS_PENDING: {
    *exitRenderLoop = true;
    // Poll for a new instance.
    *requestRestart = true;
    break;
  }
  default:
    break;
  }
}
void asdp::render::DisplayOpenXR::DisplayOpenXRImpl::OpenXRPollEvents(bool* exitRenderLoop, bool* requestRestart)
{
  *exitRenderLoop = *requestRestart = false;

  // Process all pending messages.
  while (const XrEventDataBaseHeader* event = OpenXRTryReadNextEvent()) {
    switch (event->type) {
    case XR_TYPE_EVENT_DATA_INSTANCE_LOSS_PENDING: {
      const auto& instanceLossPending = *reinterpret_cast<const XrEventDataInstanceLossPending*>(event);
      if (m_verbosity > 0) std::cerr << Fmt("XrEventDataInstanceLossPending by %lld", instanceLossPending.lossTime) << std::endl;
      *exitRenderLoop = true;
      *requestRestart = true;
      return;
    }
    case XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED: {
      auto sessionStateChangedEvent = *reinterpret_cast<const XrEventDataSessionStateChanged*>(event);
      OpenXRHandleSessionStateChangedEvent(sessionStateChangedEvent, exitRenderLoop, requestRestart);
      break;
    }
    case XR_TYPE_EVENT_DATA_INTERACTION_PROFILE_CHANGED:
      break;
    case XR_TYPE_EVENT_DATA_REFERENCE_SPACE_CHANGE_PENDING:
    default: {
      if (m_verbosity >= 2) std::cout << Fmt("Ignoring event type %d", event->type) << std::endl;
      break;
    }
    }
  }
}

void asdp::render::DisplayOpenXR::DisplayOpenXRImpl::OpenXRPollActions()
{
  m_input.handActive = { XR_FALSE, XR_FALSE };

  // Sync actions
  const XrActiveActionSet activeActionSet{ m_input.actionSet, XR_NULL_PATH };
  XrActionsSyncInfo syncInfo{ XR_TYPE_ACTIONS_SYNC_INFO };
  syncInfo.countActiveActionSets = 1;
  syncInfo.activeActionSets = &activeActionSet;
  CHECK_XRCMD(xrSyncActions(m_session, &syncInfo));

  // Get pose and grab action state and start haptic vibrate when hand is 90% squeezed.
  bool isGrabbed = false;
  for (auto hand : { Side::LEFT, Side::RIGHT }) {
    XrActionStateGetInfo getInfo{ XR_TYPE_ACTION_STATE_GET_INFO };
    getInfo.action = m_input.grabAction;
    getInfo.subactionPath = m_input.handSubactionPath[hand];

    XrActionStateFloat grabValue{ XR_TYPE_ACTION_STATE_FLOAT };
    CHECK_XRCMD(xrGetActionStateFloat(m_session, &getInfo, &grabValue));
    if (grabValue.isActive == XR_TRUE) {
      // Scale the rendered hand by 1.0f (open) to 0.5f (fully squeezed).
      m_input.handScale[hand] = 1.0f - 0.5f * grabValue.currentState;
      if (grabValue.currentState > 0.9f) {
        XrHapticVibration vibration{ XR_TYPE_HAPTIC_VIBRATION };
        vibration.amplitude = 0.5;
        vibration.duration = XR_MIN_HAPTIC_DURATION;
        vibration.frequency = XR_FREQUENCY_UNSPECIFIED;

        XrHapticActionInfo hapticActionInfo{ XR_TYPE_HAPTIC_ACTION_INFO };
        hapticActionInfo.action = m_input.vibrateAction;
        hapticActionInfo.subactionPath = m_input.handSubactionPath[hand];
        CHECK_XRCMD(xrApplyHapticFeedback(m_session, &hapticActionInfo, (XrHapticBaseHeader*)&vibration));

        isGrabbed = true;
        if (!m_lastGrabbedState) {
          // Toggle the play/pause state.
          if (m_display->m_eventHandlers && m_display->m_eventHandlers->ChangePlayPause) {
            m_display->m_eventHandlers->ChangePlayPause(!m_display->m_nowPlaying, m_display->m_userData);
          }
        }
      }
    }

    getInfo.action = m_input.poseAction;
    XrActionStatePose poseState{ XR_TYPE_ACTION_STATE_POSE };
    CHECK_XRCMD(xrGetActionStatePose(m_session, &getInfo, &poseState));
    m_input.handActive[hand] = poseState.isActive;
  }
  m_lastGrabbedState = isGrabbed;

  // There were no subaction paths specified for the quit action, because we don't care which hand did it.
  XrActionStateGetInfo getInfo{ XR_TYPE_ACTION_STATE_GET_INFO, nullptr, m_input.quitAction, XR_NULL_PATH };
  XrActionStateBoolean quitValue{ XR_TYPE_ACTION_STATE_BOOLEAN };
  CHECK_XRCMD(xrGetActionStateBoolean(m_session, &getInfo, &quitValue));
  if ((quitValue.isActive == XR_TRUE) && (quitValue.changedSinceLastSync == XR_TRUE) && (quitValue.currentState == XR_TRUE)) {
    CHECK_XRCMD(xrRequestExitSession(m_session));
    m_display->m_composite.reset();
    m_display->m_status = "Done";
  }
}

bool asdp::render::DisplayOpenXR::DisplayOpenXRImpl::OpenXRRenderLayer(XrTime predictedDisplayTime,
  std::vector<XrCompositionLayerProjectionView>& projectionLayerViews, XrCompositionLayerProjection& layer)
{
  XrResult res;

  XrViewState viewState{ XR_TYPE_VIEW_STATE };
  uint32_t viewCapacityInput = (uint32_t)m_views.size();
  uint32_t viewCountOutput;

  XrViewLocateInfo viewLocateInfo{ XR_TYPE_VIEW_LOCATE_INFO };
  viewLocateInfo.viewConfigurationType = m_viewConfigType;
  viewLocateInfo.displayTime = predictedDisplayTime;
  viewLocateInfo.space = m_appSpace;

  res = xrLocateViews(m_session, &viewLocateInfo, &viewState, viewCapacityInput, &viewCountOutput, m_views.data());
  CHECK_XRRESULT(res, "xrLocateViews");
  if ((viewState.viewStateFlags & XR_VIEW_STATE_POSITION_VALID_BIT) == 0 ||
    (viewState.viewStateFlags & XR_VIEW_STATE_ORIENTATION_VALID_BIT) == 0) {
    return false;  // There is no valid tracking poses for the views.
  }

  CHECK(viewCountOutput == viewCapacityInput);
  CHECK(viewCountOutput == m_configViews.size());
  CHECK(viewCountOutput == m_swapchains.size());

  projectionLayerViews.resize(viewCountOutput);

  // Describe all of the views and then render them.  This replaces the OpenXRRenderView() function
  // from the original sample with using the Composite to render.
  std::vector<ViewRenderInfo> viewRenderInfos;

  // Grab the swapchains and fill in the viewRenderInfos.
  for (uint32_t i = 0; i < viewCountOutput; i++) {
    // Fill in the information on the projection layer views.
    projectionLayerViews[i] = { XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW };
    projectionLayerViews[i].pose = m_views[i].pose;
    projectionLayerViews[i].fov = m_views[i].fov;
    projectionLayerViews[i].subImage.swapchain = m_swapchains[i].handle;
    projectionLayerViews[i].subImage.imageRect.offset = { 0, 0 };
    projectionLayerViews[i].subImage.imageRect.extent = { m_swapchains[i].width, m_swapchains[i].height };

    // Acquire a swapchain image for the current view.
    XrSwapchainImageAcquireInfo acquireInfo{ XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO };
    uint32_t swapchainImageIndex;
    CHECK_XRCMD(xrAcquireSwapchainImage(m_swapchains[i].handle, &acquireInfo, &swapchainImageIndex));
    XrSwapchainImageWaitInfo waitInfo{ XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO };
    waitInfo.timeout = XR_INFINITE_DURATION;
    CHECK_XRCMD(xrWaitSwapchainImage(m_swapchains[i].handle, &waitInfo));

    // Convert the orientation to helicopter space by rotating -90 degrees around the x-axis,
    // doing the inverse rotation on the other side.
    glm::quat quat(m_views[i].pose.orientation.w, m_views[i].pose.orientation.x,
      m_views[i].pose.orientation.y, m_views[i].pose.orientation.z);
    float constexpr angle = glm::radians(-90.0f);
    glm::vec3 axis = glm::vec3(1.0f, 0.0f, 0.0f);
    glm::quat rotationQuat = glm::angleAxis(angle, axis);
    glm::quat inverseRotationQuat = glm::angleAxis(-angle, axis);
    quat = inverseRotationQuat * quat * rotationQuat;

    // Construct the ViewRenderInfo for the current view and push it onto the vector.
    ViewRenderInfo vri;
    vri.viewpoint[0] = m_views[i].pose.position.x + m_display->m_viewpointOffset[0];
    vri.viewpoint[1] = m_views[i].pose.position.y + m_display->m_viewpointOffset[1];
    vri.viewpoint[2] = m_views[i].pose.position.z + m_display->m_viewpointOffset[2];
    vri.orientation[0] = quat.w;
    vri.orientation[1] = quat.x;
    vri.orientation[2] = quat.y;
    vri.orientation[3] = quat.z;
    vri.leftHalfFOV = glm::degrees(m_views[i].fov.angleLeft);
    vri.rightHalfFOV = glm::degrees(m_views[i].fov.angleRight);
    vri.topHalfFOV = glm::degrees(m_views[i].fov.angleUp);
    vri.bottomHalfFOV = glm::degrees(m_views[i].fov.angleDown);
    // We leave nearClip and farClip at their default values
    vri.frameBuffer = m_swapchainFramebuffer;
    vri.colorBuffer = reinterpret_cast<const XrSwapchainImageOpenGLKHR*>(
      m_swapchainImages[m_swapchains[i].handle][swapchainImageIndex])->image;
    vri.depthBuffer = OpenGLGetDepthTexture(vri.colorBuffer);
    vri.x = projectionLayerViews[i].subImage.imageRect.offset.x;
    vri.y = projectionLayerViews[i].subImage.imageRect.offset.y;
    vri.width = projectionLayerViews[i].subImage.imageRect.extent.width;
    vri.height = projectionLayerViews[i].subImage.imageRect.extent.height;
    
    //======================================
    // Added by Sang Yoon to adjust binocular disparity between two images for the left and right eyes in the cylindrical projection.
    // It is assumed that the Varjo XR-4 HMD is used (between 40 PPD and 51 PPD with a horizontal FOV of about 105 degrees per eye).
    // Without this adjustment, it is hard to stereoscopically fuse the images rendered using the cylindrical projection,
    // since the binocular disparity between the left and right images is too large.
    // The amount of binocular disparity adjustment is dependent on the resolution (proportional to pixel size or PPD).
    if (m_display->m_composite->m_CP_enabled) {
      float shift_amount = 225.0 * vri.width / 5184 + 0.5;
      if (i == 0)
          vri.x = (int)shift_amount;
      else
          vri.x = -(int)shift_amount;
    }
    //======================================

    viewRenderInfos.push_back(vri);
  }

  // Find out the delay from now until the predicted display time for the center of the frame.
  // Do this by converting the predicted display time to a Windows performance counter time and then
  // subtracting the current time.
  Time time;
  std::shared_ptr<Timer> timer;
  Status status = m_display->m_client->GetTimer(timer);
  status = timer->GetCoreTime(time);
  if (status != OKAY) {
    return false;
  }
  if (!m_display->m_replaying) {
#ifdef _WIN32
    // Check that the extension is available before calling it.
    // then figure out how far into the future we are and add that to the present time.
    if (m_xrConvertTimeToWin32PerformanceCounterKHR) {
      LARGE_INTEGER counterNow;
      if (!QueryPerformanceCounter(&counterNow)) {
        std::cerr << "OpenXRRenderLayer(): Failed to read performance counter" << std::endl;
        return false;
      }
      LARGE_INTEGER counterThen;
      XrResult result = m_xrConvertTimeToWin32PerformanceCounterKHR(m_instance, predictedDisplayTime, &counterThen);
      if (result != XR_SUCCESS) {
        std::cerr << "OpenXRRenderLayer(): Failed to convert OpenXR time to Windows performance counter: " << result << std::endl;
        return false;
      }
      size_t nanoseconds = 0;
      if (counterThen.QuadPart > counterNow.QuadPart) {
        LONGLONG diff = counterThen.QuadPart - counterNow.QuadPart;
        LARGE_INTEGER frequency;
        if (!QueryPerformanceFrequency(&frequency)) {
          std::cerr << "OpenXRRenderLayer(): Failed to read performance frequency" << std::endl;
          return false;
        }
        double seconds = static_cast<double>(diff) / static_cast<double>(frequency.QuadPart);
        Time dt;
        dt.seconds = static_cast<uint64_t>(seconds);
        dt.microseconds = static_cast<uint32_t>((seconds - dt.seconds) * 1e6);
        // On the HTC Vive OpenXR implementation, this returns a time many seconds into the future, it is probably returning
        // the time since the epoch rather than the time since the start of the performance timer (boot).
        if (dt.seconds == 0) {
          time += dt;
          //std::cout << "XXX dt = " << dt.seconds << "s " << dt.microseconds << "us" << std::endl;
        }
        else {
          static bool warned = false;
          if (!warned) {
            std::cerr << "OpenXRRenderLayer(): Time prediction more than a second ahead, probably a bug in the OpenXR runtime; not predicting." << std::endl;
            warned = true;
          }
        }
      }
        }
#else
    std::cerr << "Time prediction not implemented for this platform" << std::endl;
#endif
  }

  // Trigger the cameras, saying that we need the data now. The base class will handle offsetting
  // by the specified transmission/processing time as passed to its constructor by the client.
  m_display->TriggerCameras(std::chrono::steady_clock::now());

  // Record the render start time, if we have a place to put it.
  if (m_display->m_timingInfo) {
    m_display->m_timingInfo->renderStartTimes.push_back(std::chrono::steady_clock::now());
  }

  /// Render the requested views at the predicted scan-out time, pausing if we're paused.
  if (m_pauseTime) {
    time = *m_pauseTime;
  }
  m_display->m_composite->Render(time, viewRenderInfos);

  /*
  // Swap our window every other eye for RenderDoc
  /// @todo Not needed until we're drawing into that window...
  static int everyOther = 0;
  if ((everyOther++ & 1) != 0) {
    glfwSwapBuffers(m_display->m_impl->m_contextWindow);
  }
  */

  /// Release the swapchain images after rendering.
  for (uint32_t i = 0; i < viewCountOutput; i++) {
    XrSwapchainImageReleaseInfo releaseInfo{ XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO };
    CHECK_XRCMD(xrReleaseSwapchainImage(m_swapchains[i].handle, &releaseInfo));
  }

  layer.space = m_appSpace;
  layer.viewCount = (uint32_t)projectionLayerViews.size();
  layer.views = projectionLayerViews.data();
  return true;
}

void asdp::render::DisplayOpenXR::DisplayOpenXRImpl::OpenXRRenderFrame()
{
  CHECK(m_session != XR_NULL_HANDLE);

  // Wait for the time to compute depths for the next frame.
  /// @todo We don't have an easy way to do that, so we compute it right away.

  // Call the depth handler if we have one.
  if (m_display->m_eventHandlers && m_display->m_eventHandlers->ComputeDepth) {
    // Compute the time of the middle of next frame and pass it to the handler.
    // We estimate this by taking the current time and adding the time to the middle of next frame.
    std::chrono::steady_clock::time_point renderTime = std::chrono::steady_clock::now();
    renderTime += std::chrono::nanoseconds(m_frameDurationNS * 3 / 2);
    Time coreTime;
    Status status = m_display->m_timer->GetCoreTime(coreTime, renderTime);
    if (m_pauseTime) {
      coreTime = *m_pauseTime;
    }
    m_display->m_eventHandlers->ComputeDepth(coreTime, m_display->m_userData);
  }

  // Wait for the time to render the next frame
  XrFrameWaitInfo frameWaitInfo{ XR_TYPE_FRAME_WAIT_INFO };
  XrFrameState frameState{ XR_TYPE_FRAME_STATE };
  CHECK_XRCMD(xrWaitFrame(m_session, &frameWaitInfo, &frameState));

  XrFrameBeginInfo frameBeginInfo{ XR_TYPE_FRAME_BEGIN_INFO };
  CHECK_XRCMD(xrBeginFrame(m_session, &frameBeginInfo));

  std::vector<XrCompositionLayerBaseHeader*> layers;
  XrCompositionLayerProjection layer{ XR_TYPE_COMPOSITION_LAYER_PROJECTION };
  std::vector<XrCompositionLayerProjectionView> projectionLayerViews;
  if (frameState.shouldRender == XR_TRUE) {
    // Fill in the display period and then render
    m_frameDurationNS = frameState.predictedDisplayPeriod;
    if (OpenXRRenderLayer(frameState.predictedDisplayTime, projectionLayerViews, layer)) {
      layers.push_back(reinterpret_cast<XrCompositionLayerBaseHeader*>(&layer));
    }
  }

  // Record the render submission time, if we have a place to put it.
  if (m_display->m_timingInfo) {
    m_display->m_timingInfo->renderSubmitTimes.push_back(std::chrono::steady_clock::now());
  }

  XrFrameEndInfo frameEndInfo{ XR_TYPE_FRAME_END_INFO };
  frameEndInfo.displayTime = frameState.predictedDisplayTime;
  frameEndInfo.environmentBlendMode = m_environmentBlendMode;
  frameEndInfo.layerCount = (uint32_t)layers.size();
  frameEndInfo.layers = layers.data();
  CHECK_XRCMD(xrEndFrame(m_session, &frameEndInfo));
}
void asdp::render::DisplayOpenXR::DisplayOpenXRImpl::OpenGLInitializeResources()
{
  glGenFramebuffers(1, &m_swapchainFramebuffer);
}

uint32_t asdp::render::DisplayOpenXR::DisplayOpenXRImpl::OpenGLGetDepthTexture(uint32_t colorTexture)
{
  // If a depth-stencil view has already been created for this back-buffer, use it.
  auto depthBufferIt = m_colorToDepthMap.find(colorTexture);
  if (depthBufferIt != m_colorToDepthMap.end()) {
    return depthBufferIt->second;
  }

  // This back-buffer has no corresponding depth-stencil texture, so create one with matching dimensions.

  GLint width;
  GLint height;
  glBindTexture(GL_TEXTURE_2D, colorTexture);
  glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &width);
  glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_HEIGHT, &height);

  uint32_t depthTexture;
  glGenTextures(1, &depthTexture);
  glBindTexture(GL_TEXTURE_2D, depthTexture);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32, width, height, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
  glBindTexture(GL_TEXTURE_2D, 0);

  m_colorToDepthMap.insert(std::make_pair(colorTexture, depthTexture));

  return depthTexture;
}

void asdp::render::DisplayOpenXR::DisplayOpenXRImpl::OpenGLTearDown()
{
  if (m_swapchainFramebuffer != 0) {
    glDeleteFramebuffers(1, &m_swapchainFramebuffer);
  }

  for (auto& colorToDepth : m_colorToDepthMap) {
    if (colorToDepth.second != 0) {
      glDeleteTextures(1, &colorToDepth.second);
    }
  }
}

void asdp::render::DisplayOpenXR::DisplayOpenXRImpl::OpenXRTearDown()
{
  OpenGLTearDown();

  if (m_input.actionSet != XR_NULL_HANDLE) {
    for (auto hand : { Side::LEFT, Side::RIGHT }) {
      xrDestroySpace(m_input.handSpace[hand]);
    }
    xrDestroyActionSet(m_input.actionSet);
  }

  for (Swapchain swapchain : m_swapchains) {
    xrDestroySwapchain(swapchain.handle);
  }

  if (m_appSpace != XR_NULL_HANDLE) {
    xrDestroySpace(m_appSpace);
  }

  if (m_session != XR_NULL_HANDLE) {
    xrDestroySession(m_session);
  }

  if (m_instance != XR_NULL_HANDLE) {
    xrDestroyInstance(m_instance);
  }

#ifdef XR_USE_PLATFORM_WIN32
  CoUninitialize();
#endif
}

DisplayOpenXR::DisplayOpenXR(std::shared_ptr<Composite> composite, Display* sharedWindow,
    std::shared_ptr<CoreClient> client, uint8_t triggerID, uint32_t triggerAheadMicroseconds,
    uint32_t depthAheadMicroseconds, std::array<float, 3> viewpointOffset,
    uint32_t renderAheadMicroseconds, int verbosity,
    std::shared_ptr<EventHandlers> handlers, void* userData,
    RenderTimingInfo* timingInfo, bool replaying)
  : Display(composite, client, triggerID, triggerAheadMicroseconds, depthAheadMicroseconds, viewpointOffset, handlers, userData)
  , m_timingInfo(timingInfo)
  , m_replaying(replaying)
{
  m_impl = std::make_unique<DisplayOpenXRImpl>(this);
  m_impl->m_verbosity = verbosity;

  // Start the rendering thread.
  m_displayThread = std::thread(&DisplayOpenXR::DisplayThread, this, sharedWindow, renderAheadMicroseconds);

  // Wait until either the context is ready or there has been a failure so that the
  // constructor does not return before the rendering thread is ready.
  while (!Display::m_impl->m_contextAvailable && (m_status == "")) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void DisplayOpenXR::SetNowPlaying(bool nowPlaying)
{
  // Call the parent-class method to set the now-playing state.
  Display::SetNowPlaying(nowPlaying);

  // Set the pause time based on whether we are now playing so that
  // we don't extrapolate forward in time while paused.
  if (!m_nowPlaying) {
    m_impl->m_pauseTime = std::make_unique<Time>();
    m_timer->GetCoreTime(*m_impl->m_pauseTime, std::chrono::steady_clock::now());
  } else {
    m_impl->m_pauseTime.reset();
  }
}

DisplayOpenXR::~DisplayOpenXR()
{
  // Make sure we're done with our rendering state and then clean up.
  Quit();
  m_impl.reset();
}

void DisplayOpenXR::DisplayThread(Display* sharedWindow, uint32_t renderAheadMicroseconds)
{
  bool requestRestart = false;
  do {

    /// Create things that we need for rendering.
    try {
      m_impl->OpenXRCreateInstance();
      m_impl->OpenXRInitializeSystem(sharedWindow);
      m_impl->OpenXRInitializeSession();
      m_impl->OpenXRCreateSwapchains();
    } catch (const std::exception& e) {
      m_status = "DisplayOpenXR::DisplayThread(): " + std::string(e.what());
    }

    // After we're done with the context for set-up and have released it, indicate that the context is available
    // for borrowing.
    glFinish();
    Display::m_impl->m_contextAvailable = true;

    while (m_status.empty()) {
      bool exitRenderLoop = false;
      try {
        m_impl->OpenXRPollEvents(&exitRenderLoop, &requestRestart);
      } catch (const std::exception& e) {
        m_status = "DisplayOpenXR::DisplayThread(): " + std::string(e.what());
        continue;
      }
      if (exitRenderLoop) {
        break;
      }

      /// Handle any actions and render the frame

      if (m_impl->m_sessionRunning) {
        try {
          m_impl->OpenXRPollActions();
          m_impl->OpenXRRenderFrame();
        } catch (const std::exception& e) {
          m_status = "DisplayOpenXR::DisplayThread(): " + std::string(e.what());
        }
      } else {
        // Throttle loop since xrWaitFrame won't be called.
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
      }
    }

    /// Clean up
    try {
      m_impl->OpenXRTearDown();
    } catch (const std::exception& e) {
      m_status = "DisplayOpenXR::DisplayThread(): " + std::string(e.what());
    }

  } while (m_status.empty() && requestRestart);
}

#else // USE_OPENXR

/// @brief Implementation details for the DisplayOpenXR class to hide #includes from users of the DisplayOpenXR class.
class asdp::render::DisplayOpenXR::DisplayOpenXRImpl {
public:
};

DisplayOpenXR::DisplayOpenXR(std::shared_ptr<Composite> composite, Display* sharedWindow,
    std::shared_ptr<CoreClient> client, uint8_t triggerID, uint32_t triggerAheadMicroseconds,
    uint32_t depthAheadMicroseconds, std::array<float, 3> viewpointOffset,
    uint32_t renderAheadMicroseconds, int verbosity,
    std::shared_ptr<EventHandlers> handlers, void* userData,
    RenderTimingInfo* timingInfo, bool replaying)
  : Display(composite, client, triggerID, triggerAheadMicroseconds, depthAheadMicroseconds, viewpointOffset, handlers, userData)
  , m_timingInfo(timingInfo)
  , m_replaying(replaying)
{
  m_status = "OpenXR is not compiled in.";
}

void DisplayOpenXR::SetNowPlaying(bool nowPlaying)
{
}

DisplayOpenXR::~DisplayOpenXR()
{
}

void DisplayOpenXR::DisplayThread(Display* sharedWindow, uint32_t renderAheadMicroseconds)
{
}

#endif // USE_OPENXR

//==============================================================================
// Structures and methods for DisplayXSight class.

/// @brief Implementation details for the DisplayXSight class to hide #includes from users of the DisplayXSight class.
class asdp::render::DisplayXSight::DisplayXSightImpl {
public:
  /// Horizontal field of view in degrees.
  float m_horizontalFOVDegrees{ 90.0f };

  /// Views to be rendered.
  std::vector<asdp::render::ViewRenderInfo> m_views;

  /// Time point to start rendering the next frame.
  std::chrono::steady_clock::time_point m_nextRenderTime;

  /// Time point to start computing the depth for the next frame;
  std::chrono::steady_clock::time_point m_nextDepthTime;

  /// Time point in the middle of the next frame we will render.
  std::chrono::steady_clock::time_point m_nextFrameTime;

  /// Frame buffer object that we will render the full-sized image into.
  GLuint m_framebuffer{ 0 };

  /// Color buffer that we will render the full-sized image into.
  GLuint m_colorBuffer{ 0 };

  /// Depth buffer that we will render the full-sized image into.
  GLuint m_depthBuffer{ 0 };

  //======================================
  // For the X-sight helmet display, added by Sang Yoon to add key mappings

  /// Whether the space bar was pressed during the last loop, used to toggle play/pause.
  bool m_spacePressed = false;

  /// Whether or no the 'd' key was pressed during the last loop, used to toggle depth computation.
  bool m_dPressed = false;
  bool m_displayingDepth = false;
  //======================================
};

DisplayXSight::DisplayXSight(std::string NICName, std::shared_ptr<Composite> composite, Display* sharedWindow,
  std::shared_ptr<CoreClient> client, uint8_t triggerID, uint32_t triggerAheadMicroseconds,
  uint32_t depthAheadMicroseconds, std::array<float, 3> viewpointOffset,
  uint32_t renderAheadMicroseconds,
  std::shared_ptr<EventHandlers> handlers, void* userData,
  RenderTimingInfo* timingInfo, bool replaying,
  int desiredDisplay,
  int desiredWidth, int desiredHeight, float fps,
  float horizontalFOVDegrees
  )
  : Display(composite, client, triggerID, triggerAheadMicroseconds, depthAheadMicroseconds, viewpointOffset, handlers, userData)
  , m_NICName(NICName)
  , m_timingInfo(timingInfo)
  , m_replaying(replaying)
  , m_impl(new DisplayXSightImpl)
{
  // Check our parameters.
  if ((desiredWidth <= 0) || (desiredWidth % 2 != 0) || (desiredHeight <= 0) || (horizontalFOVDegrees <= 0.0f)) {
    m_status = "Invalid window size or field of view";
    return;
  }

  // Store info from the constructor.
  m_impl->m_horizontalFOVDegrees = horizontalFOVDegrees;

  // Construct a single view to be used.  We base is on the requested window size and we compute a
  // field of view that is the requested horizontal and the correct aspect ratio vertical.
  ViewRenderInfo view;
  SetViewportSizeAndFOVs(view, desiredWidth, desiredHeight);
  view.viewpoint = m_viewpointOffset;
  m_impl->m_views.push_back(view);

  // Start the rendering thread.
  m_displayThread = std::thread(&DisplayXSight::DisplayThread, this,
    fps, renderAheadMicroseconds,
    desiredWidth, desiredHeight,
    sharedWindow, desiredDisplay);

  // Wait until either the context is ready or there has been a failure so that the
  // constructor does not return before the rendering thread is ready.
  while (!Display::m_impl->m_contextAvailable && (m_status == "")) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

DisplayXSight::~DisplayXSight()
{
  // Make sure we're done with our rendering state and then clean up.
  Quit();
  m_impl.reset();
}

void DisplayXSight::SetViewportSizeAndFOVs(ViewRenderInfo& viewInfo, int width, int height)
{
  if (m_impl == nullptr) {
    return;
  }
  if ((width == 0) || (height == 0)) {
    glfwGetWindowSize(Display::m_impl->m_window, &width, &height);
    // NOTE: The final image packs two monochrome pixels horizontally into a single
    // color pixel -- the rendered width is twice the final output width.
    viewInfo.width = 2 * width;
    viewInfo.height = height;
  }
  viewInfo.leftHalfFOV = -m_impl->m_horizontalFOVDegrees / 2.0f;
  viewInfo.rightHalfFOV = m_impl->m_horizontalFOVDegrees / 2.0f;

  // The vertical field of view is based on the aspect ratio of the window.  But the aspect ratio
  // is the in-plane width divided by the in-plane height.  The horizontal and vertical fields of view
  // are based on the tangents.
  double aspectRatio = static_cast<double>(viewInfo.height) / static_cast<double>(viewInfo.width);
  double halfWidth = tan(glm::radians(m_impl->m_horizontalFOVDegrees / 2.0));
  double halfHeight = halfWidth * aspectRatio;
  double halfAngle = glm::degrees(atan(halfHeight));
  viewInfo.bottomHalfFOV = -halfAngle;
  viewInfo.topHalfFOV = halfAngle;
}

/// @brief Embeds a 4-byte entity into four RGB pixels of an image.
/// @param dataPtr Pointer to the 4-byte entity to embed (points to first byte of little-endian value).
/// This must be type-cast into a char* to allow byte-wise manipulation.
/// @param firstRGBByte Pointer to the first byte of the first of four RGB triads to embed the data into.
static void EmbedField(char const* dataPtr, uint8_t* firstRGBByte)
{
  // The 4-byte entity (float or int) is embedded into four pixels of the image according to the
  // specification in "Xsight_Rack.pdf".  The data is stored from least-significant byte to most
  // and each byte is encoded into a single RGB triad. The encoding makes the Red byte zero and encodes
  // the value in Green and Blue. The lower nybble of each byte has the value 8 and the upper is mapped
  // with the most-significanly nybble of the encoded byte in the upper nybble of Blue and the least-significant
  // nybble in the upper nybble of Green.
  // NOTE: This assumes that we are running on a little-endian machine.
  for (int i = 0; i < 4; i++) {
    uint8_t byte = dataPtr[i];
    firstRGBByte[i * 3 + 0] = 0x08;                               // Red is ignored.
    firstRGBByte[i * 3 + 1] = 0x8 | ((byte & 0x0F) << 4);         // Green is the lower nybble.
    firstRGBByte[i * 3 + 2] = 0x8 | (((byte >> 4) & 0x0F) << 4);  // Blue is the upper nybble.
  }
}

static void EmbedLOSData(float azimuth, float elevation, float roll, uint32_t time, std::vector<uint8_t>& lineData)
{
  // Embed the azimuth, elevation, and roll into the first line of the image.
  // The azimuth is in the first 4 bytes, the elevation in the next 4 bytes, the roll in the next 4 bytes,
  // and the time in the last 4 bytes.
  EmbedField(reinterpret_cast<char const*>(&azimuth), lineData.data());
  EmbedField(reinterpret_cast<char const*>(&elevation), lineData.data() + 12);
  EmbedField(reinterpret_cast<char const*>(&roll), lineData.data() + 24);
  EmbedField(reinterpret_cast<char const*>(&time), lineData.data() + 36);
  // The list entry is a constant check value.
  const unsigned char checkData[] = { 0xbb, 0x44, 0xaa, 0x55 };
  EmbedField(reinterpret_cast<char const*>(checkData), lineData.data() + 48);
}

void DisplayXSight::DisplayThread(
  float fps, uint32_t renderAheadMicroseconds,
  int desiredWidth, int desiredHeight,
  Display* sharedWindow,
  int desiredDisplay)
{
  {
    {
      // Hold the window mutex so that only one window can be created at a time.
      std::lock_guard<std::mutex> windowLock(m_windowMutex);

      // Set the window to be visible.
      glfwWindowHint(GLFW_VISIBLE, true);

      // Don't use sRGB -- we need to put in specific pixel values with the CompositeLineRawData.
      glfwWindowHint(GLFW_SRGB_CAPABLE, GLFW_FALSE);

      // Tell it not to iconify full-screen windows that lose focus.
      glfwWindowHint(GLFW_AUTO_ICONIFY, GLFW_FALSE);

      // Create a windowed mode window and its OpenGL context. This window will be half the desired
      // height because it will encode two monochrome pixels into a single color pixel.
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
      Display::m_impl->m_window = glfwCreateWindow(desiredWidth/2, desiredHeight, "XSight", nullptr,
        windowToShare);
      if (sharedWindow) {
        if (!sharedWindow->ReturnContext()) {
          m_status = "Failed to return context to shared window";
          return;
        }
      }
    }

    // Verify that the window was created.
    if (!Display::m_impl->m_window) {
      m_status = "Failed to create GLFW window";
      return;
    }

    // Determine the full-screen monitor to use.
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
    GLFWmonitor* fullScreenMonitor = monitors[desiredDisplay];

    // Engage full screen here along with specifying the refresh rate.  The width is half of that specified
    // because the final render pass will encode two monochrome pixels into each color pixel.
    glfwSetWindowMonitor(Display::m_impl->m_window, fullScreenMonitor, 0, 0, desiredWidth/2, desiredHeight, fps);

    // Grab the context mutex for the duration of the setup.  Once we have it, we know
    // that the context is not active in another thread.
    // Make the window's context current.
    // DO NOT do any GLFW calls while holding the context -- it causes rare hangs on Linux.
    std::lock_guard<std::mutex> lock(Display::m_impl->m_contextMutex);
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

    // Disable SRGB on the frame buffer so our pixel values are not gamma corrected.
    glDisable(GL_FRAMEBUFFER_SRGB);

    // Construct the frame buffer object that we will render the full-sized image into, along with
    // its color and depth buffers.
    glGenFramebuffers(1, &m_impl->m_framebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, m_impl->m_framebuffer);
    glGenTextures(1, &m_impl->m_colorBuffer);
    glBindTexture(GL_TEXTURE_2D, m_impl->m_colorBuffer);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, desiredWidth, desiredHeight, 0,
      GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glGenTextures(1, &m_impl->m_depthBuffer);
    glBindTexture(GL_TEXTURE_2D, m_impl->m_depthBuffer);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32F, desiredWidth, desiredHeight, 0,
      GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);

    // Release the window's current context in case another Display wants to borrow it.
    glfwMakeContextCurrent(nullptr);

    // After we're done with the context for set-up and have released it, indicate that the context is available
    // for borrowing.
    glFinish();
    Display::m_impl->m_contextAvailable = true;
  }

  // Place holders for LOS data read from the XSight.
  float azimuth = 0.0f;
  float elevation = 0.0f;
  float roll = 0.0f;
  uint32_t time = 0;

  // Open a multicast UDP receiver to listen for LOS data from the XSight on the specified port.
  asdp::ReceiverUDP receiver(m_NICName, 5535, 9000 - 28, false, "224.0.0.50");
  Status status = receiver.GetConstructorStatus();
  if (status != OKAY) {
    m_status = "Failed to create UDP receiver";
    return;
  }

  // Construct a CompositePackXSightFrame to pack the pixels from the full-screen render into the
  // physical display.
  Display::m_impl->m_contextMutex.lock();
  glfwMakeContextCurrent(Display::m_impl->m_window);
  CompositePackXSightFrame packComposite(m_impl->m_colorBuffer, desiredWidth / 2);
  glfwMakeContextCurrent(nullptr);
  Display::m_impl->m_contextMutex.unlock();

  // Construct a CompositeLineRawData to render frame metadata into the first line, using the whole first line.
  // Note that this is an encoded first line, so is half as long as the requested width.
  Display::m_impl->m_contextMutex.lock();
  std::vector<uint8_t> lineData(desiredWidth/2 * 3);
  glfwMakeContextCurrent(Display::m_impl->m_window);
  CompositeLineRawData lineComposite(-1, 1, 1, 1, lineData);
  glfwMakeContextCurrent(nullptr);
  Display::m_impl->m_contextMutex.unlock();

  // Loop until the display is done.
  bool frameCompleted = false;
  while (!m_done) {

    // Poll for and process events without a context to avoid multiple threads trying to get
    // the context at the same time.
    glfwPollEvents();

    // Determine the scan-out time of the frame (center of the image).
    Time renderTime;
    m_timer->GetCoreTime(renderTime, std::chrono::steady_clock::now());
    if (!m_replaying) {
      double frameTime = 1.0 / fps;
      double middleOfNextFrameOffset = frameTime / 2.0 + renderAheadMicroseconds / 1e6;
      uint32_t seconds = static_cast<uint32_t>(middleOfNextFrameOffset);
      uint32_t microseconds = (middleOfNextFrameOffset - seconds) * 1e6;
      renderTime += Time(seconds, microseconds);
    }

    // Adjust render time if we're paused.
    if (m_pauseTime) {
      renderTime = *m_pauseTime;
    }

    // Wait until it is time to compute depth for the next frame.  We must busy-wait here to avoid having our
    // thread swapped out for longer than we want.
    while (std::chrono::steady_clock::now() < m_impl->m_nextDepthTime) {
    }
    if (m_eventHandlers && m_eventHandlers->ComputeDepth) {
      // Make the window's context current during depth calculations
      std::lock_guard<std::mutex> lock(Display::m_impl->m_contextMutex);
      glfwMakeContextCurrent(Display::m_impl->m_window);

      GLenum err = glGetError();
      if (err != GL_NO_ERROR) {
        std::cerr << "OpenGL error before checking whether to call ComputeDepth: " << err << std::endl;
      }
      m_eventHandlers->ComputeDepth(renderTime, m_userData);

      // Release the window's current context in case another Display wants to borrow it.
      glfwMakeContextCurrent(nullptr);
    }

    // Wait until it is time to render the next frame.  We must busy-wait here to avoid having our
    // thread swapped out for longer than we want.
    while (std::chrono::steady_clock::now() < m_impl->m_nextRenderTime) {
    }

    // Quit when our window closes.
    if (glfwWindowShouldClose(Display::m_impl->m_window)) {
      m_composite.reset();
      m_status = "Done";
      break;
    }

    //======================================
    // For the X-sight helmet display, added by Sang Yoon to add key mappings for closing windows (Q or ESCAPE key on keyboard),
    // pausing/resuming replaying (SPACE key on keyboard),
    // and enabling/disabling depth computation (d key on keyboard)

    // Adding key mappings for closing windows
    if (glfwGetKey(Display::m_impl->m_window, GLFW_KEY_Q) == GLFW_PRESS
      || glfwGetKey(Display::m_impl->m_window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
      m_composite.reset();
      m_status = "Done";
      break;
    }

    // Toggle play/pause when the space key is pressed (once per press/release cycle).
    bool spacePressed = (glfwGetKey(Display::m_impl->m_window, GLFW_KEY_SPACE) == GLFW_PRESS);
    if (spacePressed && !m_impl->m_spacePressed) {
      if (m_eventHandlers && m_eventHandlers->ChangePlayPause) {
          m_eventHandlers->ChangePlayPause(!m_nowPlaying, m_userData);
      }
    }
    m_impl->m_spacePressed = spacePressed;

    // Toggle depth computation when the 'd' key is pressed (once per press/release cycle).
    bool dPressed = (glfwGetKey(Display::m_impl->m_window, GLFW_KEY_D) == GLFW_PRESS);
    if (dPressed && !m_impl->m_dPressed) {
      m_impl->m_displayingDepth = !m_impl->m_displayingDepth;
      if (m_eventHandlers && m_eventHandlers->SetToRenderDepth) {
        m_eventHandlers->SetToRenderDepth(m_impl->m_displayingDepth, m_userData);
      }
    }
    m_impl->m_dPressed = dPressed;
    //======================================

    // Process any incoming pose requests; update view orientation to match.
    bool packetReady = false;
    Status status = receiver.IsPacketAvailable(0, packetReady);
    if (status != OKAY) {
      m_status = "Failed to check for packet availability";
      break;
    }
    size_t length = 9000;
    std::vector<uint8_t> buffer(length);
    while (packetReady) {
      // Read the next packet, letting it fill the whole buffer.
      length = 9000;
      status = receiver.ReceiveBuffer(buffer.data(), length);
      if (status != OKAY) {
        m_status = "Failed to receive packet";
        break;
      }

      // If we don't get opcode 7, then we ignore it -- this is the first byte in the message.
      if (buffer[0] == 7) {
        // Verify that the packet length is as expected and then parse it.
        constexpr uint32_t expectedLength = 1 + 3 * 4 + 3 * 4 + 2 + 3 * 4 + 4 + 4 + 3 * 4 + 1 + 3 * 4;
        if (length != expectedLength) {
          m_status = "Received packet of unexpected length (" + std::to_string(length) +
            " received, " + std::to_string(expectedLength) + " expected)";
          break;
        }

        // Pull each of the fields out of the packet into a buffer, then perform byte order conversion,
        // then copy into the result. We do the two copies because the bytes are not always aligned in the
        // packet.  We first check to see if the packet is valid and leave things alone if it is not.
        //bool valid = buffer[1 + 6 * 4 + 2 + 3 * 4 + 4 + 4 + 3 * 4] != 0;
        if (true) { // Do not need to check for the validity of the packet -- that refers to degraded tracking
          uint32_t temp;
          memcpy(&temp, &(buffer[1 + 5 * 4]), sizeof(temp));
          temp = ntohl(temp);
          memcpy(&azimuth, &temp, sizeof(azimuth));
          memcpy(&temp, &(buffer[1 + 4 * 4]), sizeof(temp));
          temp = ntohl(temp);
          memcpy(&elevation, &temp, sizeof(elevation));
          memcpy(&temp, &(buffer[1 + 3 * 4]), sizeof(temp));
          temp = ntohl(temp);
          memcpy(&roll, &temp, sizeof(roll));
          memcpy(&temp, &(buffer[1 + 6 * 4 + 2 + 3 * 4]), sizeof(temp));
          temp = ntohl(temp);
          memcpy(&time, &temp, sizeof(time));
        }
      }

      // Check for another packet, so we gobble up any that are waiting.
      Status status = receiver.IsPacketAvailable(0, packetReady);
      if (status != OKAY) {
        m_status = "Failed to check for packet availability";
        break;
      }
    }

    // Update the transformation for the view.  We first rotate around roll, then pitch, then yaw.
    // Empirically, the azimuth is backwards from what we expect, so we negate it.
    // Empirically, the root and pitch are swapped, so we swap them.
    // Empirically, this order of rotation works.
    glm::quat rotationX = glm::angleAxis(glm::radians(elevation), glm::vec3(1.0f, 0.0f, 0.0f));
    glm::quat rotationY = glm::angleAxis(glm::radians(roll), glm::vec3(0.0f, 1.0f, 0.0f));
    glm::quat rotationZ = glm::angleAxis(glm::radians(-azimuth), glm::vec3(0.0f, 0.0f, 1.0f));
    glm::quat rotationTotal = rotationZ * rotationX * rotationY;
    m_impl->m_views[0].orientation[0] = rotationTotal.w;
    m_impl->m_views[0].orientation[1] = rotationTotal.x;
    m_impl->m_views[0].orientation[2] = rotationTotal.y;
    m_impl->m_views[0].orientation[3] = rotationTotal.z;

    // Handle any window resizing
    SetViewportSizeAndFOVs(m_impl->m_views[0]);

    // Set the viewport to use the frame buffer and its associated color and depth buffers.
    m_impl->m_views[0].frameBuffer = m_impl->m_framebuffer;
    m_impl->m_views[0].colorBuffer = m_impl->m_colorBuffer;
    m_impl->m_views[0].depthBuffer = m_impl->m_depthBuffer;

    // Trigger the cameras, saying that we need the data now. The base class will handle offsetting
    // by the specified transmission/processing time as passed to its constructor by the client.
    TriggerCameras(std::chrono::steady_clock::now());

    // Record the render start time if we have a place to put it.
    if (m_timingInfo) {
      m_timingInfo->renderStartTimes.push_back(std::chrono::steady_clock::now());
    }

    // Grab the context mutex for the duration of the loop.  Once we have it, we know
    // that the context is not active in another thread.
    // Make the window's context current.
    // DO NOT do any GLFW calls while holding the context -- it causes rare hangs on Linux.
    std::lock_guard<std::mutex> lock(Display::m_impl->m_contextMutex);
    glfwMakeContextCurrent(Display::m_impl->m_window);

    // Embed the azimuth, elevation, roll, and time into the first line of the image.
    EmbedLOSData(azimuth, elevation, roll, time, lineData);
    lineComposite.UpdateValues(lineData);

    // Render the frame data
    m_composite->Render(renderTime, m_impl->m_views);

    // Wait for the rendering to finish so that the frame buffer textures will be ready to read.
    // Use a fence to wait for the rendering to finish.
    GLsync fence = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
    if (fence == nullptr) {
      m_status = "Failed to create fence";
      return;
    }
    GLenum fenceWait = glClientWaitSync(fence, GL_SYNC_FLUSH_COMMANDS_BIT, 1000000000);
    if (fenceWait != GL_ALREADY_SIGNALED && fenceWait != GL_CONDITION_SATISFIED) {
      m_status = "glClientWaitSync() failed for xSight: code " + std::to_string(fenceWait);
    }

    // Set the viewport to use the default frame buffer and to render to half the width, as we're converting
    // from 2x wide color to mono and then embedding in 1x wide color.
    m_impl->m_views[0].frameBuffer = 0;
    m_impl->m_views[0].colorBuffer = 0;
    m_impl->m_views[0].depthBuffer = 0;
    m_impl->m_views[0].width /= 2;

    // Do a second rendering pass where we read from the double-width color buffer and write to the output
    // buffer, packing two horizontal pixels into one.  This uses a custom compositor that does the pixel
    // packing.
    packComposite.Render(renderTime, m_impl->m_views);

    // Render the frame metadata into the first line.
    lineComposite.Render(renderTime, m_impl->m_views);

    // Record the render submit time if we have a place to put it.
    if (m_timingInfo) {
      m_timingInfo->renderSubmitTimes.push_back(std::chrono::steady_clock::now());
    }

    // Swap front and back buffers and wait for it to complete, then compute the next frame time.
    glfwSwapBuffers(Display::m_impl->m_window);
    glFinish();
    auto nowTime = std::chrono::steady_clock::now();
    m_impl->m_nextRenderTime = nowTime +
      std::chrono::microseconds(static_cast<long long>(1e6 / fps) - renderAheadMicroseconds);
    m_impl->m_nextDepthTime = m_impl->m_nextRenderTime - std::chrono::microseconds(m_depthAheadMicroseconds);
    // Half way through the next frame, which is when we want the geometry adjusted for.
    m_impl->m_nextFrameTime = nowTime +
      std::chrono::microseconds(static_cast<long long>(1e6 / fps) * 3 / 2);

    // Release the window's current context in case another Display wants to borrow it.
    glfwMakeContextCurrent(nullptr);
  }

  // Grab the context so we can destroy our framebuffer and textures.
  {
    std::lock_guard<std::mutex> lock(Display::m_impl->m_contextMutex);
    glfwMakeContextCurrent(Display::m_impl->m_window);
    glDeleteTextures(1, &m_impl->m_colorBuffer);
    m_impl->m_colorBuffer = 0;
    glDeleteTextures(1, &m_impl->m_depthBuffer);
    m_impl->m_depthBuffer = 0;
    glDeleteFramebuffers(1, &m_impl->m_framebuffer);
    m_impl->m_framebuffer = 0;
    glfwMakeContextCurrent(nullptr);
  }

  // Done with the window
  glfwDestroyWindow(Display::m_impl->m_window);
}

void DisplayXSight::SetNowPlaying(bool nowPlaying)
{
  // Call the parent-class method to set the now-playing state.
  Display::SetNowPlaying(nowPlaying);

  // Set the pause time based on whether we are now playing so that
  // we don't extrapolate forward in time while paused.
  if (!m_nowPlaying) {
    m_pauseTime = std::make_unique<Time>();
    m_timer->GetCoreTime(*m_pauseTime, std::chrono::steady_clock::now());
  }
  else {
    m_pauseTime.reset();
  }
}
