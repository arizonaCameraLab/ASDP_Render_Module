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
  std::shared_ptr<EventHandlers> handlers, void* userData)
  : m_composite(composite)
  , m_eventHandlers(handlers)
  , m_userData(userData)
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

  /// Angles of rotation in degrees based on keyboard and/or joystick input.
  /// rotation is around the original Z axis, then the original Z axis.
  float m_rotationZDegrees {0.0f};
  float m_rotationXDegrees = {0.0f};

  /// Last time we checked the keyboard, used to control motion rate.
  std::chrono::steady_clock::time_point m_lastKeyboardCheck;

  /// Time point to start rendering the next frame.
  std::chrono::steady_clock::time_point m_nextFrameTime;

  /// Whether the space bar was pressed during the last loop, used to toggle play/pause.
  bool m_spacePressed = false;

  /// Index of the joystick to use, or -1 if no joystick is to be used.
  int m_glfwJoystickIndex = -1;

  /// Name of joysticks that should be flipping in the Y axis.
  std::vector<std::string> m_flipYJoysticks = { "Logitech Extreme 3D" };

  /// Scale of the joystick input in Y axis, flipped if the joystick is on the list above.
  float m_joystickScaleY = 1.0f;
};

DisplayWindow::DisplayWindow(std::string windowName, std::shared_ptr<Composite> composite,
    std::shared_ptr<CoreClient> client, uint8_t triggerID, uint32_t triggerAheadMicroseconds,
    float fps, uint32_t renderAheadMicroseconds,
    int desiredWidth, int desiredHeight, float horizontalFOVDegrees,
    std::string joystick, Display* sharedWindow,
    bool fullScreen, int desiredDisplay, bool hidden,
    std::shared_ptr<EventHandlers> handlers, void* userData)
  : Display(composite, client, triggerID, triggerAheadMicroseconds, handlers, userData)
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
  auto lastJoystickCheck = std::chrono::steady_clock::now();
  while (!m_done) {
    // Wait until it is time to render the next frame.  We must busy-wait here to avoid having our
    // thread swapped out for longer than we want.
    while (std::chrono::steady_clock::now() < m_impl->m_nextFrameTime) {
    }

    // Grab the context mutex for the duration of the loop.  Once we have it, we know
    // that the context is not active in another thread.
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
    if (m_impl->m_glfwJoystickIndex >= 0) {
      auto now = std::chrono::steady_clock::now();
      std::chrono::duration<double> elapsed = now - lastJoystickCheck;
      lastJoystickCheck = now;

      int axisCount;
      const float* axes = glfwGetJoystickAxes(m_impl->m_glfwJoystickIndex, &axisCount);
      if (axisCount >= 2) {
        if (fabs(axes[0]) > 0.15) {
          m_impl->m_rotationZDegrees -= 90.0f * elapsed.count() * axes[0];
        }
        if (fabs(axes[1]) > 0.15) {
          m_impl->m_rotationXDegrees -= 90.0f * elapsed.count() * axes[1] * m_impl->m_joystickScaleY;
        }
      }
    }
    /// @todo mouse input

    // Ensure that the view orientation stays within bounds.
    ComputeAndClampViewOrientation();

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
  // Toggle play/pause when the space key is pressed (once per press/release cycle)
  bool spacePressed = (glfwGetKey(Display::m_impl->m_window, GLFW_KEY_SPACE) == GLFW_PRESS);
  if (spacePressed && !m_impl->m_spacePressed) {
    if (m_eventHandlers && m_eventHandlers->TogglePlayPause) {
      m_eventHandlers->TogglePlayPause(m_userData);
    }
  }
  m_impl->m_spacePressed = spacePressed;
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

  // Compute the orientation in Euler angles by building two different rotation
  // matrices and applying them in the correct order.
  float rotationZRadians = glm::radians(m_impl->m_rotationZDegrees);
  float rotationXRadians = glm::radians(m_impl->m_rotationXDegrees);

  // GLM gives us rotations in order Z, Y, X but we want X, Y, Z.  We make use of
  // the fact that an inverse rotation matrix is the same as doing three individual
  // rotations in the opposite directions and order.  So we find the inverse matrix
  // that we want, then ask for Euler angles from that and then negate them.

  // Create rotation matrices
  glm::mat4 rotationZ = glm::rotate(glm::mat4(1.0f), rotationZRadians, glm::vec3(0.0f, 0.0f, 1.0f));
  glm::mat4 rotationX = glm::rotate(rotationZ, rotationXRadians, glm::vec3(1.0f, 0.0f, 0.0f));

  // Combine the rotations: first X, then Z
  glm::mat4 combinedRotation = rotationX;

  // Find the inverse matrix
  glm::mat4 inverseRotation = glm::inverse(combinedRotation);

  // Decompose the combined rotation matrix to get the Euler angles
  glm::vec3 scale, translation, skew;
  glm::vec4 perspective;
  glm::quat orientation;
  glm::decompose(inverseRotation, scale, orientation, translation, skew, perspective);

  // Convert quaternion to Euler angles (X, Y, Z)
  glm::vec3 eulerAngles = glm::eulerAngles(orientation);

  // Convert radians to degrees
  eulerAngles = glm::degrees(eulerAngles);

  // Store the negative of the Euler angles in the view, completing the inverse.
  m_impl->m_views[0].orientation[0] = -eulerAngles[0];
  m_impl->m_views[0].orientation[1] = -eulerAngles[1];
  m_impl->m_views[0].orientation[2] = -eulerAngles[2];
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

#ifdef USE_OPENXR
#include "pch.h"
#include "common.h"
#include "check.h"
#ifdef _WIN32
#define GLFW_EXPOSE_NATIVE_WIN32
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

class asdp::render::DisplayOpenXR::DisplayOpenXRImpl {
public:
  DisplayOpenXRImpl(asdp::render::DisplayOpenXR* display)
    : m_display(display)
  {
  }
  DisplayOpenXR* m_display = nullptr;

  /// @todo Rename all of the g_ variables here to m_ once we have the code working.
  XrInstance g_instance{ XR_NULL_HANDLE };
  XrSession g_session{ XR_NULL_HANDLE };
  XrSpace g_appSpace{ XR_NULL_HANDLE };
  XrSystemId g_systemId{ XR_NULL_SYSTEM_ID };
  XrFormFactor g_formFactor{ XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY };
  XrViewConfigurationType g_viewConfigType{ XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO };
  XrEnvironmentBlendMode g_environmentBlendMode{ XR_ENVIRONMENT_BLEND_MODE_OPAQUE };
  int g_verbosity{ 0 };

  /// @todo Change these to match the desired behavior.
  struct Options {
    std::string GraphicsPlugin;
    XrFormFactor FormFactor{ XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY };
    XrViewConfigurationType ViewConfiguration{ XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO };
    XrEnvironmentBlendMode EnvironmentBlendMode{ XR_ENVIRONMENT_BLEND_MODE_OPAQUE };
    std::string AppSpace{ "Local" };
  } g_options;

#ifdef XR_USE_PLATFORM_WIN32
  XrGraphicsBindingOpenGLWin32KHR g_graphicsBinding{ XR_TYPE_GRAPHICS_BINDING_OPENGL_WIN32_KHR };
#elif defined(XR_USE_PLATFORM_XLIB)
  XrGraphicsBindingOpenGLXlibKHR g_graphicsBinding{ XR_TYPE_GRAPHICS_BINDING_OPENGL_XLIB_KHR };
#elif defined(XR_USE_PLATFORM_XCB)
  XrGraphicsBindingOpenGLXcbKHR g_graphicsBinding{ XR_TYPE_GRAPHICS_BINDING_OPENGL_XCB_KHR };
#elif defined(XR_USE_PLATFORM_WAYLAND)
  XrGraphicsBindingOpenGLWaylandKHR g_graphicsBinding{ XR_TYPE_GRAPHICS_BINDING_OPENGL_WAYLAND_KHR };
#endif
  GLFWwindow* m_contextWindow{ nullptr };

  // Application's current lifecycle state according to the runtime
  XrSessionState g_sessionState{ XR_SESSION_STATE_UNKNOWN };
  bool g_sessionRunning{ false };

  XrEventDataBuffer g_eventDataBuffer;

  struct InputState {
    XrActionSet actionSet{ XR_NULL_HANDLE };
    XrAction grabAction{ XR_NULL_HANDLE };
    XrAction poseAction{ XR_NULL_HANDLE };
    XrAction vibrateAction{ XR_NULL_HANDLE };
    XrAction quitAction{ XR_NULL_HANDLE };
    std::array<XrPath, Side::COUNT> handSubactionPath;
    std::array<XrSpace, Side::COUNT> handSpace;
    std::array<float, Side::COUNT> handScale = { {1.0f, 1.0f} };
    std::array<XrBool32, Side::COUNT> handActive;
  };
  InputState g_input;

  struct Swapchain {
    XrSwapchain handle;
    int32_t width;
    int32_t height;
  };

  std::vector<XrViewConfigurationView> g_configViews;
  std::vector<Swapchain> g_swapchains;
  std::map<XrSwapchain, std::vector<XrSwapchainImageBaseHeader*>> g_swapchainImages;
  std::vector<XrView> g_views;
  int64_t g_colorSwapchainFormat{ -1 };

  std::list<std::vector<XrSwapchainImageOpenGLKHR>> g_swapchainImageBuffers;
  GLuint g_swapchainFramebuffer{ 0 };

  // Map color buffer to associated depth buffer. This map is populated on demand.
  std::map<uint32_t, uint32_t> g_colorToDepthMap;

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

  CHECK(g_instance == XR_NULL_HANDLE);

  // Create union of extensions required by OpenGL.
  std::vector<const char*> extensions = { XR_KHR_OPENGL_ENABLE_EXTENSION_NAME };

  XrInstanceCreateInfo createInfo{ XR_TYPE_INSTANCE_CREATE_INFO };
  createInfo.next = nullptr;  // Needs to be set on Android.
  createInfo.enabledExtensionCount = (uint32_t)extensions.size();
  createInfo.enabledExtensionNames = extensions.data();

  /// @todo Change the application name here.
  strcpy(createInfo.applicationInfo.applicationName, "OpenXR-OpenGL-Example");
  createInfo.applicationInfo.apiVersion = XR_CURRENT_API_VERSION;

  CHECK_XRCMD(xrCreateInstance(&createInfo, &g_instance));
}

void asdp::render::DisplayOpenXR::DisplayOpenXRImpl::OpenXRInitializeSystem(Display* sharedWindow)
{
  CHECK(g_instance != XR_NULL_HANDLE);
  CHECK(g_systemId == XR_NULL_SYSTEM_ID);

  g_formFactor = g_options.FormFactor;
  g_viewConfigType = g_options.ViewConfiguration;
  g_environmentBlendMode = g_options.EnvironmentBlendMode;

  XrSystemGetInfo systemInfo{ XR_TYPE_SYSTEM_GET_INFO };
  systemInfo.formFactor = g_formFactor;
  CHECK_XRCMD(xrGetSystem(g_instance, &systemInfo, &g_systemId));

  if (g_verbosity >= 2) std::cout << "Using system " << g_systemId
    << " for form factor " << to_string(g_formFactor) << std::endl;
  CHECK(g_instance != XR_NULL_HANDLE);
  CHECK(g_systemId != XR_NULL_SYSTEM_ID);

  // The graphics API can initialize the graphics device now that the systemId and instance
  // handle are available.
  OpenGLInitializeDevice(sharedWindow, g_instance, g_systemId);
}

void asdp::render::DisplayOpenXR::DisplayOpenXRImpl::OpenGLInitializeDevice(Display* sharedWindow, XrInstance instance, XrSystemId systemId)
{
  // Extension function must be loaded by name
  PFN_xrGetOpenGLGraphicsRequirementsKHR pfnGetOpenGLGraphicsRequirementsKHR = nullptr;
  CHECK_XRCMD(xrGetInstanceProcAddr(instance, "xrGetOpenGLGraphicsRequirementsKHR",
    reinterpret_cast<PFN_xrVoidFunction*>(&pfnGetOpenGLGraphicsRequirementsKHR)));

  XrGraphicsRequirementsOpenGLKHR graphicsRequirements{ XR_TYPE_GRAPHICS_REQUIREMENTS_OPENGL_KHR };
  CHECK_XRCMD(pfnGetOpenGLGraphicsRequirementsKHR(instance, systemId, &graphicsRequirements));

  // Determine the OpenGL version.
  GLint major = 0;
  GLint minor = 0;
  glGetIntegerv(GL_MAJOR_VERSION, &major);
  glGetIntegerv(GL_MINOR_VERSION, &minor);

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
      THROW("OpenGLInitializeDevice(): Failed to borrow context from shared window");
      return;
    }
  }

  // Open a window that we will use to get a context that we will use to hand to OpenXR as needed.
  // This is a bit of a hack, but it is the only way to get a context that we can use with OpenXR.
  // We will use the context from this window to create the OpenXR session.
  // Set the window to be hidden.
  glfwWindowHint(GLFW_VISIBLE, false);
  m_contextWindow = glfwCreateWindow(100, 100, "OpenXR OpenGL Window to get context", nullptr, windowToShare);
  // Verify that the window was created.
  if (!static_cast<Display*>(m_display)->m_impl->m_window) {
    THROW("OpenGLInitializeDevice(): Failed to create GLFW window");
    return;
  }
  glfwMakeContextCurrent(m_contextWindow);

  const XrVersion desiredApiVersion = XR_MAKE_VERSION(major, minor, 0);
  if (graphicsRequirements.minApiVersionSupported > desiredApiVersion) {
    THROW("Runtime does not support desired Graphics API and/or version");
  }
#ifdef XR_USE_PLATFORM_WIN32
  /// @todo Consider doing this (and opening the window above) once we know the desired display window size from OpenXR
  g_graphicsBinding.hDC = wglGetCurrentDC();
  g_graphicsBinding.hGLRC = wglGetCurrentContext();
#elif defined(XR_USE_PLATFORM_XLIB)
  THROW("OpenGLInitializeDevice():Xlib not implemented here");
#elif defined(XR_USE_PLATFORM_XCB)
  THROW("OpenGLInitializeDevice():XCB not implemented here");
#elif defined(XR_USE_PLATFORM_WAYLAND)
  THROW("OpenGLInitializeDevice():Wayland not implemented here");
#endif

  if (sharedWindow) {
    if (!sharedWindow->ReturnContext()) {
      THROW("OpenGLInitializeDevice(): Failed to return context to shared window");
      return;
    }
  }

  /* @todo Can enable this for debugging
  glEnable(GL_DEBUG_OUTPUT);
  glDebugMessageCallback(
    [](GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar* message,
      const void* userParam) {
        std::cout << "GL Debug: " << std::string(message, 0, length) << std::endl;
    },
    nullptr);
  */
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
  CHECK(g_instance != XR_NULL_HANDLE);
  CHECK(g_session == XR_NULL_HANDLE);

  {
    if (g_verbosity >= 2) std::cout << Fmt("Creating session...") << std::endl;

    XrSessionCreateInfo createInfo{ XR_TYPE_SESSION_CREATE_INFO };
    createInfo.next = reinterpret_cast<const XrBaseInStructure*>(&g_graphicsBinding);
    createInfo.systemId = g_systemId;
    CHECK_XRCMD(xrCreateSession(g_instance, &createInfo, &g_session));
  }

  OpenXRInitializeActions();
  // Do not need unless we want other than helicopter space:  OpenXRCreateVisualizedSpaces();

  {
    XrReferenceSpaceCreateInfo referenceSpaceCreateInfo = GetXrReferenceSpaceCreateInfo(g_options.AppSpace);
    CHECK_XRCMD(xrCreateReferenceSpace(g_session, &referenceSpaceCreateInfo, &g_appSpace));
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
    CHECK_XRCMD(xrCreateActionSet(g_instance, &actionSetInfo, &g_input.actionSet));
  }

  // Get the XrPath for the left and right hands - we will use them as subaction paths.
  CHECK_XRCMD(xrStringToPath(g_instance, "/user/hand/left", &g_input.handSubactionPath[Side::LEFT]));
  CHECK_XRCMD(xrStringToPath(g_instance, "/user/hand/right", &g_input.handSubactionPath[Side::RIGHT]));

  // Create actions.
  {
    // Create an input action for grabbing objects with the left and right hands.
    XrActionCreateInfo actionInfo{ XR_TYPE_ACTION_CREATE_INFO };
    actionInfo.actionType = XR_ACTION_TYPE_FLOAT_INPUT;
    strcpy_s(actionInfo.actionName, "grab_object");
    strcpy_s(actionInfo.localizedActionName, "Grab Object");
    actionInfo.countSubactionPaths = uint32_t(g_input.handSubactionPath.size());
    actionInfo.subactionPaths = g_input.handSubactionPath.data();
    CHECK_XRCMD(xrCreateAction(g_input.actionSet, &actionInfo, &g_input.grabAction));

    // Create an input action getting the left and right hand poses.
    actionInfo.actionType = XR_ACTION_TYPE_POSE_INPUT;
    strcpy_s(actionInfo.actionName, "hand_pose");
    strcpy_s(actionInfo.localizedActionName, "Hand Pose");
    actionInfo.countSubactionPaths = uint32_t(g_input.handSubactionPath.size());
    actionInfo.subactionPaths = g_input.handSubactionPath.data();
    CHECK_XRCMD(xrCreateAction(g_input.actionSet, &actionInfo, &g_input.poseAction));

    // Create output actions for vibrating the left and right controller.
    actionInfo.actionType = XR_ACTION_TYPE_VIBRATION_OUTPUT;
    strcpy_s(actionInfo.actionName, "vibrate_hand");
    strcpy_s(actionInfo.localizedActionName, "Vibrate Hand");
    actionInfo.countSubactionPaths = uint32_t(g_input.handSubactionPath.size());
    actionInfo.subactionPaths = g_input.handSubactionPath.data();
    CHECK_XRCMD(xrCreateAction(g_input.actionSet, &actionInfo, &g_input.vibrateAction));

    // Create input actions for quitting the session using the left and right controller.
    // Since it doesn't matter which hand did this, we do not specify subaction paths for it.
    // We will just suggest bindings for both hands, where possible.
    actionInfo.actionType = XR_ACTION_TYPE_BOOLEAN_INPUT;
    strcpy_s(actionInfo.actionName, "quit_session");
    strcpy_s(actionInfo.localizedActionName, "Quit Session");
    actionInfo.countSubactionPaths = 0;
    actionInfo.subactionPaths = nullptr;
    CHECK_XRCMD(xrCreateAction(g_input.actionSet, &actionInfo, &g_input.quitAction));
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
  CHECK_XRCMD(xrStringToPath(g_instance, "/user/hand/left/input/select/click", &selectPath[Side::LEFT]));
  CHECK_XRCMD(xrStringToPath(g_instance, "/user/hand/right/input/select/click", &selectPath[Side::RIGHT]));
  CHECK_XRCMD(xrStringToPath(g_instance, "/user/hand/left/input/squeeze/value", &squeezeValuePath[Side::LEFT]));
  CHECK_XRCMD(xrStringToPath(g_instance, "/user/hand/right/input/squeeze/value", &squeezeValuePath[Side::RIGHT]));
  CHECK_XRCMD(xrStringToPath(g_instance, "/user/hand/left/input/squeeze/force", &squeezeForcePath[Side::LEFT]));
  CHECK_XRCMD(xrStringToPath(g_instance, "/user/hand/right/input/squeeze/force", &squeezeForcePath[Side::RIGHT]));
  CHECK_XRCMD(xrStringToPath(g_instance, "/user/hand/left/input/squeeze/click", &squeezeClickPath[Side::LEFT]));
  CHECK_XRCMD(xrStringToPath(g_instance, "/user/hand/right/input/squeeze/click", &squeezeClickPath[Side::RIGHT]));
  CHECK_XRCMD(xrStringToPath(g_instance, "/user/hand/left/input/grip/pose", &posePath[Side::LEFT]));
  CHECK_XRCMD(xrStringToPath(g_instance, "/user/hand/right/input/grip/pose", &posePath[Side::RIGHT]));
  CHECK_XRCMD(xrStringToPath(g_instance, "/user/hand/left/output/haptic", &hapticPath[Side::LEFT]));
  CHECK_XRCMD(xrStringToPath(g_instance, "/user/hand/right/output/haptic", &hapticPath[Side::RIGHT]));
  CHECK_XRCMD(xrStringToPath(g_instance, "/user/hand/left/input/menu/click", &menuClickPath[Side::LEFT]));
  CHECK_XRCMD(xrStringToPath(g_instance, "/user/hand/right/input/menu/click", &menuClickPath[Side::RIGHT]));
  CHECK_XRCMD(xrStringToPath(g_instance, "/user/hand/left/input/b/click", &bClickPath[Side::LEFT]));
  CHECK_XRCMD(xrStringToPath(g_instance, "/user/hand/right/input/b/click", &bClickPath[Side::RIGHT]));
  CHECK_XRCMD(xrStringToPath(g_instance, "/user/hand/left/input/trigger/value", &triggerValuePath[Side::LEFT]));
  CHECK_XRCMD(xrStringToPath(g_instance, "/user/hand/right/input/trigger/value", &triggerValuePath[Side::RIGHT]));
  // Suggest bindings for KHR Simple.
  {
    XrPath khrSimpleInteractionProfilePath;
    CHECK_XRCMD(
      xrStringToPath(g_instance, "/interaction_profiles/khr/simple_controller", &khrSimpleInteractionProfilePath));
    std::vector<XrActionSuggestedBinding> bindings{ {// Fall back to a click input for the grab action.
                                                    {g_input.grabAction, selectPath[Side::LEFT]},
                                                    {g_input.grabAction, selectPath[Side::RIGHT]},
                                                    {g_input.poseAction, posePath[Side::LEFT]},
                                                    {g_input.poseAction, posePath[Side::RIGHT]},
                                                    {g_input.quitAction, menuClickPath[Side::LEFT]},
                                                    {g_input.quitAction, menuClickPath[Side::RIGHT]},
                                                    {g_input.vibrateAction, hapticPath[Side::LEFT]},
                                                    {g_input.vibrateAction, hapticPath[Side::RIGHT]}} };
    XrInteractionProfileSuggestedBinding suggestedBindings{ XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING };
    suggestedBindings.interactionProfile = khrSimpleInteractionProfilePath;
    suggestedBindings.suggestedBindings = bindings.data();
    suggestedBindings.countSuggestedBindings = (uint32_t)bindings.size();
    CHECK_XRCMD(xrSuggestInteractionProfileBindings(g_instance, &suggestedBindings));
  }
  // Suggest bindings for the Oculus Touch.
  {
    XrPath oculusTouchInteractionProfilePath;
    CHECK_XRCMD(
      xrStringToPath(g_instance, "/interaction_profiles/oculus/touch_controller", &oculusTouchInteractionProfilePath));
    std::vector<XrActionSuggestedBinding> bindings{ {{g_input.grabAction, squeezeValuePath[Side::LEFT]},
                                                    {g_input.grabAction, squeezeValuePath[Side::RIGHT]},
                                                    {g_input.poseAction, posePath[Side::LEFT]},
                                                    {g_input.poseAction, posePath[Side::RIGHT]},
                                                    {g_input.quitAction, menuClickPath[Side::LEFT]},
                                                    {g_input.vibrateAction, hapticPath[Side::LEFT]},
                                                    {g_input.vibrateAction, hapticPath[Side::RIGHT]}} };
    XrInteractionProfileSuggestedBinding suggestedBindings{ XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING };
    suggestedBindings.interactionProfile = oculusTouchInteractionProfilePath;
    suggestedBindings.suggestedBindings = bindings.data();
    suggestedBindings.countSuggestedBindings = (uint32_t)bindings.size();
    CHECK_XRCMD(xrSuggestInteractionProfileBindings(g_instance, &suggestedBindings));
  }
  // Suggest bindings for the Vive Controller.
  {
    XrPath viveControllerInteractionProfilePath;
    CHECK_XRCMD(
      xrStringToPath(g_instance, "/interaction_profiles/htc/vive_controller", &viveControllerInteractionProfilePath));
    std::vector<XrActionSuggestedBinding> bindings{ {{g_input.grabAction, triggerValuePath[Side::LEFT]},
                                                    {g_input.grabAction, triggerValuePath[Side::RIGHT]},
                                                    {g_input.poseAction, posePath[Side::LEFT]},
                                                    {g_input.poseAction, posePath[Side::RIGHT]},
                                                    {g_input.quitAction, menuClickPath[Side::LEFT]},
                                                    {g_input.quitAction, menuClickPath[Side::RIGHT]},
                                                    {g_input.vibrateAction, hapticPath[Side::LEFT]},
                                                    {g_input.vibrateAction, hapticPath[Side::RIGHT]}} };
    XrInteractionProfileSuggestedBinding suggestedBindings{ XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING };
    suggestedBindings.interactionProfile = viveControllerInteractionProfilePath;
    suggestedBindings.suggestedBindings = bindings.data();
    suggestedBindings.countSuggestedBindings = (uint32_t)bindings.size();
    CHECK_XRCMD(xrSuggestInteractionProfileBindings(g_instance, &suggestedBindings));
  }

  // Suggest bindings for the Valve Index Controller.
  {
    XrPath indexControllerInteractionProfilePath;
    CHECK_XRCMD(
      xrStringToPath(g_instance, "/interaction_profiles/valve/index_controller", &indexControllerInteractionProfilePath));
    std::vector<XrActionSuggestedBinding> bindings{ {{g_input.grabAction, squeezeForcePath[Side::LEFT]},
                                                    {g_input.grabAction, squeezeForcePath[Side::RIGHT]},
                                                    {g_input.poseAction, posePath[Side::LEFT]},
                                                    {g_input.poseAction, posePath[Side::RIGHT]},
                                                    {g_input.quitAction, bClickPath[Side::LEFT]},
                                                    {g_input.quitAction, bClickPath[Side::RIGHT]},
                                                    {g_input.vibrateAction, hapticPath[Side::LEFT]},
                                                    {g_input.vibrateAction, hapticPath[Side::RIGHT]}} };
    XrInteractionProfileSuggestedBinding suggestedBindings{ XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING };
    suggestedBindings.interactionProfile = indexControllerInteractionProfilePath;
    suggestedBindings.suggestedBindings = bindings.data();
    suggestedBindings.countSuggestedBindings = (uint32_t)bindings.size();
    CHECK_XRCMD(xrSuggestInteractionProfileBindings(g_instance, &suggestedBindings));
  }

  // Suggest bindings for the Microsoft Mixed Reality Motion Controller.
  {
    XrPath microsoftMixedRealityInteractionProfilePath;
    CHECK_XRCMD(xrStringToPath(g_instance, "/interaction_profiles/microsoft/motion_controller",
      &microsoftMixedRealityInteractionProfilePath));
    std::vector<XrActionSuggestedBinding> bindings{ {{g_input.grabAction, squeezeClickPath[Side::LEFT]},
                                                    {g_input.grabAction, squeezeClickPath[Side::RIGHT]},
                                                    {g_input.poseAction, posePath[Side::LEFT]},
                                                    {g_input.poseAction, posePath[Side::RIGHT]},
                                                    {g_input.quitAction, menuClickPath[Side::LEFT]},
                                                    {g_input.quitAction, menuClickPath[Side::RIGHT]},
                                                    {g_input.vibrateAction, hapticPath[Side::LEFT]},
                                                    {g_input.vibrateAction, hapticPath[Side::RIGHT]}} };
    XrInteractionProfileSuggestedBinding suggestedBindings{ XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING };
    suggestedBindings.interactionProfile = microsoftMixedRealityInteractionProfilePath;
    suggestedBindings.suggestedBindings = bindings.data();
    suggestedBindings.countSuggestedBindings = (uint32_t)bindings.size();
    CHECK_XRCMD(xrSuggestInteractionProfileBindings(g_instance, &suggestedBindings));
  }
  XrActionSpaceCreateInfo actionSpaceInfo{ XR_TYPE_ACTION_SPACE_CREATE_INFO };
  actionSpaceInfo.action = g_input.poseAction;
  actionSpaceInfo.poseInActionSpace.orientation.w = 1.f;
  actionSpaceInfo.subactionPath = g_input.handSubactionPath[Side::LEFT];
  CHECK_XRCMD(xrCreateActionSpace(g_session, &actionSpaceInfo, &g_input.handSpace[Side::LEFT]));
  actionSpaceInfo.subactionPath = g_input.handSubactionPath[Side::RIGHT];
  CHECK_XRCMD(xrCreateActionSpace(g_session, &actionSpaceInfo, &g_input.handSpace[Side::RIGHT]));

  XrSessionActionSetsAttachInfo attachInfo{ XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO };
  attachInfo.countActionSets = 1;
  attachInfo.actionSets = &g_input.actionSet;
  CHECK_XRCMD(xrAttachSessionActionSets(g_session, &attachInfo));
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
  g_swapchainImageBuffers.push_back(std::move(swapchainImageBuffer));

  return swapchainImageBase;
}

void asdp::render::DisplayOpenXR::DisplayOpenXRImpl::OpenXRCreateSwapchains()
{
  // Create the swapchains for the views.
  // @todo Implement this.  CHECK(g_session != XR_NULL_HANDLE);
  CHECK(g_swapchains.empty());
  CHECK(g_configViews.empty());

  // Read graphics properties for preferred swapchain length and logging.
  XrSystemProperties systemProperties{ XR_TYPE_SYSTEM_PROPERTIES };
  CHECK_XRCMD(xrGetSystemProperties(g_instance, g_systemId, &systemProperties));

  // Log system properties.
  if (g_verbosity >= 1) {
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
  CHECK_MSG(g_viewConfigType == XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, "Unsupported view configuration type");

  // Query and cache view configuration views.
  uint32_t viewCount;
  CHECK_XRCMD(xrEnumerateViewConfigurationViews(g_instance, g_systemId, g_viewConfigType, 0, &viewCount, nullptr));
  g_configViews.resize(viewCount, { XR_TYPE_VIEW_CONFIGURATION_VIEW });
  CHECK_XRCMD(xrEnumerateViewConfigurationViews(g_instance, g_systemId, g_viewConfigType, viewCount, &viewCount,
    g_configViews.data()));

  // Create and cache view buffer for xrLocateViews later.
  g_views.resize(viewCount, { XR_TYPE_VIEW });

  // Create the swapchain and get the images.
  if (viewCount > 0) {
    // Select a swapchain format.
    uint32_t swapchainFormatCount;
    CHECK_XRCMD(xrEnumerateSwapchainFormats(g_session, 0, &swapchainFormatCount, nullptr));
    std::vector<int64_t> swapchainFormats(swapchainFormatCount);
    CHECK_XRCMD(xrEnumerateSwapchainFormats(g_session, (uint32_t)swapchainFormats.size(), &swapchainFormatCount,
      swapchainFormats.data()));
    CHECK(swapchainFormatCount == swapchainFormats.size());
    g_colorSwapchainFormat = OpenGLSelectColorSwapchainFormat(swapchainFormats);

    // Print swapchain formats and the selected one.
    {
      std::string swapchainFormatsString;
      for (int64_t format : swapchainFormats) {
        const bool selected = format == g_colorSwapchainFormat;
        swapchainFormatsString += " ";
        if (selected) {
          swapchainFormatsString += "[";
        }
        swapchainFormatsString += std::to_string(format);
        if (selected) {
          swapchainFormatsString += "]";
        }
      }
      if (g_verbosity >= 1) std::cout << Fmt("Swapchain Formats: %s", swapchainFormatsString.c_str()) << std::endl;
    }

    // Create a swapchain for each view.
    for (uint32_t i = 0; i < viewCount; i++) {
      const XrViewConfigurationView& vp = g_configViews[i];
      if (g_verbosity >= 1) {
        std::cout <<
          Fmt("Creating swapchain for view %d with dimensions Width=%d Height=%d SampleCount=%d", i,
            vp.recommendedImageRectWidth, vp.recommendedImageRectHeight, vp.recommendedSwapchainSampleCount)
          << std::endl;
      }

      // Create the swapchain.
      XrSwapchainCreateInfo swapchainCreateInfo{ XR_TYPE_SWAPCHAIN_CREATE_INFO };
      swapchainCreateInfo.arraySize = 1;
      swapchainCreateInfo.format = g_colorSwapchainFormat;
      swapchainCreateInfo.width = vp.recommendedImageRectWidth;
      swapchainCreateInfo.height = vp.recommendedImageRectHeight;
      swapchainCreateInfo.mipCount = 1;
      swapchainCreateInfo.faceCount = 1;
      swapchainCreateInfo.sampleCount = vp.recommendedSwapchainSampleCount;
      swapchainCreateInfo.usageFlags = XR_SWAPCHAIN_USAGE_SAMPLED_BIT | XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT;
      Swapchain swapchain;
      swapchain.width = swapchainCreateInfo.width;
      swapchain.height = swapchainCreateInfo.height;
      CHECK_XRCMD(xrCreateSwapchain(g_session, &swapchainCreateInfo, &swapchain.handle));

      g_swapchains.push_back(swapchain);

      uint32_t imageCount;
      CHECK_XRCMD(xrEnumerateSwapchainImages(swapchain.handle, 0, &imageCount, nullptr));
      // XXX This should really just return XrSwapchainImageBaseHeader*
      std::vector<XrSwapchainImageBaseHeader*> swapchainImages =
        OpenGLAllocateSwapchainImageStructs(imageCount, swapchainCreateInfo);
      CHECK_XRCMD(xrEnumerateSwapchainImages(swapchain.handle, imageCount, &imageCount, swapchainImages[0]));

      g_swapchainImages.insert(std::make_pair(swapchain.handle, std::move(swapchainImages)));
    }
  }
}

// Return event if one is available, otherwise return null.
XrEventDataBaseHeader* asdp::render::DisplayOpenXR::DisplayOpenXRImpl::OpenXRTryReadNextEvent()
{
  // It is sufficient to clear the just the XrEventDataBuffer header to
  // XR_TYPE_EVENT_DATA_BUFFER
  XrEventDataBaseHeader* baseHeader = reinterpret_cast<XrEventDataBaseHeader*>(&g_eventDataBuffer);
  *baseHeader = { XR_TYPE_EVENT_DATA_BUFFER };
  const XrResult xr = xrPollEvent(g_instance, &g_eventDataBuffer);
  if (xr == XR_SUCCESS) {
    if (baseHeader->type == XR_TYPE_EVENT_DATA_EVENTS_LOST) {
      const XrEventDataEventsLost* const eventsLost = reinterpret_cast<const XrEventDataEventsLost*>(baseHeader);
      if (g_verbosity > 0) std::cerr << Fmt("%d events lost", eventsLost) << std::endl;
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
  const XrSessionState oldState = g_sessionState;
  g_sessionState = stateChangedEvent.state;

  if (g_verbosity >= 1) {
    std::cout << Fmt("XrEventDataSessionStateChanged: state %s->%s session=%lld time=%lld", to_string(oldState),
      to_string(g_sessionState), stateChangedEvent.session, stateChangedEvent.time)
      << std::endl;
  }

  if ((stateChangedEvent.session != XR_NULL_HANDLE) && (stateChangedEvent.session != g_session)) {
    std::cerr << "XrEventDataSessionStateChanged for unknown session" << std::endl;
    return;
  }

  switch (g_sessionState) {
  case XR_SESSION_STATE_READY: {
    CHECK(g_session != XR_NULL_HANDLE);
    XrSessionBeginInfo sessionBeginInfo{ XR_TYPE_SESSION_BEGIN_INFO };
    sessionBeginInfo.primaryViewConfigurationType = g_viewConfigType;
    CHECK_XRCMD(xrBeginSession(g_session, &sessionBeginInfo));
    g_sessionRunning = true;
    break;
  }
  case XR_SESSION_STATE_STOPPING: {
    CHECK(g_session != XR_NULL_HANDLE);
    g_sessionRunning = false;
    CHECK_XRCMD(xrEndSession(g_session))
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
      if (g_verbosity > 0) std::cerr << Fmt("XrEventDataInstanceLossPending by %lld", instanceLossPending.lossTime) << std::endl;
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
      if (g_verbosity >= 2) std::cout << Fmt("Ignoring event type %d", event->type) << std::endl;
      break;
    }
    }
  }
}

void asdp::render::DisplayOpenXR::DisplayOpenXRImpl::OpenXRPollActions()
{
  g_input.handActive = { XR_FALSE, XR_FALSE };

  // Sync actions
  const XrActiveActionSet activeActionSet{ g_input.actionSet, XR_NULL_PATH };
  XrActionsSyncInfo syncInfo{ XR_TYPE_ACTIONS_SYNC_INFO };
  syncInfo.countActiveActionSets = 1;
  syncInfo.activeActionSets = &activeActionSet;
  CHECK_XRCMD(xrSyncActions(g_session, &syncInfo));

  // Get pose and grab action state and start haptic vibrate when hand is 90% squeezed.
  for (auto hand : { Side::LEFT, Side::RIGHT }) {
    XrActionStateGetInfo getInfo{ XR_TYPE_ACTION_STATE_GET_INFO };
    getInfo.action = g_input.grabAction;
    getInfo.subactionPath = g_input.handSubactionPath[hand];

    XrActionStateFloat grabValue{ XR_TYPE_ACTION_STATE_FLOAT };
    CHECK_XRCMD(xrGetActionStateFloat(g_session, &getInfo, &grabValue));
    if (grabValue.isActive == XR_TRUE) {
      // Scale the rendered hand by 1.0f (open) to 0.5f (fully squeezed).
      g_input.handScale[hand] = 1.0f - 0.5f * grabValue.currentState;
      if (grabValue.currentState > 0.9f) {
        XrHapticVibration vibration{ XR_TYPE_HAPTIC_VIBRATION };
        vibration.amplitude = 0.5;
        vibration.duration = XR_MIN_HAPTIC_DURATION;
        vibration.frequency = XR_FREQUENCY_UNSPECIFIED;

        XrHapticActionInfo hapticActionInfo{ XR_TYPE_HAPTIC_ACTION_INFO };
        hapticActionInfo.action = g_input.vibrateAction;
        hapticActionInfo.subactionPath = g_input.handSubactionPath[hand];
        CHECK_XRCMD(xrApplyHapticFeedback(g_session, &hapticActionInfo, (XrHapticBaseHeader*)&vibration));
      }
    }

    getInfo.action = g_input.poseAction;
    XrActionStatePose poseState{ XR_TYPE_ACTION_STATE_POSE };
    CHECK_XRCMD(xrGetActionStatePose(g_session, &getInfo, &poseState));
    g_input.handActive[hand] = poseState.isActive;
  }

  // There were no subaction paths specified for the quit action, because we don't care which hand did it.
  XrActionStateGetInfo getInfo{ XR_TYPE_ACTION_STATE_GET_INFO, nullptr, g_input.quitAction, XR_NULL_PATH };
  XrActionStateBoolean quitValue{ XR_TYPE_ACTION_STATE_BOOLEAN };
  CHECK_XRCMD(xrGetActionStateBoolean(g_session, &getInfo, &quitValue));
  if ((quitValue.isActive == XR_TRUE) && (quitValue.changedSinceLastSync == XR_TRUE) && (quitValue.currentState == XR_TRUE)) {
    CHECK_XRCMD(xrRequestExitSession(g_session));
  }
}

// Function to convert a quaternion to Euler angles (X, Y, Z)
/// @todo Test this function.
static void QuaternionToEulerXYZDegrees(const XrQuaternionf& q, float& roll, float& pitch, float& yaw) {
  // Calculate the Euler angles
  float sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
  float cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
  roll = std::atan2(sinr_cosp, cosr_cosp);

  float sinp = 2 * (q.w * q.y - q.z * q.x);
  if (std::abs(sinp) >= 1)
    pitch = std::copysign(M_PI / 2, sinp); // Use 90 degrees if out of range
  else
    pitch = std::asin(sinp);

  float siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  float cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  yaw = std::atan2(siny_cosp, cosy_cosp);
  roll = glm::degrees(roll);
  pitch = glm::degrees(pitch);
  yaw = glm::degrees(yaw);
}

bool asdp::render::DisplayOpenXR::DisplayOpenXRImpl::OpenXRRenderLayer(XrTime predictedDisplayTime,
  std::vector<XrCompositionLayerProjectionView>& projectionLayerViews, XrCompositionLayerProjection& layer)
{
  XrResult res;

  XrViewState viewState{ XR_TYPE_VIEW_STATE };
  uint32_t viewCapacityInput = (uint32_t)g_views.size();
  uint32_t viewCountOutput;

  XrViewLocateInfo viewLocateInfo{ XR_TYPE_VIEW_LOCATE_INFO };
  viewLocateInfo.viewConfigurationType = g_viewConfigType;
  viewLocateInfo.displayTime = predictedDisplayTime;
  viewLocateInfo.space = g_appSpace;

  res = xrLocateViews(g_session, &viewLocateInfo, &viewState, viewCapacityInput, &viewCountOutput, g_views.data());
  CHECK_XRRESULT(res, "xrLocateViews");
  if ((viewState.viewStateFlags & XR_VIEW_STATE_POSITION_VALID_BIT) == 0 ||
    (viewState.viewStateFlags & XR_VIEW_STATE_ORIENTATION_VALID_BIT) == 0) {
    return false;  // There is no valid tracking poses for the views.
  }

  CHECK(viewCountOutput == viewCapacityInput);
  CHECK(viewCountOutput == g_configViews.size());
  CHECK(viewCountOutput == g_swapchains.size());

  projectionLayerViews.resize(viewCountOutput);

  // Describe all of the views and then render them.  This replaces the OpenXRRenderView() function
  // from the original sample with using the Composite to render.
  std::vector<ViewRenderInfo> viewRenderInfos;

  // Grab the swapchains and fill in the viewRenderInfos.
  for (uint32_t i = 0; i < viewCountOutput; i++) {
    // Fill in the information on the projection layer views.
    projectionLayerViews[i] = { XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW };
    projectionLayerViews[i].pose = g_views[i].pose;
    projectionLayerViews[i].fov = g_views[i].fov;
    projectionLayerViews[i].subImage.swapchain = g_swapchains[i].handle;
    projectionLayerViews[i].subImage.imageRect.offset = { 0, 0 };
    projectionLayerViews[i].subImage.imageRect.extent = { g_swapchains[i].width, g_swapchains[i].height };

    // Acquire a swapchain image for the current view.
    XrSwapchainImageAcquireInfo acquireInfo{ XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO };
    uint32_t swapchainImageIndex;
    CHECK_XRCMD(xrAcquireSwapchainImage(g_swapchains[i].handle, &acquireInfo, &swapchainImageIndex));
    XrSwapchainImageWaitInfo waitInfo{ XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO };
    waitInfo.timeout = XR_INFINITE_DURATION;
    CHECK_XRCMD(xrWaitSwapchainImage(g_swapchains[i].handle, &waitInfo));

    // Construct the ViewRenderInfo for the current view and push it onto the vector.
    ViewRenderInfo vri;
    vri.viewpoint[0] = g_views[i].pose.position.x;
    vri.viewpoint[1] = g_views[i].pose.position.y;
    vri.viewpoint[2] = g_views[i].pose.position.z;
    QuaternionToEulerXYZDegrees(g_views[i].pose.orientation, vri.orientation[0], vri.orientation[1], vri.orientation[2]);
    vri.leftHalfFOV = glm::degrees(g_views[i].fov.angleLeft);
    vri.rightHalfFOV = glm::degrees(g_views[i].fov.angleRight);
    vri.topHalfFOV = glm::degrees(g_views[i].fov.angleUp);
    vri.bottomHalfFOV = glm::degrees(g_views[i].fov.angleDown);
    vri.nearClip = 0.5;         /// @todo See if we get this from somewhere
    vri.farClip = 1000.0;       /// @todo See if we get this from somewhere
    vri.frameBuffer = g_swapchainFramebuffer;
    vri.colorBuffer = reinterpret_cast<const XrSwapchainImageOpenGLKHR*>(
      g_swapchainImages[g_swapchains[i].handle][swapchainImageIndex])->image;
    vri.depthBuffer = OpenGLGetDepthTexture(vri.colorBuffer);
    vri.x = projectionLayerViews[i].subImage.imageRect.offset.x;
    vri.y = projectionLayerViews[i].subImage.imageRect.offset.y;
    vri.width = projectionLayerViews[i].subImage.imageRect.extent.width;
    vri.height = projectionLayerViews[i].subImage.imageRect.extent.height;
    
    viewRenderInfos.push_back(vri);
  }

  /// @todo Render the requested views at the predicted scan-out time.
  std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
  std::shared_ptr<Timer> timer;
  Status status = m_display->m_client->GetTimer(timer);
  if (status != OKAY) {
    return false;
  }
  Time time;
  status = timer->GetCoreTime(time);
  if (status != OKAY) {
    return false;
  }
  m_display->m_composite->Render(time, viewRenderInfos);

  /// Release the swapchain images after rendering.
  for (uint32_t i = 0; i < viewCountOutput; i++) {
    XrSwapchainImageReleaseInfo releaseInfo{ XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO };
    CHECK_XRCMD(xrReleaseSwapchainImage(g_swapchains[i].handle, &releaseInfo));
  }

  layer.space = g_appSpace;
  layer.viewCount = (uint32_t)projectionLayerViews.size();
  layer.views = projectionLayerViews.data();
  return true;
}

void asdp::render::DisplayOpenXR::DisplayOpenXRImpl::OpenXRRenderFrame()
{
  CHECK(g_session != XR_NULL_HANDLE);

  XrFrameWaitInfo frameWaitInfo{ XR_TYPE_FRAME_WAIT_INFO };
  XrFrameState frameState{ XR_TYPE_FRAME_STATE };
  CHECK_XRCMD(xrWaitFrame(g_session, &frameWaitInfo, &frameState));

  XrFrameBeginInfo frameBeginInfo{ XR_TYPE_FRAME_BEGIN_INFO };
  CHECK_XRCMD(xrBeginFrame(g_session, &frameBeginInfo));

  std::vector<XrCompositionLayerBaseHeader*> layers;
  XrCompositionLayerProjection layer{ XR_TYPE_COMPOSITION_LAYER_PROJECTION };
  std::vector<XrCompositionLayerProjectionView> projectionLayerViews;
  if (frameState.shouldRender == XR_TRUE) {
    if (OpenXRRenderLayer(frameState.predictedDisplayTime, projectionLayerViews, layer)) {
      layers.push_back(reinterpret_cast<XrCompositionLayerBaseHeader*>(&layer));
    }
  }

  XrFrameEndInfo frameEndInfo{ XR_TYPE_FRAME_END_INFO };
  frameEndInfo.displayTime = frameState.predictedDisplayTime;
  frameEndInfo.environmentBlendMode = g_environmentBlendMode;
  frameEndInfo.layerCount = (uint32_t)layers.size();
  frameEndInfo.layers = layers.data();
  CHECK_XRCMD(xrEndFrame(g_session, &frameEndInfo));
}
void asdp::render::DisplayOpenXR::DisplayOpenXRImpl::OpenGLInitializeResources()
{
  glGenFramebuffers(1, &g_swapchainFramebuffer);
}

uint32_t asdp::render::DisplayOpenXR::DisplayOpenXRImpl::OpenGLGetDepthTexture(uint32_t colorTexture)
{
  // If a depth-stencil view has already been created for this back-buffer, use it.
  auto depthBufferIt = g_colorToDepthMap.find(colorTexture);
  if (depthBufferIt != g_colorToDepthMap.end()) {
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

  g_colorToDepthMap.insert(std::make_pair(colorTexture, depthTexture));

  return depthTexture;
}

void asdp::render::DisplayOpenXR::DisplayOpenXRImpl::OpenGLTearDown()
{
  if (g_swapchainFramebuffer != 0) {
    glDeleteFramebuffers(1, &g_swapchainFramebuffer);
  }

  for (auto& colorToDepth : g_colorToDepthMap) {
    if (colorToDepth.second != 0) {
      glDeleteTextures(1, &colorToDepth.second);
    }
  }
}

void asdp::render::DisplayOpenXR::DisplayOpenXRImpl::OpenXRTearDown()
{
  OpenGLTearDown();

  if (g_input.actionSet != XR_NULL_HANDLE) {
    for (auto hand : { Side::LEFT, Side::RIGHT }) {
      xrDestroySpace(g_input.handSpace[hand]);
    }
    xrDestroyActionSet(g_input.actionSet);
  }

  for (Swapchain swapchain : g_swapchains) {
    xrDestroySwapchain(swapchain.handle);
  }

  if (g_appSpace != XR_NULL_HANDLE) {
    xrDestroySpace(g_appSpace);
  }

  if (g_session != XR_NULL_HANDLE) {
    xrDestroySession(g_session);
  }

  if (g_instance != XR_NULL_HANDLE) {
    xrDestroyInstance(g_instance);
  }

#ifdef XR_USE_PLATFORM_WIN32
  CoUninitialize();
#endif
}

DisplayOpenXR::DisplayOpenXR(std::shared_ptr<Composite> composite, Display* sharedWindow,
    std::shared_ptr<CoreClient> client, uint8_t triggerID, uint32_t triggerAheadMicroseconds,
    uint32_t renderAheadMicroseconds, int verbosity)
  : Display(composite, client, triggerID, triggerAheadMicroseconds)
{
  m_impl = std::make_unique<DisplayOpenXRImpl>(this);
  m_impl->g_verbosity = verbosity;

  // Start the rendering thread.
  m_displayThread = std::thread(&DisplayOpenXR::DisplayThread, this, sharedWindow, renderAheadMicroseconds);

  // Wait until either the context is ready or there has been a failure so that the
  // constructor does not return before the rendering thread is ready.
  while (!Display::m_impl->m_contextAvailable && (m_status == "")) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
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

    /// @todo Create things that we need
    try {
      m_impl->OpenXRCreateInstance();
      m_impl->OpenXRInitializeSystem(sharedWindow);
      m_impl->OpenXRInitializeSession();
      m_impl->OpenXRCreateSwapchains();
    } catch (const std::exception& e) {
      m_status = e.what();
    }

    // After we're done with the context for set-up and have released it, indicate that the context is available
    // for borrowing.
    Display::m_impl->m_contextAvailable = true;

    while (m_status.empty()) {
      bool exitRenderLoop = false;
      try {
        m_impl->OpenXRPollEvents(&exitRenderLoop, &requestRestart);
      } catch (const std::exception& e) {
        m_status = e.what();
        continue;
      }
      if (exitRenderLoop) {
        break;
      }

      /// Handle any actions and render the frame

      if (m_impl->g_sessionRunning) {
        try {
          m_impl->OpenXRPollActions();
          m_impl->OpenXRRenderFrame();
        } catch (const std::exception& e) {
          m_status = e.what();
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
      m_status = e.what();
    }

  } while (m_status.empty() && requestRestart);
}

#else // USE_OPENXR

class asdp::render::DisplayOpenXR::DisplayOpenXRImpl {
public:
}

DisplayOpenXR::DisplayOpenXR(std::shared_ptr<Composite> composite, Display* sharedWindow,
    std::shared_ptr<CoreClient> client, uint8_t triggerID, uint32_t triggerAheadMicroseconds,
    uint32_t renderAheadMicroseconds, int verbosity)
  : Display(composite, client, triggerID, triggerAheadMicroseconds)
{
  m_status = "OpenXR is not compiled in.";
}

DisplayOpenXR::~DisplayOpenXR()
{
}

void DisplayOpenXR::DisplayThread(Display* sharedWindow, uint32_t renderAheadMicroseconds)
{
}

#endif // USE_OPENXR
