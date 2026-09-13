/*
 * Copyright (C) 2026: Arizona Board of Regents on Behalf of the University of Arizona
 */

 /**
  * @file WindowCreation.cpp
  * @brief Apache Strap-Down Pilotage Render/Display window creation implementation file.
  *
 * @author ReliaSolve.
 * @date September 12, 2026.
 */

#include <thread>
#include <atomic>
#include <mutex>
#include "WindowCreation.h"

using namespace asdp::render;

/// @brief The ID of the main thread, used to ensure that window creation is only done on the main thread.
static std::thread::id main_thread_id;
static std::mutex main_thread_mutex;    ///< Mutex to protect access to the main thread ID.

/// @brief The count of currently open windows, used to determine when to initialize and terminate GLFW.
static std::atomic<int> window_count(0);

std::string asdp::render::CreateWindowOrContext(std::shared_ptr<GLFWwindow>& window, int width, int height,
  const std::string& title, GLFWwindow* sharedWindow, int fullScreenDisplay,
  bool hidden, bool sRGBCapable)
{
  // Clear the shared pointer in case of failure and make a place to store the new window.
  window.reset();
  GLFWwindow* new_window = nullptr;

  // Ensure that this function is called from the main thread.
  // This only ensures that it is always called by the same thread because
  // there is not a cross-platform way to determine the main thread.
  {
    std::lock_guard<std::mutex> lock(main_thread_mutex);
    if (main_thread_id == std::thread::id()) {
      main_thread_id = std::this_thread::get_id();
    } else if (std::this_thread::get_id() != main_thread_id) {
      return "Error: CreateWindow must be called from the main thread.";
    }
  }

  // Initialize GLFW if this is the first window, atomically incrementing the window count
  // to see if we're the first.  Then make a shared_ptr that will decrement the count whenever
  // it leaves focus and then terminate GLFW if it is zero.
  int previous_window_count = window_count.fetch_add(1);
  std::shared_ptr<void> glfw_init_guard(nullptr, [](void*) {
      if (window_count.fetch_sub(1) == 1) {
        glfwTerminate();
      }
    });
  if (previous_window_count == 0) {
    if (!glfwInit()) {
      return "Error: Failed to initialize GLFW.";
    }
  }

  // Set up the GLFW hints for the window creation.

  // Set the window visibility.
  glfwWindowHint(GLFW_VISIBLE, !hidden);
  // Tell it not to iconify full-screen windows that lose focus.
  glfwWindowHint(GLFW_AUTO_ICONIFY, GLFW_FALSE);
  // Set the sRGB capable flag accordingly.
  if (sRGBCapable) {
    glfwWindowHint(GLFW_SRGB_CAPABLE, GLFW_TRUE);
  } else {
    glfwWindowHint(GLFW_SRGB_CAPABLE, GLFW_FALSE);
  }

  // Create the window.
  new_window = glfwCreateWindow(width, height, title.c_str(), nullptr, sharedWindow);
  if (!new_window) {
    return "Error: Failed to create GLFW window.";
  }

  // Determine which full-screen monitor to use, if any.
  if (fullScreenDisplay >= 0) {
    int monitor_count;
    GLFWmonitor** monitors = glfwGetMonitors(&monitor_count);
    if (fullScreenDisplay < monitor_count) {
      GLFWmonitor* monitor = monitors[fullScreenDisplay];
      const GLFWvidmode* mode = glfwGetVideoMode(monitor);
      if (mode) {
        glfwSetWindowMonitor(new_window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
      } else {
        return "Error: Could not get video mode for fullScreenDisplay.";
      }
    } else {
      return "Error: Invalid fullScreenDisplay index.";
    }
  }

  // Set the OpenGL context to the new window and initialize GLAD in that context.
  glfwMakeContextCurrent(new_window);
  if (!gladLoadGL(glfwGetProcAddress)) {
    glfwDestroyWindow(new_window);
    return "Error: Failed to initialize GLAD.";
  }

  // Release our context so that the caller or other threads can make it current if they want to.
  glfwMakeContextCurrent(nullptr);

  // Store the new window in a shared pointer that will destroy it when it is no longer needed
  // and that will decrement the window count when it is destroyed and then will
  // terminate GLFW if this is the last window.
  // First, we bump the window count again so that when the shared pointer above is destroyed,
  // it will not terminate GLFW until this shared pointer is also destroyed.
  window_count.fetch_add(1);
  std::shared_ptr<GLFWwindow> ret(new_window, [](GLFWwindow* w) {
    // Ensure this window's context is not current on this thread before destroying it.
    if (glfwGetCurrentContext() == w) {
      glfwMakeContextCurrent(nullptr);
    }
    glfwDestroyWindow(w);
    if (window_count.fetch_sub(1) == 1) {
      glfwTerminate();
    }
  });
  window = ret;
  return "";
}
