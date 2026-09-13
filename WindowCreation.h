/*
 * Copyright (C) 2026: Arizona Board of Regents on Behalf of the University of Arizona
 */

 /**
  * @file WindowCreation.h
  * @brief Apache Strap-Down Pilotage window-creation header file.
  *
 * @author ReliaSolve.
 * @date September 12, 2026.
 */

#pragma once
#include <memory>
#include <string>
#include <glad/gl.h>
#include <GLFW/glfw3.h>

namespace asdp {
  namespace render {

    /// @brief Create a GLFW window or hidden GL context and return a shared pointer to it.
    /// @details This function initializes GLFW for the first window created and
    /// terminates GLFW when the last window is destroyed. It creates an OpenGL context
    /// and initializes GLAD in its context. It may create a windowless context and it may
    /// create a full-screen window. The returned shared pointer will automatically destroy the window
    /// when it is destroyed, and if it is the last window, GLFW will be terminated.
    /// This function can only be called from the main thread, as GLFW requires that all
    /// window creation and event handling must be on that thread.
    /// @param window A shared pointer to the created GLFW window.  When this pointer is destroyed,
    /// the window will be destroyed and GLFW will be terminated if this is the last window.
    /// It returns a nullptr and returns an error message if the window could not be created.
    /// @param width Width of the window.
    /// @param height Height of the window.
    /// @param title Title of the window.
    /// @param monitor Monitor to use for full-screen mode (default is nullptr for windowed
    /// mode).
    /// @param sharedWindow Window to share resources with (default is nullptr for no sharing).
    /// NOTE: The caller must ensure that the shared window's context is current on this thread
    /// before calling this function if it is to be shared. This function does not make the
    /// shared window's context current.
    /// @param fullScreenDisplay The display index for full-screen mode (default is -1 for windowed mode).
    /// @param hidden Whether the window should be initially hidden (default is false).
    /// @param sRGBCapable Whether the window should be sRGB capable (default is true).
    /// @return An error message if the window could not be created, or an empty string on success.
    std::string CreateWindowOrContext(std::shared_ptr<GLFWwindow> &window,
      int width, int height, const std::string& title,
      GLFWmonitor* monitor = nullptr, GLFWwindow* sharedWindow = nullptr,
      int fullScreenDisplay = -1,
      bool hidden = false, bool sRGBCapable = true);

  } // namespace render
} // namespace asdp
