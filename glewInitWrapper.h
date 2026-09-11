/*
 * Copyright (C) 2026: Arizona Board of Regents on Behalf of the University of Arizona
 */

 /**
  * @file glewInitWrapper.h
  * @brief Apache Strap-Down Pilotage Render header file for initializing GLEW.
  * @details A file that includes this must have already included glew.h.
  *
 * @author ReliaSolve.
 * @date September 11, 2026.
 */

#pragma once

#include <string>

namespace asdp {
  namespace render {

    /// @brief Call glewInit() on Windows, Linux/X11 or Linux/Wayland.
    /// @return Empty string on success, error that can be printed on failure.
    inline std::string glewInitWrapper()
    {
      glewExperimental = true;
      GLenum ret = glewInit();
      if (ret != GLEW_OK) {
#if defined(__linux__)
        // Under native Wayland, GLEW fails because GLX is not available.
        // If it fails with this specific error, then the OpenGL extensions have been loaded.
        if (ret == GLEW_ERROR_NO_GLX_DISPLAY) {
          return "";
        }
#endif
        return reinterpret_cast<char const*>(glewGetErrorString(ret));
      }
      return "";
    }

  } // namespace analysis
} // namespace asdp
