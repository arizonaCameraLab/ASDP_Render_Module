/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

 /**
  * @file RenderTimingInfo.h
  * @brief Apache Strap-Down Pilotage Render timing info header file.
  *
 * @author ReliaSolve.
 * @date November 14, 2024.
 */

#pragma once
#include <ASDP_Core_API.h>

namespace asdp {
  namespace render {

    /// @brief Data structure to hold timing information for helping with system tuning.
    struct RenderTimingInfo
    {
      /// @brief Data structure to hold timing information for a single camera.
      struct camera {
        std::vector<std::chrono::steady_clock::time_point> frameBeginTimes;   ///< The times for the begin frame message receipts.
        std::vector<std::chrono::steady_clock::time_point> frameEndTimes;     ///< The times for the end frame message receipts.
        std::vector<std::chrono::steady_clock::time_point> textureTimes;      ///< The times for texture fill complete.
        /// The center time for each of the images used in a render frame.
        std::vector<Time> centerRenderTimes;
      };
      std::vector<camera> cameras;  ///< The timing information for each camera.

      std::vector<std::chrono::steady_clock::time_point> depthStartTimes;   ///< The times for the start of depth calc.
      std::vector<std::chrono::steady_clock::time_point> depthEndTimes;     ///< The times for the end of depth calc.

      std::vector<std::chrono::steady_clock::time_point> renderStartTimes;  ///< The times for the start of rendering.
      std::vector<std::chrono::steady_clock::time_point> renderSubmitTimes; ///< The times for the render frame submission.

      std::chrono::steady_clock::time_point startTime = std::chrono::steady_clock::now(); ///< The start time for the program.

      void SetNumCameras(size_t numCameras)
      {
        cameras.resize(numCameras);
      }
    };

  }
}
