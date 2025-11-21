/*
 * Copyright (C) 2025: Arizona Board of Regents on Behalf of the University of Arizona
 */

 /**
  * @file RenderHaloedLines.h
  * @brief Apache Strap-Down Pilotage Render header file for rendering lines with halos.
  *
 * @author ReliaSolve.
 * @date November 21, 2025.
 */

#pragma once

#include <memory>
#include <vector>
#include <array>
#include <string>

namespace asdp {
  namespace render {

    class RenderHaloedLines {
    public:
      /// @brief Constructor
      /// @details Note: An OpenGL context must be current when this is called.
      RenderHaloedLines();

      /// @brief Render the given lines with halos.
      /// @param lines The lines to render, each defined by two endpoints, with each
      /// point being a 2D coordinate (x, y) in the range [-1..1], normalized device
      /// coordinates.
      /// @param lineWidth The width of the lines.
      /// @param haloWidth The width of the halos.
      /// @param lineColor The color of the lines (RGB).
      /// @param haloColor The color of the halos (RGB).
      /// @param alpha The alpha (transparency) component (0 to 1).
      std::string Draw(std::vector< std::array< std::array<float, 2>, 2> > lines,
        float lineWidth, float haloWidth, std::array<float, 3> lineColor,
        std::array<float, 3> haloColor, float alpha = 1);

    private:
      /// @brief Implementation class that hides details and #include files
      class Impl;
      std::shared_ptr<Impl> m_impl;
    };

  } // namespace render
} // namespace asdp
