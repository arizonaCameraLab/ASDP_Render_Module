/*
 * Copyright (C) 2025: Arizona Board of Regents on Behalf of the University of Arizona
 */

 /**
  * @file RenderText.h
  * @brief Apache Strap-Down Pilotage Render header file for rendering text.
  *
 * @author ReliaSolve.
 * @date November 20, 2025.
 */

#pragma once

#include <string>
#include <memory>

namespace asdp {
  namespace render {

    class RenderText {
    public:
      /// @brief Constructor
      /// @param windowWidth The width of the window in pixels.
      /// @param windowHeight The height of the window in pixels.
      /// @details Note: An OpenGL context must be current when this is called.
      RenderText(int windowWidth, int windowHeight);

      /// @brief If the window size changes, this updates the internal parameters.
      /// @param windowWidth The new width of the window in pixels.
      /// @param windowHeight The new height of the window in pixels.
      void SetWindowSize(int windowWidth, int windowHeight);

      /// @brief Render the given text at the given position and color.
      /// @param text The text to render. May include newline characters.
      /// @param x The x position in normalized device coordinates (-1 to 1).
      /// @param y The y position in normalized device coordinates (-1 to 1).
      /// @param r The red color component (0 to 1).
      /// @param g The green color component (0 to 1).
      /// @param b The blue color component (0 to 1).
      std::string Draw(std::string text, float x, float y, float r, float g, float b);

    private:
      /// @brief Implementation class that hides details and #include files
      class Impl;
      std::shared_ptr<Impl> m_impl;
    };

  } // namespace render
} // namespace asdp
