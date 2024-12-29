/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

 /**
  * @file ToneMap.h
  * @brief Apache Strap-Down Pilotage Render/Display ToneMap header file.
  *
 * @author ReliaSolve.
 * @date May 29, 2024.
 */

#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>
#include <array>

namespace asdp {
  namespace render {

    /// @brief Structure holding an entry in the piecewise-linear tone map.
    struct ToneMapEntry {
      /// @brief Constructor
      /// @param texCoord The texture coordinate for this value between 0 and 1.
      /// @param red The red output value for this entry.
      /// @param green The green output value for this entry.
      /// @param blue The blue output value for this entry.
      ToneMapEntry(float texCoord, float red, float green, float blue)
        : texCoord(texCoord), red(red), green(green), blue(blue) { }
      float texCoord;       ///< Texture coordinate for this value between 0 and 1.
      float red;            ///< Red output value for this entry.
      float green;          ///< Green output value for this entry.
      float blue;           ///< Blue output value for this entry.
    };

    /// @brief ToneMap base class that constructs 1D texture (intensity) to color maps and writes them to textures.
    class ToneMap {
    public:
      /// @brief Constructor
      /// @details Constructs a piecewise-linear tone map from the given entries.
      /// @param mapping The mapping from intensity to color.  For example, a straight mapping from
      /// black to white would use { {0.0, 0.0, 0.0, 0.0}, {1.0, 1.0, 1.0, 1.0} }.  Passing an empty
      /// vector will create a default mapping from black to white.
      /// @param numEntries The number of entries in the mapping.  All OpenGL implementations must
      /// support at least 1024 entries, so that is the default value.
      ToneMap(std::vector<ToneMapEntry> mapping = {}, size_t numEntries = 1024);

      /// @brief Destructor, virtual so that derived classes can have their destructors called from pointers.
      virtual ~ToneMap() = default;

      /// @brief Generate a 1D OpenGL texture and fill it in with the mapping.
      /// @details The caller is responsible for making sure that the OpenGL context is current
      /// on the calling thread before calling this function.  The caller is responsible for deleting
      /// the texture when it is no longer needed.
      /// @return The OpenGL texture ID, 0 on failure.
      uint32_t GenerateTexture() const;

      /// @brief Fill in the texture with the mapping.
      /// @details The caller is responsible for making sure that the OpenGL context is current
      /// on the calling thread before calling this function.
      /// @param textureID The OpenGL texture ID to fill in.
      /// @return True on success, false on failure.
      bool FillTexture(uint32_t textureID) const;

    protected:
      std::vector< std::array<float, 3> >   m_mapping;  ///< The mapping from intensity to color.
    };

    /// @brief Specialized tone map that uses the black-body radiation spectrum.
    class ToneMapBlackbody : public ToneMap {
    public:
      /// @brief Constructor
      /// @param numEntries The number of entries in the mapping.
      ToneMapBlackbody(size_t numEntries = 1024);
    };

    /// @brief Specialized tone map that stretches half the view to blue/cold sky and other half blackbody.
    class ToneMapBlueSky : public ToneMap {
    public:
      /// @brief Constructor
      /// @param numEntries The number of entries in the mapping.
      ToneMapBlueSky(size_t numEntries = 1024);
    };

    /// @brief Monochrome tone map with a specified gamma.
    class ToneMapGamma : public ToneMap {
    public:
      /// @brief Constructor
      /// @param red The fraction of red to use in the brightest color (range 0-1).
      /// @param green The fraction of green to use in the brightest color (range 0-1).
      /// @param blue The fraction of blue to use in the brightest color (range 0-1).
      /// @param gamma The gamma value to use for the tone map.  This should be lower than
      /// 1 to highlight bright areas and higher than 1 to highlight dark areas.
      /// @param numEntries The number of entries in the mapping.
      ToneMapGamma(float gamma, float red = 1, float green = 1, float blue = 1, size_t numEntries = 1024);
    };


  } // namespace render
} // namespace asdp
