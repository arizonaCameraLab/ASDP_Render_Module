/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

 /**
  * @file Composite.h
  * @brief Apache Strap-Down Pilotage Render/Composite submodule header file.
  *
 * @author ReliaSolve.
 * @date April 26, 2024.
 */

#pragma once

#include <string>
#include <array>
#include <vector>
#include <cstdint>
#ifdef WIN32
#include <windows.h>
#endif
#include <GL/gl.h>
#include <ASDP_Core_API.h>

namespace asdp {
  namespace render {

    /// @brief Information about a single camera needed to produce a renderable view from it.
    struct CameraRenderInfo {
      uint16_t m_ID = 0;                              ///< ID of the camera.
      std::array<double, 3> m_positionMeters = {};    ///< Position of the camera in meters from the camera device origin.
      /// Orientation of the camera in degrees, Euler rotation around X, then Y, then Z.
      /// The canonical orientation is a bit unusual, with the camera rotated to portrait mode
      /// and looking along +Y: X down, Y to the right, Z towards -Y.  This makes it convenient because
      /// the cameras are aligned this way in the system and this order of rotations specifies the
      /// direction of the view and then rotation around the view direction.
      std::array<double, 3> m_orientationRad = {};
      std::array<uint16_t, 2> m_resolutionPixels = {};///< Resolution of the camera in pixels.
      std::array<double, 2> m_fovDegrees = {};        ///< Field of view of the camera in degrees, horizontal then vertical.
      /// Distortion coefficients of the camera. @todo Describe the format.
      std::vector<double> m_distortion;
    };

    /// @brief Information about the rendering of a single viewpoint, enabling multiple views to be requested at the same time.
    struct ViewRenderInfo {
      float* modelViewProjection;   ///< 4x4 Model view projection matrix.
      GLuint frameBuffer;           ///< Frame buffer to render into.
      GLuint color;                 ///< Texture for color to be rendered into (will be bound to the frameBuffer).
      GLuint depth;                 ///< Depth buffer to be rendered into, 0 for no depth buffer (will be bound to the frameBuffer).
      GLint x;                      ///< X coordinate of the lower left corner of the viewport.
      GLint y;                      ///< Y coordinate of the lower left corner of the viewport.
      GLsizei width;                ///< Width of the viewport.
      GLsizei height;               ///< Height of the viewport.
    };

    /// @brief Composite base class that renders a composite image from multiple cameras.
    class Composite {
    public:
      /// @brief Constructor
      /// @param cameraRenderInfo The configuration of the cameras needed to generate textured geometry.
      Composite(std::vector<CameraRenderInfo>& cameraRenderInfo);

      /// @brief Destructor, virtual so that derived classes can have their destructors called from pointers.
      virtual ~Composite();

      /// @brief Render the composite image from one or more viewpoints.
      /// @details This is a pure virtual function that must be implemented by derived classes.  It does not return
      /// until the rendering is complete and written to the buffers.  It calls SetupFrenderFrame() once per frame.
      /// It calls the RenderView() method to render each viewpoint after it has set up the frame buffer
      /// and cleared the color and depth textures.  It calls TearDownRenderFrame() after all views.
      /// @param scanOutTime The time that the scan out is occurring, in ASDP Core time.  This is the time of the middle of the frame.
      /// @param views A vector of RenderInfo structures that contain the information about the views to render.
      virtual void Render(asdp::Time scanOutTime, std::vector<ViewRenderInfo> views);

      /// @todo

    protected:

      /// Information about the cameras, filled in by the constructor.
      std::vector<CameraRenderInfo> m_cameraRenderInfos;

      /// @brief Render the geometry for a particular view, assuming all parameters set up.
      /// @details This is a pure virtual function that must be implemented by derived classes.
      /// The Render() method calls it after setting up and clearing the frame buffer color and
      /// depth textures and binding the frame buffer.  RenderView() is responsible for setting
      /// the program and the matrix parameter for it.
      /// @param modelViewProjection The matrix specifying the entire viewing transformation to use.
      virtual void RenderView(float* modelViewProjection) = 0;

      /// @brief Set up state needed for rendering, perhaps including the shader program and geometry/textures.
      virtual void SetupRenderFrame(asdp::Time scanOutTime) = 0;

      /// @brief Tear down state needed for rendering.
      virtual void TearDownRenderFrame() = 0;
    };

    /// @brief Composite class that renders a cube rather than camera views.  Useful for debugging displays.
    class CompositeCube : public Composite {
    public:
      /// @brief Constructor
      /// @param radius Half the length of one side of the cube.
      CompositeCube(double radius);

      /// @brief Destructor
      ~CompositeCube();

    protected:
      /// @brief The radius of the cube.
      double m_radius;

      /// @brief The OpenGL program ID.
      GLuint m_programId = 0;
      /// @brief The Uniform ID of the modelview-projection matrix.
      GLuint m_modelViewProjectionUniformId = 0;

      void checkShaderError(GLuint shaderId, const std::string& exceptionMsg);
      void checkProgramError(GLuint programId, const std::string& exceptionMsg);

      GLuint m_colorBuffer = 0;
      GLuint m_vertexBuffer = 0;
      GLuint m_vertexArrayId = 0;
      std::vector<GLfloat> m_colorBufferData;
      std::vector<GLfloat> m_vertexBufferData;

      class MeshCube;
      MeshCube* m_roomCube = nullptr;

      void RenderView(float* modelViewProjection) override;
      void SetupRenderFrame(asdp::Time scanOutTime) override;
      void TearDownRenderFrame() override;
    };

  } // namespace render
} // namespace asdp
