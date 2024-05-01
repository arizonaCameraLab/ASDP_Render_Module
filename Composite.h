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
#include <memory>
#ifdef WIN32
#include <windows.h>
#endif
#include <GL/gl.h>
#include <ASDP_Core_API.h>
#include <ImageQueue.h>

namespace asdp {
  namespace render {

    /// @brief Information about a single camera needed to produce a renderable view from it.
    struct CameraRenderInfo {
      uint16_t m_ID = 0;                              ///< ID of the camera.
      /// Position of the cameras center of projection in meters from the camera device origin.
      /// The canonical orientation is in the local helicopter coordinate system, with +X pointing
      /// right, +Y pointing forwards, and +Z pointing up.  The camera is translated first and then
      /// rotated around its new center.
      std::array<double, 3> m_positionMeters = {};
      /// Orientation of the camera in degrees, Euler rotation around X, then Y, then Z.
      /// The canonical orientation is in the local helicopter coordinate system, with +X pointing
      /// right, +Y pointing forwards, and +Z pointing up.  The camera is translated first and then
      /// rotated around its new center.
      std::array<double, 3> m_orientationDegrees = {};
      std::array<uint16_t, 2> m_resolutionPixels = {};///< Resolution of the camera in pixels.
      std::array<double, 2> m_fovDegrees = {};        ///< Field of view of the camera in degrees, horizontal then vertical.
      /// Distortion coefficients of the camera. @todo Describe the format.
      std::vector<double> m_distortion;
      /// Queue of images from the camera.  The newest image is the one to render.
      std::shared_ptr<asdp::render::ImageQueue> m_imageQueue;
    };

    /// @brief Information about the rendering of a single viewpoint, enabling multiple views to be requested at the same time.
    struct ViewRenderInfo {
      /// Position of the viewpoint in meters from the camera device origin.
      /// Specifies the center of the view frustum in the camera coordinate system.
      /// The canonical orientation is in the local helicopter coordinate system, with +X pointing
      /// right, +Y pointing forwards, and +Z pointing up.  The camera is translated first and then
      /// rotated around its new center.
      std::array<float, 3> viewpoint = {};
      /// Orientation of the viewpoint in degrees, Euler rotation around X, then Y around the
      /// new Y, then around the new Z.
      /// The canonical orientation is in the local helicopter coordinate system, with +X pointing
      /// right, +Y pointing forwards, and +Z pointing up.  The camera is translated first and then
      /// rotated around its new center.
      std::array<float, 3> orientation = {};
      /// Left edge of the view in degrees from the principal ray (this will be half the horizontal FOV).
      /// Left and right are different for off-center projection.
      float leftHalfFOV = -45;
      /// Right edge of the view in degrees from the principal ray (this will be half the horizontal FOV).
      /// Left and right are different for off-center projection.
      float rightHalfFOV = 45;
      /// Top edge of the view in degrees from the principal ray (this will be half the vertical FOV).
      /// Top and bottom are different for off-center projection.
      float topHalfFOV = 45;
      /// Bottom edge of the view in degrees from the principal ray (this will be half the vertical FOV).
      /// Top and bottom are different for off-center projection.
      float bottomHalfFOV = -45;
      float nearClip = 0.1;                 ///< Near clipping plane in meters.
      float farClip = 1000;                 ///< Far clipping plane in meters.
      GLuint frameBuffer = 0;               ///< Frame buffer to render into.  Set to 0 for the default frame buffer.
      GLuint colorBuffer = 0;               ///< Texture for color to be rendered into (will be bound to the frameBuffer). Ignored for frameBuffer 0.
      GLuint depthBuffer = 0;               ///< Depth buffer to be rendered into, 0 for no depth buffer (will be bound to the frameBuffer). Ignored for frameBuffer 0.
      GLint x = 0;                          ///< X coordinate of the lower left corner of the viewport.
      GLint y = 0;                          ///< Y coordinate of the lower left corner of the viewport.
      GLsizei width = 0;                    ///< Width of the viewport.
      GLsizei height = 0;                   ///< Height of the viewport.
    };

    /// @brief Composite base class that renders a composite image from multiple cameras.
    class Composite {
    public:
      /// @brief Constructor
      /// @details The OpenGL context to be used by the Composite object must be current when this
      /// constructor is called.
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
      virtual void RenderView(const float* modelViewProjection) = 0;

      /// @brief Set up state needed for rendering, perhaps including the shader program and geometry/textures.
      virtual void SetupRenderFrame(asdp::Time scanOutTime) = 0;

      /// @brief Tear down state needed for rendering.
      virtual void TearDownRenderFrame() = 0;

      static void checkShaderError(GLuint shaderId, const std::string& exceptionMsg);
      static void checkProgramError(GLuint programId, const std::string& exceptionMsg);
    };

    /// @brief Composite class that renders a cube rather than camera views.
    /// @details This class does not make use of image data, rendering a stationary cube.
    /// It is useful for testing the mechanics of the rendering pipeline.
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

      class MeshCube;
      std::shared_ptr<MeshCube> m_roomCube;

      void RenderView(const float* modelViewProjection) override;
      void SetupRenderFrame(asdp::Time scanOutTime) override;
      void TearDownRenderFrame() override;
    };

    /// @brief Composite class that renders a set of camera views.
    /// @details This is the class that is most likely to be used in
    /// an application.
    class CompositeCameras : public Composite {
    public:
      /// @brief Constructor
      /// @param cameraRenderInfo The configuration of the cameras needed to generate textured geometry.
      CompositeCameras(std::vector<CameraRenderInfo>& cameraRenderInfo);

      /// @brief Destructor
      ~CompositeCameras();

    protected:
      /// @brief The OpenGL program ID.
      GLuint m_programId = 0;

      /// @brief The Uniform ID of the modelview-projection matrix.
      GLuint m_modelViewProjectionUniformId = 0;

      /// @brief Vector of Image objects to use during a frame rendering, one per camera.
      std::vector<std::shared_ptr<asdp::render::ImageData>> m_images;

      /// @brief Vector of vertex buffer objects to hold the position + texture coords for each camera.
      std::vector<GLuint> m_vertexBufferObjects;

      /// @brief Vector of vertex buffer objects to hold the indices for each camera.
      std::vector<GLuint> m_indexBufferObjects;

      /// @brief Vector of number of elements for the element buffer objects for each camera.
      std::vector<GLsizei> m_numIndices;

      /// @brief Vector of vertex array objects for each camera.
      std::vector<GLuint> m_vertexArrayObjects;

      /// @brief Add the buffer objects for a camera to the OpenGL context and store the IDs.
      /// @details This function creates the buffer objects for the camera and stores the IDs
      /// in the m_vertexBufferObjects, m_indexBufferObjects, and m_vertexArrayObjects vectors.
      /// It also stores the number of elements in the m_numIndices vector.
      /// @param cameraRenderInfo The camera to add the buffer objects for.
      /// @param nx The number of vertices in the X direction.
      /// @param ny The number of vertices in the Y direction.
      /// @param z The distance from the camera to the quadrilateral displaying the image.
      void AddBufferObjects(const CameraRenderInfo& cameraRenderInfo, size_t nx = 10, size_t ny = 10,
        GLfloat depth = 10);

      // Overridden methods
      void RenderView(const float* modelViewProjection) override;
      void SetupRenderFrame(asdp::Time scanOutTime) override;
      void TearDownRenderFrame() override;
    };

  } // namespace render
} // namespace asdp
