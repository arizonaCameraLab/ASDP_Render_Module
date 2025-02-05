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
#include <map>
#include <cstdint>
#include <memory>
#include <mutex>
#include <atomic>
#ifdef WIN32
#include <windows.h>
#endif
#include <GL/gl.h>
#include <ASDP_Core_API.h>
#include <ImageQueue.h>
#include <Distortion.h>
#include <Vignette.h>
#include <PoseAdjuster.h>
#include <RenderTimingInfo.h>
#include <CameraRenderInfo.h>
#include <RangeEstimator.h>

namespace asdp {
  namespace render {

    /// @brief Information about the rendering of a single viewpoint, enabling multiple views to be requested at the same time.
    struct ViewRenderInfo {
      /// Position of the viewpoint in meters from the camera device origin.
      /// Specifies the center of the view frustum in the camera coordinate system.
      /// The canonical orientation is in the local helicopter coordinate system, with +X pointing
      /// right, +Y pointing forwards, and +Z pointing up.  The camera is translated in the
      /// helicopter frame of reference and then rotated around its new center.  A rotation around
      /// the +X axis will tip the camera's view up, and a rotation around the +Y axis will pan the
      /// camera's view left.
      std::array<float, 3> viewpoint = {};
      /// Orientation of the viewpoint in degrees, Quaternion in (W,X,Y,Z) order.
      /// The canonical orientation is in the local helicopter coordinate system, with +X pointing
      /// right, +Y pointing forwards, and +Z pointing up.  The camera is translated in the
      /// helicopter frame of reference and then rotated around its new center.
      /// The camera is looking out the front of the helicopter (along the +Y axis) with its "up" vector
      /// pointing above the helicopter (along the +Z axis) when the orientation is (1,0,0,0).
      std::array<float, 4> orientation = {1, 0, 0, 0};
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
      float nearClip = 2.0;                 ///< Near clipping plane in meters.
      float farClip = 4000;                 ///< Far clipping plane in meters. Must be > max Composite depth
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

      /// @brief Destructor, virtual so that derived classes can have their destructors called from pointers.
      virtual ~Composite();

      /// @brief Render the composite image from one or more viewpoints.
      /// @details This function does not return until the rendering is complete and written to the buffers.
      /// If initialization has not been done, it calls SetupRendering() to set up the rendering state.
      /// An OpenGL context must be active when this function is called.
      /// It calls SetupRenderFrame() once per frame.
      /// It calls the RenderView() method to render each viewpoint after it has set up the frame buffer
      /// and cleared the color and depth textures.  It calls TearDownRenderFrame() after all views.
      /// @param scanOutTime The time that the scan out is occurring, in ASDP Core time.
      /// This is the time of the middle of the frame.
      /// @param views A vector of RenderInfo structures that contain the information about the views to render.
      virtual void Render(asdp::Time scanOutTime, std::vector<ViewRenderInfo> views);

    protected:

      /// Records whether we've initialized our geometry.
      std::atomic_bool m_initialized {false};

      /// Mutex to control access to initialization.
      std::mutex m_initMutex;

      /// @brief Render the geometry for a particular view, assuming all parameters set up.
      /// @details This is a pure virtual function that must be implemented by derived classes.
      /// The Render() method calls it after setting up and clearing the frame buffer color and
      /// depth textures and binding the frame buffer.  RenderView() is responsible for setting
      /// the program and the matrix parameter for it.
      /// @param scanOutTime The time that the scan out is occurring, in ASDP Core time.  This is the time of the middle of the frame.
      /// @param viewProjection The matrix specifying the transformation from helicopter space
      /// into final projected points.
      virtual void RenderView(asdp::Time scanOutTime, const float* viewProjection) = 0;

      /// @brief Set up state needed for rendering, perhaps including the shader program and geometry/textures.
      /// @details This function is called during the first call to Render().  If it fails, rendering is not
      /// done that frame and it tries again the next.
      virtual bool SetupRendering() = 0;

      /// @brief Set up state needed for rendering, perhaps including the shader program and geometry/textures.
      virtual void SetupRenderFrame(asdp::Time scanOutTime) = 0;

      /// @brief Tear down state needed for rendering.
      /// @details This function is called after all views have been rendered for the frame.  If this
      /// method requires all of the objects/textures to no longer be needed, it must call
      /// glFinish() or use another synchronization method to ensure that the GPU has finished
      /// rendering before releasing the objects.
      virtual void TearDownRenderFrame() = 0;

      /// @brief Helper function to check for errors.
      static void checkShaderError(GLuint shaderId, const std::string& exceptionMsg);
      /// @brief Helper function to check for errors.
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
      GLuint m_programId;
      /// @brief The Uniform ID of the modelview-projection matrix.
      GLuint m_modelViewProjectionUniformId;

      /// @briegf Forward declaration of a class defined in the source code.
      class MeshCube;
      /// @brief Pointer to the mesh to use to draw the room.
      std::shared_ptr<MeshCube> m_roomCube;

      bool SetupRendering() override;
      void RenderView(asdp::Time scanOutTime, const float* viewProjection) override;
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
      /// @param toneMapTexture The OpenGL texture ID of the tone map to use.
      /// @param poseAdjuster A shared pointer to the pose adjuster to use for transforming points.
      /// @param cameraFrameInterval The interval between camera frames to use for latency correction.
      /// @param renderOffsetMicroseconds The offset in microseconds to render the images at.  This is used
      /// for the replay case to change the algorithm so that it deals with the fact that the image
      /// generation cannot be synchronized with rendering.  This specifies how far back (probably 1+
      /// image frame time) for both frame selection and pose adjustment to mimic the behavior of a live
      /// capture.
      /// @param renderFrameInterval The interval between rendered frames to use for replay mode.
      /// @param renderTimingInfo A pointer to the render timing information to fill in.
      /// @param rangeEstimator A shared pointer to the range estimator to use for tone-map range determination.
      CompositeCameras(std::vector< std::shared_ptr<CameraRenderInfo> >& cameraRenderInfo, GLuint toneMapTexture,
        std::shared_ptr<PoseAdjuster> poseAdjuster, Time cameraFrameInterval,
        uint32_t renderOffsetMicroseconds = 0,
        Time renderFrameInterval = Time(),
        RenderTimingInfo *renderTimingInfo = nullptr,
        std::shared_ptr<asdp::render::RangeEstimator> rangeEstimator = nullptr);

      /// @brief Update the vertex buffer object for a camera based on its current depth information.
      /// @details This function updates the vertex buffer object for a camera based on the current depth
      /// information in the cameraRenderInfo object.  It uses the mesh information stored in the
      /// m_cameraBufferInfos object to update the vertex buffer object with the new depth information.
      /// NOTE: This does not call glFinish() to ensure that that data has been written before returning.
      void UpdateVertexBuffer(const CameraRenderInfo& cameraRenderInfo);

      /// @brief Destructor
      ~CompositeCameras();

    protected:
      /// Information about the cameras, filled in by the constructor.
      std::vector< std::shared_ptr<CameraRenderInfo> > m_cameraRenderInfos;
      GLuint m_toneMapTexture; ///< The OpenGL texture ID of the tone map to use.
      std::shared_ptr<PoseAdjuster> m_poseAdjuster; ///< A shared pointer to the pose adjuster to use for transforming points.

      // Parameters used to handle replay mode, keeping a consistent interval between frame selection
      uint32_t m_renderOffsetMicroseconds;  ///< The offset in microseconds to render the images at for replay mode.
      Time m_lastFrameTime;                 ///< The average time of the last frames used.
      Time m_renderFrameInterval;           ///< The interval between render frames to use for replay mode.
      RenderTimingInfo *m_renderTimingInfo; ///< A pointer to the render timing information to fill in.
      std::shared_ptr<asdp::render::RangeEstimator> m_rangeEstimator;  ///< The range estimator to use for tone-map range determination.

      Time m_cameraFrameInterval;           ///< The interval between camera frames to use for distortion correction.

      /// @brief The OpenGL program ID.
      GLuint m_programId;

      /// @brief The Uniform ID of the view-projection matrix taking points from helicopter space.
      GLint m_viewProjectionUniformId;

      /// @brief The Uniform ID of the poseAdjust matrix moving points to their earlier position
      /// in helicopter space.
      GLint m_poseAdjustUniformId;

      GLint m_fVelocityUniformID;    ///< The Uniform ID of the per-frame velocity in helicopter space.
      GLint m_fAxisUniformID;        ///< The Uniform ID of the axis of rotation in helicopter space.
      GLint m_fAngleUniformID;       ///< The Uniform ID of the per-frame angle of rotation in helicopter space.

      /// The identifiers for the image and tone-map textures.
      GLuint m_imageTextureId;
      GLuint m_toneMapTextureId;

      GLuint m_offsetUniformID;      ///< The Uniform ID of the color offset.
      GLuint m_gainUniformID;        ///< The Uniform ID of the color gain.

      /// Global scale based on the product of exposure and gain for all cameras being rendered.
      /// This is averaged over time to enable autoexposure and autogain without causing flickering
      /// if they change rapidly.
      float m_globalExposureGain;

      /// @brief Vector of Image objects to use during a frame rendering, one per camera.
      std::vector<std::shared_ptr<asdp::render::ImageData>> m_images;

      /// @brief Information about the buffers for a camera and mesh used to create/edit them.
      struct CameraBufferInfo {
        GLuint vertexBufferObject; ///< Vertex buffer object for the camera.
        GLuint indexBufferObject; ///< Index buffer object for the camera.
        GLsizei numIndices; ///< Number of indices in the index buffer.
      };

      /// Information about the buffers for each camera, looked up by the camera ID from CameraRenderInfo.
      std::map<uint16_t,CameraBufferInfo> m_cameraBufferInfos;

      /// @brief Create the buffer objects for a camera and store them along with other buffer info.
      /// @details This function creates the buffer objects for the camera and stores the IDs
      /// in the m_cameraBufferInfos object, along with the mesh information used to construct them.
      /// This must be called once for each camera to produce the required buffers before they are used
      /// for rendering.
      /// @param cameraRenderInfo The camera to add the buffer objects for.
      /// @param mesh The mesh information for the camera.
      /// @sideeffect The buffer objects are created and added to m_cameraBufferInfos.
      void CreateBufferInfo(const CameraRenderInfo& cameraRenderInfo, MeshInfo const &mesh);

      // Overridden methods
      bool SetupRendering() override;
      void RenderView(asdp::Time scanOutTime, const float* viewProjection) override;
      void SetupRenderFrame(asdp::Time scanOutTime) override;
      void TearDownRenderFrame() override;
    };

  } // namespace render
} // namespace asdp
