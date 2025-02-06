/*
 * Copyright (C) 2025: Arizona Board of Regents on Behalf of the University of Arizona
 */

 /**
  * @file CameraRenderInfo.h
  * @brief Apache Strap-Down Pilotage Render/CameraRenderInfo header file.
  *
 * @author ReliaSolve.
 * @date January 29, 2025.
 */

#pragma once

#include <string>
#include <array>
#include <vector>
#include <cstdint>
#include <memory>
#include <mutex>
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

namespace asdp {
  namespace render {

    /// @brief Information needed per vertex to generate the vertices for the render mesh for a camera.
    struct VertexInfo {
      glm::vec2 texCoord; ///< The texture coordinates for the vertex.
      glm::vec3 offset; ///< The offset from the camera origin to the point on the plane at a specified depth.
      float vignetteGain = 1.0f; ///< The scale factor for the vignette effect.
      /// The length of the offset vector (distance from the camera).  This is initialized to the length
      /// of the offset vector in ComputeCameraMeshInfo, but can be updated by depth calculations later to
      /// adjust the distance along the normalizedOffset direction to push a point.
      float depth;
      glm::vec3 normalizedOffset; ///< The normalized offset from the camera origin.
    };

    /// @brief Information needed to generate the vertices for the render mesh for a camera.
    struct MeshInfo {
      std::vector<VertexInfo> vertexInfo; ///< The vertex information for the camera.
      size_t nx = 0;                      ///< The number of vertices in the X direction.
      size_t ny = 0;                      ///< The number of vertices in the Y direction.
    };

    /// @brief Information about a single camera needed to produce a renderable view from it.
    struct CameraRenderInfo {
      CameraRenderInfo() = delete;

      /// @brief Constructor
      CameraRenderInfo(uint16_t id, std::array<double, 3> positionMeters,
          std::array<double, 3> orientationDegrees,
          std::array<uint16_t, 2> resolutionPixels, std::array<double, 2> fovDegrees,
          std::shared_ptr<Distortion> distortion,
          std::shared_ptr<Vignette> vignette,
          std::shared_ptr<asdp::render::ImageQueue> imageQueue,
          float depthScale)
        : m_ID(id)
        , m_positionMeters(positionMeters)
        , m_orientationDegrees(orientationDegrees)
        , m_resolutionPixels(resolutionPixels)
        , m_fovDegrees(fovDegrees)
        , m_distortion(distortion)
        , m_vignette(vignette)
        , m_imageQueue(imageQueue)
        , m_depthScale(depthScale)
        {}

      CameraRenderInfo(const CameraRenderInfo& other)
        :CameraRenderInfo(other.m_ID, other.m_positionMeters, other.m_orientationDegrees,
          other.m_resolutionPixels, other.m_fovDegrees, other.m_distortion, other.m_vignette,
          other.m_imageQueue, other.m_depthScale)
      {
        float offset, gain;
        other.GetColorOffsetGain(offset, gain);
        SetColorOffsetGain(offset, gain);

        m_mesh = other.m_mesh;
      };

      /// @brief Thread-safe access to the color offset and gain, which must be consistent.
      void GetColorOffsetGain(float& colorOffset, float& colorGain) const {
        std::lock_guard<std::mutex> lock(m_colorMutex);
        colorOffset = m_colorOffset;
        colorGain = m_colorGain;
      }

      /// @brief Thread-safe set of the color offset and gain, which must be consistent.
      void SetColorOffsetGain(float colorOffset, float colorGain) {
        std::lock_guard<std::mutex> lock(m_colorMutex);
        m_colorOffset = colorOffset;
        m_colorGain = colorGain;
      }

      uint16_t m_ID = 0;                              ///< ID of the camera.

      /// @brief Position of the camera's center of projection in meters from the camera device origin.
      /// @details The canonical orientation is in the local helicopter coordinate system, with +X pointing
      /// right, +Y pointing forwards, and +Z pointing up.  The camera is translated in the
      /// helicopter frame of reference and then rotated around its new center.
      std::array<double, 3> m_positionMeters = {};
      /// @brief Orientation of the camera in degrees, Euler rotation around X, then Y, then Z.
      /// @details The canonical orientation is in the local helicopter coordinate system, with +X pointing
      /// right, +Y pointing forwards, and +Z pointing up.  The camera is translated in the
      /// helicopter frame of reference and then rotated around its new center.
      std::array<double, 3> m_orientationDegrees = {};
      std::array<uint16_t, 2> m_resolutionPixels = {};///< Resolution of the camera in pixels, X then Y.
      std::array<double, 2> m_fovDegrees = {};        ///< Field of view of the camera in degrees, horizontal then vertical.

      /// Distortion correction object for the camera.
      std::shared_ptr<Distortion> m_distortion;
      /// Vignette correction object for the camera.
      std::shared_ptr<Vignette> m_vignette;
      /// Queue of images from the camera.  The newest image is the one to render.
      std::shared_ptr<asdp::render::ImageQueue> m_imageQueue;

      /// The mesh to use to render the camera's image. Can be constructed using ComputePlanarCameraMeshInfo.
      MeshInfo m_mesh;
      mutable std::mutex m_meshMutex;  ///< Mutex to control access to information that must be read as a single unit.

      /// @brief Depth scale to use for this view, -1 disables and > 0 sets color based on scaled depth.
      /// @details This is used for debugging purposes to show the depth of the scene in the camera view.
      GLfloat m_depthScale = -1.0;

      /// @brief Compute the values needed to create the vertices for the render mesh for a camera.
      /// @details This function computes the vertices for a quadrilateral that will be used to display
      /// the image from the camera.  The quadrilateral is at a specified depth from the camera and
      /// is centered on the camera's view direction.  This is used to create a fixed-depth planar
      /// mesh that can be used to render the camera's image.
      /// @param nx The number of vertices in the X direction.
      /// @param ny The number of vertices in the Y direction.
      /// @param depth The distance from the camera to the quadrilateral displaying the image.
      /// @return Information about the mesh that was constructed.
      void ComputePlanarCameraMeshInfo(size_t nx = 100, size_t ny = 100, GLfloat depth = 900);

    protected:
      mutable std::mutex m_colorMutex; ///< Mutex to control access to information that must be read as a single unit.
      float m_colorOffset = 0;         ///< Color = (original + offset) * gain.
      float m_colorGain = 1;           ///< Color = (original + offset) * gain.
    };

  } // namespace render
} // namespace asdp
