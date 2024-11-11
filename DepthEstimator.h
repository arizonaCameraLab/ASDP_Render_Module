/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

 /**
  * @file ImageQueue.h
  * @brief Apache Strap-Down Pilotage Render/ImageQueue class header file.
  *
  * @author ReliaSolve.
  * @date November 9th, 2024.
  */

#pragma once
#include <vector>
#include <memory>
#include <Composite.h>

namespace asdp {
  namespace render {

    /// @brief Stores a surface mesh surrounding the helicopter and time at which it was estimated.
    /// @details The mesh is a manifold surface representing the world in Helicopter space.  There are
    /// the same number of points in each row and column of the mesh.  The outer index is in X (surrounding
    /// the helicopter from left to right), the inner index is in Y (lower to higher), and the value is the
    /// mesh coordinate.  This can be used to form a vertex array and then indices can be used to generate
    /// a triangle mesh.
    struct DepthEstimate {

      /// A manifold surface representing the world in Helicopter space.  Its outer index
      /// is in X (surrounding the helicopter from left to right), its inner index is in Y (lower
      /// to higher), and the vec3 is the mesh coordinate.
      std::vector<std::vector<glm::vec3>> mesh;

      /// Average time for all images used to generate the mesh.
      asdp::Time time = {};
    };

    /// @brief Constructs a manifold surface representing the world in Helicopter space from image pairs.
    /// @todo
    class DepthEstimator {
    public:
      /// brief Construct the estimator with a list of cameras.
      /// @param[in] cameras List of cameras to use to estimate depth.  These will be grouped into
      /// pairs of cameras to estimate depth.
      /// @param[in] nx Number of points to create in the X direction.
      /// @param[in] ny Number of points to create in the Y direction.
      /// @param[in] depths List of depths to check in meters in increasing distance order.
      DepthEstimator(std::vector<CameraRenderInfo> cameras, unsigned nx, unsigned ny,
        std::vector<float> depths = {10, 20, 50, 100, 200, 500, 1000});

      virtual ~DepthEstimator() = default;

      /// @brief Estimate the depth manifold in Helicopter space to project our images onto.
      /// @param[in] time Time to estimate the depth at.
      /// @return A manifold surface representing the world in Helicopter space.  Its outer index
      /// is in X (surrounding the helicopter from left to right), its inner index is in Y (lower
      /// to higher), and the vec3 is the mesh coordinate.  Returns an empty mesh on failure.
      /// Each point defaults to the furthest value when there is insufficient local image evidence
      /// for the appropriate depth.
      DepthEstimate EstimateDepth(Time time);

      /// @brief Test function to test the class.
      /// @return Empty string on success, string with error message on failure.
      static std::string Test();

    protected:
      /// Cameras to use to estimate depth.
      std::vector<CameraRenderInfo> m_cameras;

      /// Number of points to create in the X direction.
      unsigned m_nx;

      /// Number of points to create in the Y direction.
      unsigned m_ny;

      /// Depths to check in meters.
      std::vector<float> m_depths;

      /// Used to hide the implementation details and avoid the need for client code to #include
      /// all of the files needed to implement the class.
      class DepthEstimatorImpl;
      std::unique_ptr<DepthEstimatorImpl> m_impl;
    };

  } // namespace render
} // namespace asdp
