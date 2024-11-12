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
      /// to higher), and the vec3 is the mesh coordinate, so mesh[x][y] holds the point for (x,y).
      std::vector<std::vector<glm::vec3>> mesh;

      /// Time for which the mesh was generated.
      asdp::Time time = {};
    };

    /// @brief Constructs a manifold surface representing the world in Helicopter space from image pairs.
    /// @details The depth estimator uses a list of cameras to estimate the depth of the world in Helicopter
    /// space.  The cameras are grouped into pairs and the depth is estimated by comparing the images from
    /// the two cameras.  This is used to produce a manifold surface mesh whose directions are in field
    /// coordinate angles, producing a subset of a sphere around the point that is the closest intersection
    /// of the primary rays from the viewpoints halfway between each camera pair.
    class DepthEstimator {
    public:
      /// brief Construct the estimator with a list of cameras.
      /// @param[in] cameras List of cameras to use to estimate depth.  These will be grouped into
      /// pairs of cameras to estimate depth.
      /// @param[in] nx Number of points to create in the X direction.
      /// @param[in] ny Number of points to create in the Y direction.
      /// @param[in] minZRotDeg Minimum Z rotation in degrees for the manifold representing depth.
      ///            Rotation is around helicopter Z first (mesh X axis), then around the new X
      ///            axis (mesh Y axis).
      /// @param[in] maxZRotDeg Maximum Z rotation in degrees for the manifold representing depth.
      ///            Rotation is around helicopter Z first (mesh X axis), then around the new X
      ///            axis (mesh Y axis).
      /// @param[in] minXRotDeg Minimum X rotation in degrees for the manifold representing depth.
      ///            Rotation is around helicopter Z first (mesh X axis), then around the new X
      ///            axis (mesh Y axis).
      /// @param[in] maxXRotDeg Maximum X rotation in degrees for the manifold representing depth.
      ///            Rotation is around helicopter Z first (mesh X axis), then around the new X
      ///            axis (mesh Y axis).
      /// @param[in] depths List of depths to check in meters in increasing distance order.
      DepthEstimator(std::vector<CameraRenderInfo> cameras, unsigned nx, unsigned ny,
        float minZRotDeg = -115, float maxZRotDeg = 115,
        float minXRotDeg = -55, float maxXRotDeg = 55,
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

      /// Used to hide the implementation details and avoid the need for client code to #include
      /// all of the files needed to implement the class.
      class DepthEstimatorImpl;
      std::unique_ptr<DepthEstimatorImpl> m_impl;
    };

  } // namespace render
} // namespace asdp
