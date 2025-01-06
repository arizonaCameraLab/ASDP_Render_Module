/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

 /**
  * @file DepthEstimator.h
  * @brief Apache Strap-Down Pilotage Render/DepthEstimator class header file.
  *
  * @author ReliaSolve.
  * @date December 11th, 2024.
  */

#pragma once
#include <vector>
#include <array>
#include <memory>
#include <string>
#include <Composite.h>
#include <PoseAdjuster.h>

namespace asdp {
  namespace render {

    /// @brief Constructs a queryable depth estimate in Helicopter space from image pairs at specified times.
    /// @details The depth estimator uses a list of cameras to estimate the depth of the world in Helicopter
    /// space.  Cameras are grouped into pairs and the depth is estimated by comparing the images from
    /// each pair.  This is used to produce an internal representation of depths around the helicopter that
    /// can then be queried based on rays to determine the distance from the ray start to an object in the
    /// world.
    /// NOTE: The caller must have the same current OpenGL context on the calling thread when calling any of the
    /// functions in this class, including the constructor. This context must have had glewInit() called on it.
    class DepthEstimator {
    public:
      /// brief Construct the estimator with a list of cameras.
      /// @param[in] cameras List of pairs of cameras to use to estimate depth.
      /// @param[in] poseAdjuster Pose adjuster to use to adjust the poses of the cameras to the same time.
      /// @param[in] cameraFrameInterval The time between frames in the camera images.
      /// @param[in] nx Number of points to create in the X direction for each camera pair.
      /// @param[in] ny Number of points to create in the Y direction for each camera pair.
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
      /// @param[in] fitnessThreshold The threshold for fitness of the depth manifold.  If the
      ///            difference between the average squared difference between the region in the
      ///            best and worst matched conditions is less than this, use the default depth
      ///            because there is not enough distinction.
      DepthEstimator(std::vector< std::array<CameraRenderInfo, 2> > cameras,
        std::shared_ptr<PoseAdjuster> poseAdjuster, Time cameraFrameInterval,
        unsigned nx, unsigned ny,
        float minZRotDeg = -115, float maxZRotDeg = 115,
        float minXRotDeg = -55, float maxXRotDeg = 55,
        std::vector<float> depths = {10, 20, 50, 100, 200, 500, 1000},
        float fitnessThreshold = 5.0f);

      virtual ~DepthEstimator() = default;

      /// @brief Estimate the depth manifold in Helicopter space to project images onto.
      /// @details This function estimates the depth manifold in Helicopter space to project images onto.
      /// It fills in internal data structures for a specified time.  The actual depths should be
      /// queried using EstimateDepth() after this function is called.
      /// @param[in] time Time to estimate the depth at.
      /// @return Empty string on success, string describing the error on failure.
      std::string ComputeDepthEstimate(Time time);

      /// @brief Estimate the depth of a camera ray in Helicopter space.
      /// @details This function estimates the depth of a camera ray in Helicopter space.  It uses the
      /// depth manifold that was estimated with ComputeDepthEstimate() to estimate the depth of the ray.
      /// A successfull call to ComputeDepthEstimate() must be made before calling this function.
      /// @param[in] point The starting point of the ray in Helicopter space.
      /// @param[in] direction The direction of the ray in Helicopter space.
      /// @return The estimated distance from ray start in Helicopter space to an object, -1e6 if there is an error.
      float EstimateDepth(const glm::vec3 &point, const glm::vec3 &direction) const;

      /// @brief Test function to test the class.
      /// @return Empty string on success, string with error message on failure.
      static std::string Test();

    protected:

      /// Used to hide the implementation details and avoid the need for client code to #include
      /// all of the files needed to implement the class.
      class DepthEstimatorImpl;
      std::unique_ptr<DepthEstimatorImpl> m_impl;

      std::string m_constructorStatus;  ///< Status of the constructor, empty if successful, error message if not.
    };

  } // namespace render
} // namespace asdp
