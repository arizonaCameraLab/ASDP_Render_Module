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
      /// @param[in] nx Number of points to create in the X direction for each camera pair. The maximum size
      /// for each region is 100x100, so these should be enough to ensure that the combined distortion-adjusted
      /// ideal camera has fewer than this many pixels. One approach is to double the number of pixels for the
      /// original camera and divide by 100 to get the number of points in each direction.
      /// @param[in] ny Number of points to create in the Y direction for each camera pair.  The maximum size
      /// for each region is 100x100, so these should be enough to ensure that the combined distortion-adjusted
      /// ideal camera has fewer than this many pixels. One approach is to double the number of pixels for the
      /// original camera and divide by 100 to get the number of points in each direction.
      /// @param[in] depths List of depths to check in meters in increasing distance order.
      /// @param[in] fitnessThreshold The threshold for fitness of the depth manifold.  If the
      ///            difference between the average squared difference between the region in the
      ///            best and worst matched conditions is less than this, use the default depth
      ///            because there is not enough distinction.
      DepthEstimator(std::vector< std::array<std::shared_ptr<CameraRenderInfo>, 2> > cameras,
        std::shared_ptr<PoseAdjuster> poseAdjuster, Time cameraFrameInterval,
        unsigned nx, unsigned ny,
        std::vector<float> depths = {10, 20, 50, 100, 200, 500, 1000},
        float fitnessThreshold = 5.0f);

      virtual ~DepthEstimator() = default;

      /// @brief Estimate the depth manifold in Helicopter space to project images onto.
      /// @details This function estimates the depth manifold in Helicopter space to project images onto.
      /// It fills in internal data structures for a specified time.  The actual depths should be
      /// queried using EstimateDepth() after this function is called.
      /// @param[in] time Time to estimate the depth at.  This should be the center pixel time for the
      /// scan out for the display.
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

      /// @brief Update the mesh associated with a camera using calls to EstimateDepth().
      /// @details This method is used to update a different camera mesh than the ones used to estimate
      /// the depth.  It uses the depth estimates to update the mesh for the specified camera.
      /// A successfull call to ComputeDepthEstimate() must be made before calling this function.
      /// @param cam Camera whose mesh we are to update.
      void UpdateMesh(CameraRenderInfo& cam);

      /// @brief Test function to test the class.
      /// @return Empty string on success, string with error message on failure.
      static std::string Test();

      /// @brief Speed test function for the class.
      /// @details Remember that the maximum region size is 100x100 and that nx and ny must evenly
      /// divide the image size.
      /// @param[in] width Width of the images to render.
      /// @param[in] height Height of the images to render.
      /// @param[in] nx Number of points to create in the X direction for each camera pair.
      /// @param[in] ny Number of points to create in the Y direction for each camera pair.
      /// @return The average time in seconds to estimate depth on a single camera pair, ignoring
      /// the initial estimation where things are being allocated and configured.  Returns -1 on
      /// failure.
      static float SpeedTestSingleEstimation(uint16_t width, uint16_t height, uint16_t nx, uint16_t ny);

    protected:

      /// Used to hide the implementation details and avoid the need for client code to #include
      /// all of the files needed to implement the class.
      class DepthEstimatorImpl;
      std::shared_ptr<DepthEstimatorImpl> m_impl;

      std::string m_constructorStatus;  ///< Status of the constructor, empty if successful, error message if not.

      /// @brief Produce a set of images with known depths for each camera.
      /// @details We analytically render the images with different depths for different image regions in Y so that
      /// each of the ny samples is completely at the same depth.  We use a sum of sinusoids at relatively
      /// prime frequencies with different phases to make the image have contrast and a specific alignment.
      /// We move this pattern to different Z depths for each region in the Y camera axis.
      /// @param[in] de Depth estimator to modify.
      /// @param[in] width Width of the images to render.
      /// @param[in] height Height of the images to render.
      /// @param[in] nx Number of points to create in the X direction for each camera pair.
      /// @param[in] ny Number of points to create in the Y direction for each camera pair.
      /// @param[in] cameraFrameInterval The time between frames in the camera images.
      /// @param[in] testDepths List of depths in meters to use in increasing distance order.
      /// The first half are used to adjust the bottom half of the image and the top is left blank.
      static void BuildGradientImages(DepthEstimator& de, uint16_t width, uint16_t height,
        uint16_t nx, uint16_t ny, float cameraFrameInterval,
        std::vector<float> testDepths);
    };

  } // namespace render
} // namespace asdp
