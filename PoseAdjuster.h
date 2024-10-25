/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

 /**
  * @file PoseAdjuster.h
  * @brief Apache Strap-Down Pilotage Render/PoseAdjuster submodule header file.
  *
 * @author ReliaSolve.
 * @date October 24, 2024.
 */

#pragma once

#include <string>
#include <list>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <ASDP_Core_API.h>

namespace asdp {
  namespace render {

    /// @brief PoseAdjuster class that produces a differential transform to move points in Helicopter space.
    class PoseAdjuster {
    public:

      /// @brief Constructor
      PoseAdjuster() = default;

      /// @brief Destructor, virtual so that derived classes can have their destructors called from pointers.
      virtual ~PoseAdjuster();

      /// @brief Records pose messages for the helicopter.
      /// @param poseMessage The pose message to be recorded.  These are used to determine the pose of the helicopter at different times.
      void AddPose(const asdp::MessagePose& poseMessage);

      /// @brief Provide a transform that moves points in helicopter space from one time to another based on helicopter pose change.
      /// @param endTime The end time for the transform (probably the time of scan out for the center of the rendered image).
      /// @param startTime The start time for the transform (probably the time of the center of image capture).
      /// @return A 4x4 transformation matrix that can be used to transform points from their current location at endTime
      /// to their previous location at startTime.
      glm::mat4 GetTransform(asdp::Time endTime, asdp::Time startTime);

      /// @brief Test function for the PoseAdjuster class.
      /// @return A string indicating the result of the test, empty on success and message describing the problem on failure.
      static std::string Test();

    protected:
      // Structure describing a helicopter pose, including the position, orientation, and velocities.
      struct Pose {
        glm::vec3 position;         ///< Position of the helicopter in 3D space w.r.t. Earth center with Z north spin axis and X towards (0,0) lat/long.
        glm::quat orientation;      ///< Orientation of the helicopter in Earth-centered coordinates of position.
        glm::vec3 velocity;         ///< Velocity of the helicopter in meters per second in Helicopter space.
        glm::quat angularVelocity;  ///< Angular velocity of the helicopter in Helicopter space, rotation per dt.
        double dt;                  ///< Delta time related to angular velocity (allows faster rotation than half a rotation per second).
        asdp::Time time;            ///< Time of the pose.
      };

      std::list<Pose> m_poses;    ///< Vector to store recorded poses, sorted in time with earliest first.
    };

  } // namespace render
} // namespace asdp
