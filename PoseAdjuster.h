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
      /// @param maxCount The maximum number of poses to store.
      PoseAdjuster(size_t maxCount = 2000);

      /// @brief Destructor, virtual so that derived classes can have their destructors called from pointers.
      virtual ~PoseAdjuster();

      /// @brief Records pose messages for the helicopter after unpacking entries from a MessagePose.
      /// @param poseMessage The pose message to be recorded.  These are used to determine the pose of the helicopter at different times.
      void AddPose(const MessagePose& poseMessage);

      /// @brief Records pose messages for the helicopter using already-unpacked entries.
      /// @param latitude The latitude of the helicopter in degrees.
      /// @param longitude The longitude of the helicopter in degrees.
      /// @param altitude The altitude of the helicopter in meters.
      /// @param rot The rotation of the helicopter as a 3D vector (roll, pitch, yaw).
      /// @param vel The velocity of the helicopter as a 3D vector (vx, vy, vz).
      /// @param rotVel The angular velocity of the helicopter as a 3D vector (roll rate, pitch rate, yaw rate).
      /// @param time The time of the pose.
      void AddPose(double latitude, double longitude, double altitude,
        std::array<float, 3> const &rot,
        std::array<float, 3> const &vel,
        std::array<float, 3> const &rotVel,
        asdp::Time time);

      /// @brief Provide a transform that moves points in helicopter space from one time to another based on helicopter pose change.
      /// @param endTime The end time for the transform (probably the time of scan out for the center of the rendered image).
      /// @param startTime The start time for the transform (probably the time of the center of image capture).
      /// @return A 4x4 transformation matrix that can be used to transform points in helicopter coordinates for the
      /// pose at startTime to helicopter coordinates for the pose at endTime.
      /// If there are no poses available, the identity matrix is returned.
      glm::dmat4 GetTransform(asdp::Time endTime, asdp::Time startTime) const;

      /// @brief Test function for the PoseAdjuster class.
      /// @return A string indicating the result of the test, empty on success and message describing the problem on failure.
      static std::string Test();

    protected:
      /// Structure describing a helicopter pose, including the position, orientation, and velocities.
      /// Its entries must be double precision to avoid numerical instability with the large Earth radius included in position.
      struct Pose {
        glm::dvec3 position = { 0, 0, 0 };            ///< Position of the helicopter in 3D space w.r.t. Earth center with Z north spin axis and X towards (0,0) lat/long.
                                                      /// Takes point in helicopter space to Earth space.
        glm::dquat orientation = { 1, 0, 0, 0 };      ///< Orientation of the helicopter in the same Earth-centric coordinate system as position.
                                                      /// Takes vectors in helicopter space to Earth space.
        glm::dvec3 velocity = { 0, 0, 0 };            ///< Velocity of the helicopter in meters per second in Helicopter space.
        glm::dquat angularVelocity = { 1, 0, 0, 0 };  ///< Angular velocity of the helicopter in Helicopter space, rotation per dt.
        double dt = 0.01;                             ///< Delta time related to angular velocity (allows faster rotation than half a rotation per second).
        asdp::Time time = { 0, 0 };                   ///< Time of the pose.
      };

      size_t m_maxCount;            ///< Maximum number of poses to store.
      std::list<Pose> m_poses;      ///< Vector to store recorded poses, sorted in time with earliest first.

      /// @brief Extrapolate the pose at a specific time based on a recent pose.
      /// @param pose The recent pose to use for extrapolation.
      /// @param time The time to extrapolate to.
      static Pose ExtrapolatePose(const Pose& pose, Time time);

      /// @brief Get the pose at a specific time, either interpolating between poses or extrapolating from the nearest.
      /// @param time The time for which to get the pose.
      /// @return The pose of the helicopter at the specified time.  Return the identity pose if there are no stored poses.
      Pose GetPose(asdp::Time time) const;
    };

  } // namespace render
} // namespace asdp
