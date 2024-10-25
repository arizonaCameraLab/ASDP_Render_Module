/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include "PoseAdjuster.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cmath>

// Constants for WGS84
const double a = 6378137.0; // Semi-major axis in meters
const double e = 0.081819190842622; // First eccentricity

struct ECEF {
  double x;
  double y;
  double z;
};

static ECEF convertLLAtoECEF(double latitude, double longitude, double altitude) {
  // Convert latitude and longitude from degrees to radians
  double latRad = glm::radians(latitude);
  double lonRad = glm::radians(longitude);

  // Calculate the prime vertical radius of curvature
  double N = a / std::sqrt(1 - e * e * std::sin(latRad) * std::sin(latRad));

  // Calculate ECEF coordinates
  double x = (N + altitude) * std::cos(latRad) * std::cos(lonRad);
  double y = (N + altitude) * std::cos(latRad) * std::sin(lonRad);
  double z = ((1 - e * e) * N + altitude) * std::sin(latRad);

  return { x, y, z };
}

using namespace asdp::render;

PoseAdjuster::~PoseAdjuster()
{
  // Empty destructor.
}

void PoseAdjuster::AddPose(const asdp::MessagePose& poseMessage)
{
  // Get the entries from the pose message.
  float longitude = -1e6, latitude = -1e6, altitude = -1e6;
  std::array<float, 3> rot = {-1e6, -1e6, -1e6}, vel = { -1e6, -1e6, -1e6 }, rotVel = { -1e6, -1e6, -1e6 };
  Time time;
  poseMessage.GetLongitude(longitude);
  poseMessage.GetLatitude(latitude);
  poseMessage.GetAltitude(altitude);
  poseMessage.GetRot(rot);
  poseMessage.GetVel(vel);
  poseMessage.GetRotVel(rotVel);
  poseMessage.GetTime(time);

  // Convert the pose message to a Pose structure and add it to the list.
  // Any entries that are invalid will produce a pose with identity values.
  Pose newPose;
  if (longitude > -1e5 && latitude > -1e5 && altitude > -1e5) {
    // Compute the Earth-centric position in meters from the longitude, latitude, and altitude
    ECEF ecef = convertLLAtoECEF(latitude, longitude, altitude);
    newPose.position = glm::vec3(ecef.x, ecef.y, ecef.z);
  }
  if (rot[0] > -1e5 && rot[1] > -1e5 && rot[2] > -1e5) {
    /// @todo Compute the Earth-centric orientation quaternion from the rotation values.
    // Start with the canonical East-North-Up rotation and then rotate around X, Y, and Z.
  }

  // Velocity in local coordinates in meters per second
  if (vel[0] > -1e5 && vel[1] > -1e5 && vel[2] > -1e5) {
    newPose.velocity = glm::vec3(vel[0], vel[1], vel[2]);
  }

  /// @todo Angular velocity in local coordinates in rotation per dt.
  if (rotVel[0] > -1e5 && rotVel[1] > -1e5 && rotVel[2] > -1e5) {
    float dt = 0.01;

    // Compute the rotation over 1/100th of a second, rotating first around X, then Y, then Z.  Store this into
    // a quaternion.
    glm::quat rotationX = glm::angleAxis(glm::radians(rotVel[0]) * dt, glm::vec3(1.0f, 0.0f, 0.0f));
    glm::quat rotationY = glm::angleAxis(glm::radians(rotVel[1]) * dt, glm::vec3(0.0f, 1.0f, 0.0f));
    glm::quat rotationZ = glm::angleAxis(glm::radians(rotVel[2]) * dt, glm::vec3(0.0f, 0.0f, 1.0f));

    // Order matters for quaternion multiplication. Do X, then Y, then Z.
    newPose.angularVelocity = rotationZ * rotationY * rotationX;
    newPose.dt = dt;
  }

  newPose.time = time;

  m_poses.push_back(newPose);
}
