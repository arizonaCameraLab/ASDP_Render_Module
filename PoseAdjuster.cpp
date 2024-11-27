/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include "PoseAdjuster.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <cmath>
#include <iostream>

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
using namespace asdp;

PoseAdjuster::PoseAdjuster(size_t maxCount, PoseAdjusterCoordinates coordinates, bool ignoreTimeDifference)
  : m_maxCount(maxCount)
  , m_coordinates(coordinates)
  , m_ignoreTimeDifference(ignoreTimeDifference)
{
}

PoseAdjuster::~PoseAdjuster()
{
  // Empty destructor.
}

void PoseAdjuster::AddPose(const MessagePose& poseMessage)
{
  double latitude, longitude, altitude;
  std::array<float, 3> rot, vel, rotVel;
  asdp::Time time;
  poseMessage.GetLatitude(latitude);
  poseMessage.GetLongitude(longitude);
  poseMessage.GetAltitude(altitude);
  poseMessage.GetRot(rot);
  poseMessage.GetVel(vel);
  poseMessage.GetRotVel(rotVel);
  poseMessage.GetTime(time);
  AddPose(latitude, longitude, altitude, rot, vel, rotVel, time);
}

void PoseAdjuster::AddPose(double latitude, double longitude, double altitude,
  std::array<float, 3> const& rot,
  std::array<float, 3> const& vel,
  std::array<float, 3> const& rotVel,
  asdp::Time time)
{
  // Store the pose into a Pose structure and add it to the list.
  // Any entries that are invalid will produce pose entries with identity values.
  Pose newPose;
  if (longitude > -1e5 && latitude > -1e5 && altitude > -1e5) {
    // Compute the Earth-centric position in meters from the longitude, latitude, and altitude
    ECEF ecef = convertLLAtoECEF(latitude, longitude, altitude);
    newPose.position = glm::dvec3(ecef.x, ecef.y, ecef.z);
  }
  if (rot[0] > -1e5 && rot[1] > -1e5 && rot[2] > -1e5) {
    // Compute the Earth-centric orientation quaternion from the rotation values.
    // The base coordinate system is Earth-Centered Earth-Fixed (ECEF).
    // Start with the canonical East-North-Up rotation.
    //   Rotate around Z by the longitude, then around the new Y by the latitude.
    // Then rotate to canonical coordinates (X East, Y north)
    //   Rotate around X by 90 degrees, then around Y by 90 degrees.
    // Then apply the specified rotation from canonical space to helicopter space.
    // The resulting transformation specifies the helicopter orientation in Earth-centered space.
    glm::dquat rotationLong = glm::angleAxis(glm::radians(longitude), glm::dvec3(0, 0, 1));
    glm::dquat rotationLat = glm::angleAxis(glm::radians(latitude), glm::dvec3(0, 1, 0));
    glm::dquat HeliX = glm::angleAxis(glm::radians(90.0), glm::dvec3(1, 0, 0));
    glm::dquat HeliY = glm::angleAxis(glm::radians(90.0), glm::dvec3(0, 1, 0));
    glm::dquat rotationX = glm::angleAxis(glm::radians(double(rot[0])), glm::dvec3(1, 0, 0));
    glm::dquat rotationY = glm::angleAxis(glm::radians(double(rot[1])), glm::dvec3(0, 1, 0));
    glm::dquat rotationZ = glm::angleAxis(glm::radians(double(rot[2])), glm::dvec3(0, 0, 1));
    newPose.orientation =  rotationLong * rotationLat * HeliX * HeliY * rotationX * rotationY * rotationZ;
  }

  // Store the initial orientation if this is the first one.
  std::lock_guard<std::mutex> lock(m_poseMutex);
  if (m_poses.empty()) {
    m_initialOrientation = newPose.orientation;
  }

  // Velocity in local coordinates in meters per second
  if (vel[0] > -1e5 && vel[1] > -1e5 && vel[2] > -1e5) {
    newPose.velocity = glm::dvec3(vel[0], vel[1], vel[2]);
  }

  // Angular velocity in local coordinates in rotation per dt.
  if (rotVel[0] > -1e5 && rotVel[1] > -1e5 && rotVel[2] > -1e5) {

    // Compute the rotation over dt, rotating first around X, then Y, then Z.  Store this into
    // a quaternion.
    glm::dquat rotationX = glm::angleAxis(glm::radians(rotVel[0]) * newPose.dt, glm::dvec3(1, 0, 0));
    glm::dquat rotationY = glm::angleAxis(glm::radians(rotVel[1]) * newPose.dt, glm::dvec3(0, 1, 0));
    glm::dquat rotationZ = glm::angleAxis(glm::radians(rotVel[2]) * newPose.dt, glm::dvec3(0, 0, 1));

    // Order matters for quaternion multiplication. Do X, then Y, then Z.
    newPose.angularVelocity = rotationX * rotationY * rotationZ;
  }
  newPose.time = time;

  // Put the pose into the list and keep it sorted.  We find the first entry that is before the new pose and
  // place it after that entry.
  auto it = m_poses.rbegin();
  while (it != m_poses.rend()) {
    if (it->time <= newPose.time) {
      m_poses.insert(it.base(), newPose);
      break;
    }
    ++it;
  }
  if (it == m_poses.rend()) {
    m_poses.push_front(newPose);
  }

  // While we have too many poses, remove the oldest.
  while (m_poses.size() > m_maxCount) {
    m_poses.pop_front();
  }
}

PoseAdjuster::Pose PoseAdjuster::ExtrapolatePose(const Pose& pose, Time time)
{
  // Velocity and angular velocity are assumed to stay the same, as is the dt.
  Pose out = pose;
  out.time = time;

  double delta;
  if (time < pose.time) {
    Time diff = pose.time - time;
    delta = -(diff.seconds + diff.microseconds * 1e-6);
  } else {
    Time diff = time - pose.time;
    delta = diff.seconds + diff.microseconds * 1e-6;
  }

  // Convert the velocity in helicopter coordinates to Earth coordinates and apply it to the position
  // scaled by delta.
  // Transform by the pose orientation to take a vector from helicopter space into Earth space.
  out.position += pose.orientation * (pose.velocity * delta);

  // Apply the angular velocity to the orientation in Earth coordinates scaled by delta.
  // We must transform the rotation from the helicopter coordinates to Earth coordinates
  // before applying it.
  glm::dquat EarthAngularVelocity = pose.orientation * pose.angularVelocity * glm::inverse(pose.orientation);
  out.orientation = glm::angleAxis((delta / pose.dt) * glm::angle(EarthAngularVelocity), glm::axis(EarthAngularVelocity)) * pose.orientation;

  return out;
}

PoseAdjuster::Pose PoseAdjuster::GetPose(asdp::Time time) const
{
  std::lock_guard<std::mutex> lock(m_poseMutex);

  // Find the poses that are just before and just after the requested time.
  Pose beforePose, afterPose;
  bool foundBefore = false, foundAfter = false;
  for (const auto& pose : m_poses) {
    if (pose.time <= time) {
      beforePose = pose;
      foundBefore = true;
    }
    if (pose.time > time && !foundAfter) {
      afterPose = pose;
      foundAfter = true;
      break; // We can stop searching once we find the first after pose
    }
  }

  // If we found no poses, return the identity pose with zero time.
  if (!foundBefore && !foundAfter) {
    return Pose();
  }

  // If we found both poses, we can interpolate between them.
  if (foundBefore && foundAfter) {
    Time before = time - beforePose.time;
    double beforeSeconds = before.seconds + before.microseconds * 1e-6;
    Time span = afterPose.time - beforePose.time;
    double spanSeconds = span.seconds + span.microseconds * 1e-6;
    double t = beforeSeconds / spanSeconds;
    Pose interpolatedPose;
    interpolatedPose.position = glm::mix(beforePose.position, afterPose.position, t);
    interpolatedPose.orientation = glm::slerp(beforePose.orientation, afterPose.orientation, t);
    interpolatedPose.velocity = glm::mix(beforePose.velocity, afterPose.velocity, t);
    interpolatedPose.angularVelocity = glm::slerp(beforePose.angularVelocity, afterPose.angularVelocity, t);
    interpolatedPose.dt = beforePose.dt * (1-t) + afterPose.dt * t;
    interpolatedPose.time = time;
    return interpolatedPose;
  }

  // Extrapolate from whichever pose we found.  We found exactly one of them.
  if (foundBefore) {
    return ExtrapolatePose(beforePose, time);
  }
  return ExtrapolatePose(afterPose, time);
}

glm::dmat4 PoseAdjuster::GetTransform(asdp::Time endTime, asdp::Time startTime) const
{
  // If we have been asked to ignore time differences, make both of these the end time.
  if (m_ignoreTimeDifference) {
    startTime = endTime;
  }

  // Get the poses at the start and end times.
  Pose startPose = GetPose(startTime);
  Pose endPose = GetPose(endTime);

  // The transformation matrix is computed by constructing the inverse of the end pose and then
  // multiplying it by the start pose.  This takes points from the original helicopter space into
  // Earth space and then back into the new helicopter space.
  // Do rotation in local space and then translation in global space.
  glm::dmat4 startRot = glm::mat4_cast(startPose.orientation);
  glm::dmat4 startTrans = glm::translate(glm::dmat4(1.0), startPose.position);
  glm::dmat4 startTransform = startTrans * startRot;
  glm::dmat4 endRot = glm::mat4_cast(endPose.orientation);
  if (m_coordinates == INITIAL_ORIENTATION) {
    // If we are using the initial orientation, that's the space we always want to rotate into.
    endRot = glm::mat4_cast(m_initialOrientation);
  }
  glm::dmat4 endTrans = glm::translate(glm::dmat4(1.0), endPose.position);
  glm::dmat4 endTransform = endTrans * endRot;
  glm::mat4 transform = glm::inverse(endTransform) * startTransform;

  return transform;
}

static double isNear(double a, double b, double epsilon = 1e-6) {
  return std::abs(a - b) < epsilon;
}

std::string PoseAdjuster::Test()
{
  // Test the AddPose method, making sure that they are inserted in order and that we don't overfill.
  {
    PoseAdjuster adjuster(3);
    asdp::Time time1{ 1, 0 };
    asdp::Time time2{ 2, 0 };
    asdp::Time time3{ 3, 0 };
    asdp::Time time4{ 4, 0 };

    adjuster.AddPose(0, 0, 0, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, time4);
    adjuster.AddPose(1, 1, 1, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, time2);
    adjuster.AddPose(2, 2, 2, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, time3);
    adjuster.AddPose(3, 3, 3, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, time1);

    if (adjuster.m_poses.size() != 3) {
      return "PoseAdjuster Test: Expected 3 poses, got " + std::to_string(adjuster.m_poses.size());
    }

    auto it = adjuster.m_poses.begin();
    if (it->time != time2) {
      return "PoseAdjuster Test: Expected first pose time to be 2, got " + std::to_string(it->time.seconds);
    }
    ++it;
    if (it->time != time3) {
      return "PoseAdjuster Test: Expected second pose time to be 3, got " + std::to_string(it->time.seconds);
    }
    ++it;
    if (it->time != time4) {
      return "PoseAdjuster Test: Expected third pose time to be 4, got " + std::to_string(it->time.seconds);
    }
  }

  // Test AddPose to make sure it puts things in the correct location and orientation based on lat/long/alt
  {
    // A pose at 0,0,0 lat, long, alt should be near sea level and Y pointing up.
    PoseAdjuster adjuster(1);
    asdp::Time time{ 1, 0 };
    adjuster.AddPose(0, 0, 0, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, time);
    auto it = adjuster.m_poses.begin();
    if (!isNear(it->position[0], a) || !isNear(it->position[1], 0) || !isNear(it->position[2], 0)) {
      return "PoseAdjuster Test: Expected position to be near ("+std::to_string(a)+", 0, 0), got("
        + std::to_string(it->position[0]) + ", " + std::to_string(it->position[1]) + ", " + std::to_string(it->position[2]) + ")";
    }
    glm::dvec3 up = it->orientation * glm::dvec3(0, 1, 0);
    if (!isNear(up[0], 0) || !isNear(up[1], 0) || !isNear(up[2], 1)) {
      return "PoseAdjuster Test: O1 Expected +Y to be pointing up, got " + std::to_string(up[0]) + ", " + std::to_string(up[1]) + ", " + std::to_string(up[2]);
    }

    // A pose at 0,90,0, lat,long,alt should have X pointing in -X
    adjuster.AddPose(0, 90, 0, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, time);
    it = adjuster.m_poses.begin();
    glm::dvec3 rotatedX = it->orientation * glm::dvec3(1, 0, 0);
    if (!isNear(rotatedX[0], -1) || !isNear(rotatedX[1], 0) || !isNear(rotatedX[2], 0)) {
      return "PoseAdjuster Test: O2 Expected +X to be pointing in -X, got " + std::to_string(rotatedX[0]) + ", " + std::to_string(rotatedX[1]) + ", " + std::to_string(rotatedX[2]);
    }

    // A pose at 0,0,0 with rotation by -90 around Y should have X pointing in X
    adjuster.AddPose(0, 0, 0, { 0, -90, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, time);
    it = adjuster.m_poses.begin();
    rotatedX = it->orientation * glm::dvec3(1, 0, 0);
    if (!isNear(rotatedX[0], 1) || !isNear(rotatedX[1], 0) || !isNear(rotatedX[2], 0)) {
      return "PoseAdjuster Test: O3 Expected +X to be pointing in +X, got " + std::to_string(rotatedX[0]) + ", " + std::to_string(rotatedX[1]) + ", " + std::to_string(rotatedX[2]);
    }

    // Test both world and helicopter local rotation to ensure order is correct
    adjuster.AddPose(0, 90, 0, { 0, -90, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, time);
    it = adjuster.m_poses.begin();
    rotatedX = it->orientation * glm::dvec3(1, 0, 0);
    if (!isNear(rotatedX[0], 0) || !isNear(rotatedX[1], 1) || !isNear(rotatedX[2], 0)) {
      return "PoseAdjuster Test: O4 Expected +X to be pointing in +Y, got " + std::to_string(rotatedX[0]) + ", " + std::to_string(rotatedX[1]) + ", " + std::to_string(rotatedX[2]);
    }
  }

  // Test the ExtrapolatePose method for a time after the pose and a time before the pose.
  {
    Pose pose;

    // Point lies along the +X axis with the Y axis pointing in +Z direction.
    pose.position = glm::dvec3(1000, 0, 0);
    pose.orientation = glm::angleAxis(glm::radians(90.0), glm::dvec3(1, 0, 0));

    // Velocity is 10 m/s in the +Y direction in helicopter space, which is +Z in Earth space.
    pose.velocity = glm::dvec3(0, 10, 0);

    // Angular velocity is 0.1 degrees/s around the +Z axis in helicopter space, tipping +Y helicopter towards -X world away from +Z.
    // This will rotate +2 degrees in a second, or dt * 2 degrees in dt.
    float rotDegrees = 2.0f;
    pose.angularVelocity = glm::angleAxis(glm::radians(rotDegrees) * pose.dt, glm::dvec3(0, 0, 1));

    pose.time = Time(10, 0);

    //=============
    // Check After
    asdp::Time time{ 11, 0 };
    Pose extrapolated = PoseAdjuster::ExtrapolatePose(pose, time);

    // Check that the position is updated correctly.
    if (!isNear(extrapolated.position[0], 1000) || !isNear(extrapolated.position[1], 0) || !isNear(extrapolated.position[2], 10)) {
      return "PoseAdjuster Test: Extrapolated position after is incorrect.";
    }

    // Check that the orientation is updated correctly by rotating the +Y helicopter axis and seeing it point just towards -X from +Z.
    glm::dvec3 rotatedY = extrapolated.orientation * glm::dvec3(0, 1, 0);
    if (!isNear(rotatedY[0], -std::sin(glm::radians(rotDegrees)), 0.02)
      || !isNear(rotatedY[1], 0, 0.01) || !isNear(rotatedY[2], std::cos(glm::radians(rotDegrees)), 0.02)) {
      return "PoseAdjuster Test: Extrapolated orientation after is incorrect.";
    }

    //=============
    // Check Before
    time = Time(9, 0);
    extrapolated = PoseAdjuster::ExtrapolatePose(pose, time);

    // Check that the position is updated correctly.
    if (!isNear(extrapolated.position[0], 1000) || !isNear(extrapolated.position[1], 0) || !isNear(extrapolated.position[2], -10)) {
      return "PoseAdjuster Test: Extrapolated position before is incorrect.";
    }

    // Check that the orientation is updated correctly by rotating the +Y helicopter axis and seeing it point just towards +X from +Z.
    rotatedY = extrapolated.orientation * glm::dvec3(0, 1, 0);
    if (!isNear(rotatedY[0], std::sin(glm::radians(rotDegrees)), 0.02)
      || !isNear(rotatedY[1], 0, 0.01) || !isNear(rotatedY[2], std::cos(glm::radians(rotDegrees)), 0.02)) {
      return "PoseAdjuster Test: Extrapolated orientation before is incorrect.";
    }
  }

  // Check the GetPose() method for a time before the first entry, a time between entries, and a time after the last entry.
  {
    PoseAdjuster adjuster(3);
    asdp::Time time1{ 1, 0 };
    asdp::Time time2{ 2, 0 };
    asdp::Time time3{ 3, 0 };

    adjuster.AddPose(0, 0, 0, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, time1);
    adjuster.AddPose(0, 0, 0, { 0, 0, 0 }, { 1, 1, 1 }, { 0, 0, 0 }, time2);
    adjuster.AddPose(0, 0, 0, { 0, 0, 0 }, { 2, 2, 2 }, { 0, 0, 0 }, time3);

    // Check before the first entry, which should extrapolate
    asdp::Time time{ 0, 0 };
    Pose pose = adjuster.GetPose(time);
    if (pose.velocity != glm::dvec3(0, 0, 0) || pose.angularVelocity != glm::dquat(1, 0, 0, 0)) {
      return "PoseAdjuster Test: GetPose() before first entry is incorrect.";
    }

    // Check between the first and second entries, which should interpolate
    time = Time(1, 500000); // 1.5 seconds
    pose = adjuster.GetPose(time);
    if (!isNear(pose.velocity[0], 0.5) || !isNear(pose.velocity[1], 0.5) || !isNear(pose.velocity[2], 0.5)) {
      return "PoseAdjuster Test: GetPose() between first and second entry is incorrect.";
    }

    // Check after the last entry, which should extrapolate
    time = Time(4, 0);
    pose = adjuster.GetPose(time);
    if (pose.velocity != glm::dvec3(2, 2, 2) || pose.angularVelocity != glm::dquat(1, 0, 0, 0)) {
      return "PoseAdjuster Test: GetPose() after last entry is incorrect.";
    }
  }

  // Check GetTransform() to be sure that the differential transform matches what is expected.
  // Use times that exactly match two entries in the pose list for angle testing, and use one
  // that is extrapolated past the end with a velocity by no rotational velocity to test the
  // translation case and one that is extrapolated before the first to test the rotational case
  // with no translational velocity.
  {
    PoseAdjuster adjuster(3);
    asdp::Time time1{ 1, 0 };
    asdp::Time time2{ 2, 0 };
    asdp::Time time3{ 3, 0 };

    {
      // Verify that we get the identity transform when there are no poses.
      glm::mat4 transform = adjuster.GetTransform(time1, time2);
      glm::mat4 identity = glm::identity<glm::mat4>();
      for (int i = 0; i < 16; ++i) {
        if (!isNear(transform[i / 4][i % 4], identity[i / 4][i % 4])) {
          return "PoseAdjuster Test: GetTransform() with no poses does not return identity.";
        }
      }
    }

    // Add poses that rotate 90 degrees around the X axis from the first to the last and have velocities of +1/second in X
    // and rotations by 2 degrees per second around y.
    adjuster.AddPose(0, 0, 0, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 2, 0 }, time1);
    adjuster.AddPose(0, 0, 0, {30, 0, 0 }, { 1, 0, 0 }, { 0, 2, 0 }, time2);
    adjuster.AddPose(0, 0, 0, {90, 0, 0 }, { 1, 0, 0 }, { 0, 0, 0 }, time3);

    // The +Y axis should spin to point at the -Z axis from the third entry to the first.
    glm::mat4 transform = adjuster.GetTransform(time3, time1);
    glm::vec4 pos1 = transform * glm::vec4(0, 1, 0, 1);
    if (!isNear(pos1[0], 0) || !isNear(pos1[1], 0) || !isNear(pos1[2], -1)) {
      return "PoseAdjuster Test: GetTransform() rotation from time3 to time1 is incorrect.";
    }

    // Relative translation should be -1 meters in the X direction a second after time3 because the
    // helicopter would have moved 1 meter forwards, making the earlier origin be 1 meter behind it.
    transform = adjuster.GetTransform(Time(4, 0), time3);
    glm::vec4 pos2 = transform * glm::vec4(0, 0, 0, 1);
    if (!isNear(pos2[0], -1) || !isNear(pos2[1], 0) || !isNear(pos2[2], 0)) {
      return "PoseAdjuster Test: GetTransform() translation after time3 is incorrect.";
    }

    // The Z axis should spin to point in the +X direction when we extrapolate before the first entry.
    // This is because the helicopter will be rotated -2 degrees around Y at the earlier time, which
    // means that the Z axis from the original time will be +2 degrees.
    transform = adjuster.GetTransform(Time(0,0), time1);
    glm::dvec4 rotatedZ = transform * glm::vec4(0, 0, 1, 1);
    if (!isNear(rotatedZ[0], std::sin(glm::radians(2.0f)), 0.02)
        || !isNear(rotatedZ[1], 0, 0.01) || !isNear(rotatedZ[2], std::cos(glm::radians(2.0f)), 0.02)) {
      return "PoseAdjuster Test: GetTransform() rotation before time1 is incorrect.";
    }
  }

  // Check to be sure that we can lock the orientation.  In this case, rotations to any time should
  // rotate to the original orientation.
  {
    PoseAdjuster adjuster(3, INITIAL_ORIENTATION);
    asdp::Time time1{ 1, 0 };
    asdp::Time time2{ 2, 0 };
    asdp::Time time3{ 3, 0 };

    {
      // Verify that we get the identity transform when there are no poses.
      glm::mat4 transform = adjuster.GetTransform(time1, time2);
      glm::mat4 identity = glm::identity<glm::mat4>();
      for (int i = 0; i < 16; ++i) {
        if (!isNear(transform[i / 4][i % 4], identity[i / 4][i % 4])) {
          return "PoseAdjuster Test: GetTransform() locked with no poses does not return identity.";
        }
      }
    }

    // Add poses that rotate 90 degrees around the X axis from the first to the last and have velocities of +1/second in X
    // and rotations by 2 degrees per second around y.
    adjuster.AddPose(0, 0, 0, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 2, 0 }, time1);
    adjuster.AddPose(0, 0, 0, { 30, 0, 0 }, { 1, 0, 0 }, { 0, 2, 0 }, time2);
    adjuster.AddPose(0, 0, 0, { 90, 0, 0 }, { 1, 0, 0 }, { 0, 0, 0 }, time3);

    // The +Y axis should stay put because we're rotating from time1 to time1.
    glm::mat4 transform = adjuster.GetTransform(time3, time1);
    glm::vec4 pos1 = transform * glm::vec4(0, 1, 0, 1);
    if (!isNear(pos1[0], 0) || !isNear(pos1[1], 1) || !isNear(pos1[2], 0)) {
      return "PoseAdjuster Test: GetTransform() locked rotation from time3 to time1 is incorrect.";
    }

    // Relative translation should be -1 meters in the X direction a second after time3 because the
    // helicopter would have moved 1 meter forwards, making the earlier origin be 1 meter behind it.
    transform = adjuster.GetTransform(Time(4, 0), time3);
    glm::vec4 pos2 = transform * glm::vec4(0, 0, 0, 1);
    if (!isNear(pos2[0], -1) || !isNear(pos2[1], 0) || !isNear(pos2[2], 0)) {
      return "PoseAdjuster Test: GetTransform() locked translation after time3 is incorrect.";
    }
  }

  return "";
}
