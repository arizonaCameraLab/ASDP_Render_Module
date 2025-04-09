/*
 * Copyright (C) 2025: Arizona Board of Regents on Behalf of the University of Arizona
 */

/**
 * @file Calibration_Helpers.h
 * @brief Apache Strap-Down Pilotage configuration calibration program.
 *
* @author ReliaSolve.
* @date April 2nd, 2025.
*/

#include <CameraRenderInfo.h>
#include <glm/glm.hpp>
#include <string>
#include <vector>

namespace asdp { namespace render { namespace calibration {

/// @brief Read the camera render information from the specified configuration file.
/// @param configFileName The name of the configuration file.
/// @return A vector of CameraRenderInfo objects.
/// @throws std::runtime_error if the file cannot be opened or parsed.
std::vector<CameraRenderInfo> GetCameraRenderInfos(const std::string& configFileName);

struct TargetInfo {
  int id;
  glm::dvec3 position;
};

/// @brief Read the target information from the specified configuration file.
/// @param configFileName The name of the configuration file.
/// @return A vector of TargetInfo objects.
/// @throws std::runtime_error if the file cannot be opened or parsed.
std::vector<TargetInfo> GetTargetInfos(const std::string& configFileName);

struct PoseInfo {
  int frameIndex;
  double zRotationDegrees;
  double xRotationDegrees;
  int cameraID;
  int numFrames;
};

/// @brief Read the pose information from the specified file.
/// @param filename The name of the file.
/// @return A vector of PoseInfo objects.
/// @throws std::runtime_error if the file cannot be opened or parsed.
std::vector<PoseInfo> GetPoseInfos(const std::string& filename);

/// @brief Compute the world-space ray from the camera through the specified pixel, ignoring distortion.
/// @param cri The camera render information to use to determine the ray in camera space.
/// @param xPixels The X pixel coordinate, need not be centered on a pixel.
/// @param yPixels The Y pixel coordinate, need not be centered on a pixel.
/// @param zRotationDegrees The gimbal rotation around the Z axis in degrees.
/// @param xRotationDegrees The gimbal rotation around the X axis in degrees.
/// @param outRayStartInWorld The start of the ray in world (gimbol helicopter) coordinates.
/// @param outRayDirectionInWorld The direction of the ray in world (gimbol helicopter) coordinates.
/// @param verbose If true, print debugging information.
void WorldSpaceRayNoDistortion(const asdp::render::CameraRenderInfo& cri, double xPixels, double yPixels,
  double zRotationDegrees, double xRotationDegrees,
  glm::dvec3& outRayStartInWorld, glm::dvec3& outRayDirectionInWorld,
  bool verbose = false);

/// @brief Compute the gimbal angles to point a camera pixel at the specified 3D target location
/// as closely as possible, ignoring distortion.
/// @param cri The camera render information to use to determine the ray in camera space.
/// @param xPixels The X pixel coordinate, need not be centered on a pixel.
/// @param yPixels The Y pixel coordinate, need not be centered on a pixel.
/// @param minXRotationDegrees The minimum gimbal rotation around the X axis in degrees, probably negative.
/// @param maxXRotationDegrees The maximum gimbal rotation around the X axis in degrees, probably positive.
/// @param target The 3D target location in world (gimbol helicopter) coordinates.
/// @param outZRotationDegrees The gimbal rotation around the Z axis in degrees.
/// @param outXRotationDegrees The gimbal rotation around the X axis in degrees.
/// @param precisionDegrees The required precision for the gimbal angles.
/// @param verbose If true, print debugging information.
void PointPixelAtTargetNoDistortion(const asdp::render::CameraRenderInfo& cri, double xPixels, double yPixels,
  double minXRotationDegrees, double maxXRotationDegrees,
  glm::dvec3 target,
  double &outZRotationDegrees, double &outXRotationDegrees,
  double precisionDegrees = 0.01,
  bool verbose = false);

/// @brief See whether the specified target is within the view frustum of the camera at the
/// specified gimbal angles, not counting distortion.
/// @param cri The camera render information.
/// @param zRotationDegrees The gimbal rotation around the Z axis in degrees.
/// @param xRotationDegrees The gimbal rotation around the X axis in degrees.
/// @param targetPoint The target point in world (gimbal helicopter) coordinates.
/// @param xPixels The X pixel coordinate, might not be centered on a pixel; -1e6 if outside image.
/// @param yPixels The Y pixel coordinate, might not be centered on a pixel; -1e6 if outside image.
/// @return True if the target is within the view frustum, false otherwise.
bool TargetProjectedLocationNoDistortion(const asdp::render::CameraRenderInfo& cri,
  double zRotationDegrees, double xRotationDegrees, const glm::dvec3& targetPoint,
  double &xPixels, double &yPixels);

/// @brief Test the calibration helpers.
/// @return An empty string if the test passes, otherwise an error message.
std::string Test();

} } };
