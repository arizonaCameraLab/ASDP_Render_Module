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
#include <Gimbal.h>
#include <glm/glm.hpp>
#include <string>
#include <vector>
#include <memory>

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

/// @brief Read the pose information from the specified JSON file.
/// @param filename The name of the file.
/// @return A vector of PoseInfo objects.
/// @throws std::runtime_error if the file cannot be opened or parsed.
std::vector<PoseInfo> GetPoseInfos(const std::string& filename);

struct GimbalInfo {
  // Required fields
  std::string name;
  bool pitchFirst = true;
  double minYawDegrees = -180;
  double minPitchDegrees = -90;
  double maxYawDegrees = 180;
  double maxPitchDegrees = 180;

  // Optional fields below
  std::string comPort;              ///< If required
  int baud = 9600;                  ///< If required, default is 9600
  double speed = -1;                ///< If required, default leaves unchanged
  double acceleration = -1;         ///< If required, default leaves unchanged
};

/// @brief Read the gimbal information from the specified file.
/// @param filename The name of the file.
/// @return A structure containing the information.
/// @throws std::runtime_error if the file cannot be opened or parsed.
GimbalInfo GetGimbalInfo(const std::string& filename);

/// @brief Construct a gimbal object from the specified information.
/// @param gimbalInfo The gimbal information.
/// @return A shared pointer to the gimbal object.
/// @throws std::runtime_error if there is an error.
std::shared_ptr<Gimbal> ConstructGimbal(const GimbalInfo& gimbalInfo);

/// @brief Compute the world-space ray from the camera through the specified pixel, ignoring distortion.
/// @param cri The camera render information to use to determine the ray in camera space.
/// @param xPixels The X pixel coordinate, need not be centered on a pixel.
/// @param yPixels The Y pixel coordinate, need not be centered on a pixel.
/// @param rotateXFirst If true, rotate around the X axis first, then the Z axis; otherwise, Z first.
/// @param zRotationDegrees The gimbal rotation around the Z axis in degrees.
/// @param xRotationDegrees The gimbal rotation around the X axis in degrees.
/// @param outRayStartInWorld The start of the ray in world (gimbol helicopter) coordinates.
/// @param outRayDirectionInWorld The normalized direction of the ray in world (gimbol helicopter) coordinates.
/// @param verbose If true, print debugging information.
void WorldSpaceRayNoDistortion(const asdp::render::CameraRenderInfo& cri, double xPixels, double yPixels,
  bool rotateXFirst,
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
/// @param rotateXFirst If true, rotate around the X axis first, then the Z axis; otherwise, Z first.
/// @param outZRotationDegrees The gimbal rotation around the Z axis in degrees.
/// @param outXRotationDegrees The gimbal rotation around the X axis in degrees.
/// @param precisionDegrees The required precision for the gimbal angles.
/// @param verbose If true, print debugging information.
void PointPixelAtTargetNoDistortion(const asdp::render::CameraRenderInfo& cri, double xPixels, double yPixels,
  double minXRotationDegrees, double maxXRotationDegrees,
  glm::dvec3 target,
  double rotateXFirst,
  double &outZRotationDegrees, double &outXRotationDegrees,
  double precisionDegrees = 0.01,
  bool verbose = false);

/// @brief See whether the specified target is within the view frustum of the camera at the
/// specified gimbal angles, not counting distortion.
/// @param cri The camera render information.
/// @param rotateXFirst If true, rotate around the X axis first, then the Z axis; otherwise, Z first.
/// @param zRotationDegrees The gimbal rotation around the Z axis in degrees.
/// @param xRotationDegrees The gimbal rotation around the X axis in degrees.
/// @param targetPoint The target point in world (gimbal helicopter) coordinates.
/// @param xPixels The X pixel coordinate, might not be centered on a pixel; -1e6 if outside image.
/// @param yPixels The Y pixel coordinate, might not be centered on a pixel; -1e6 if outside image.
/// @return True if the target is within the view frustum, false otherwise.
bool TargetProjectedLocationNoDistortion(const asdp::render::CameraRenderInfo& cri,
  bool rotateXFirst,
  double zRotationDegrees, double xRotationDegrees, const glm::dvec3& targetPoint,
  double &xPixels, double &yPixels);

/// @brief Test the calibration helpers.
/// @return An empty string if the test passes, otherwise an error message.
std::string Test();

} } };
