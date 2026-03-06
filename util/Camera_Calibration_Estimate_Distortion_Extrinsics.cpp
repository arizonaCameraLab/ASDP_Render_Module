/*
 * Copyright (C) 2025-2026: Arizona Board of Regents on Behalf of the University of Arizona
 */

/**
 * @file Camera_Calibration_Estimate_Distortion_Extrinsics.cpp
 * @brief Apache Strap-Down Pilotage configuration calibration program.
 *
* @author ReliaSolve.
* @date April 24th, 2025.
*/

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <climits>
#include <random>
#include <CameraRenderInfo.h>
#include <ASDP_ImageSource.h>
#include <Calibration_Helpers.h>
#include <spot_tracker.h>
#include <nlohmann/json.hpp>
#ifdef USE_OPENCV
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/calib3d.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp> // For glm::quat and glm::mat3
#endif

using namespace asdp;
using namespace asdp::render;
using namespace asdp::render::calibration;
using json = nlohmann::json;

static std::string VERSION = "2.4.0";

void usage(std::string name)
{
  std::cerr << "Usage: " << name << " [options] camConfig.json targetConfig.json gimbalConfig.json poses.csv imageDirectory threshold outputConfig.json" << std::endl;
  std::cerr << "  camConfig.json                Camera configuration file." << std::endl;
  std::cerr << "  targetConfig.json             Target configuration file." << std::endl;
  std::cerr << "  gimbalConfig.json             Gimbal configuration file." << std::endl;
  std::cerr << "  poses.csv                     CSV file with the poses of the camera." << std::endl;
  std::cerr << "  imageDirectory                Directory with the images." << std::endl;
  std::cerr << "  threshold                     Threshold brightness (int value) for target center." << std::endl;
  std::cerr << "  outputConfig.json             Output configuration file." << std::endl;
  std::cerr << "  Options:" << std::endl;
  std::cerr << "    --help                        Print this information and quit." << std::endl;
  std::cerr << "    --offsetThresholdPixels <int> Maximum offset in pixels before target ignored. Default MAXINT for 1 target, 200 for >1." << std::endl;
  std::cerr << "                                  This is scaled based on the camera's field of view, specify based on 40-degree FOV camera." << std::endl;
  std::cerr << "    --writeMaps <filename.csv>    Write the expected to as-seen mappings to the specified CSV file." << std::endl;
  std::cerr << "    --readMaps <filename.csv>     Read the expected to as-seen mappings from the specified CSV file, don't compute." << std::endl;
  std::cerr << "    --invert                      Invert each image (useful for dark targets)." << std::endl;
};

/// @brief Structure to hold a point entry for a camera, describing its 3D location in space and its 2D projection.
struct PointEntry {
  PointEntry(std::array<double, 3> p3D, std::array<double, 2> imgPt, double zRotDeg, double xRotDeg, uint16_t camID, int tgtID)
    : point3D(p3D), imagePoint(imgPt), zRotationDegrees(zRotDeg), xRotationDegrees(xRotDeg), cameraID(camID), targetID(tgtID) {}

  std::array<double, 3> point3D;     ///< The 3D location of the point in world (gimbol helicopter) coordinates.
  std::array<double, 2> imagePoint;   ///< The 2D measured location of the point in pixel coordinates.
  double zRotationDegrees;            ///< The gimbal Z rotation in degrees when the measurement was made.
  double xRotationDegrees;            ///< The gimbal X rotation in degrees when the measurement was made.

  // The following are used to enable re-calculation of the point3D based on a moved location for the target,
  // which is needed to wrap an optimization loop around the RMS error calculation.
  uint16_t cameraID;                    ///< The ID of the camera that made the measurement.
  int targetID;                         ///< The ID of the target that was measured.
};

#ifdef USE_OPENCV
/// @brief RMS error calculation class suitable for use as the error function in OpenCV optimization.
/// @details The constructor is used to pass in the parameters to use for the error calculation and then
/// the calc() function is called one or more times to determing the RMS values and to update the current
/// camera render infos and bags of mappings based on the parameters passed in.  The GetUpdatedCameraRenderInfos()
/// and GetUpdatedBags() functions can be called after calc() to get the updated camera render infos and bags of mappings.
/// This method is intended to be used by OpenCV optimization functions, which will call calc() with different parameters
/// to find the parameters that minimize the RMS error.  When that is done, the caller should call calc() with the
/// optimized parameters to set the current information to the optimal values.
class RMSErrorFunction : public cv::MinProblemSolver::Function
{
public:

  RMSErrorFunction(std::vector<CameraRenderInfo> const& cameraRenderInfos, std::vector<TargetInfo> targetInfos,
    std::map<int, std::vector<PointEntry> > pointEntries, bool pitchFirst, int verbosity = 1)
    : m_cameraRenderInfos(cameraRenderInfos)
    , m_targetInfos(targetInfos)
    , m_pointEntriesInitial(pointEntries)
    , m_pitchFirst(pitchFirst)
    , m_verbosity(verbosity)
  {
  }

  /// @brief Calculate the RMS error for the given parameters.
  /// @param x The parameters for which to calculate the RMS error.
  /// This consists of dx1, dy1, dz1, dx2, dy2, dz2, for the two targets, shifting each from its initial position.
  /// @return The RMS error sum.
  double calc(const double* x) const override
  {
    // Make adjusted target infos based on the parameters passed in, which are adjustments to the initial target positions.
    if (m_targetInfos.size() != 2) {
      throw std::runtime_error("RMSErrorFunction currently only supports exactly 2 targets.");
    }
    std::vector<TargetInfo> targetInfos = m_targetInfos;
    targetInfos[0].position.x += x[0];
    targetInfos[0].position.y += x[1];
    targetInfos[0].position.z += x[2];
    targetInfos[1].position.x += x[3];
    targetInfos[1].position.y += x[4];
    targetInfos[1].position.z += x[5];

    if (m_verbosity > 1) {
      std::cout << "calc() solving for position: ";
      for (int i = 0; i < 2; i++) {
        std::cout << targetInfos[i].position.x << " " << targetInfos[i].position.y << " " << targetInfos[i].position.z << " ";
      }
      std::cout << std::endl;
    }

    // Make maps for CameraRenderInfo and TargetInfo by ID for easy lookup.
    std::map<uint16_t, const CameraRenderInfo*> cameraRenderInfoByID;
    for (const auto& cri : m_cameraRenderInfos) {
      cameraRenderInfoByID[cri.m_ID] = &cri;
    }
    std::map<int, const TargetInfo*> targetInfoByID;
    for (const auto& target : targetInfos) {
      targetInfoByID[target.id] = &target;
    }

    /// Map from target ID to location
    std::map<int, std::array<double, 3>> pointByID;
    for (const auto& target : targetInfos) {
      pointByID[target.id] = { target.position.x, target.position.y, target.position.z };
    }

    // Make adjusted PointEntry values based on the updated target locations, changing the position
    // for each.
    // Skip those that have invalid expected locations because we can't find their proper location.
    std::map<int, std::vector<PointEntry> > pointEntries;
    for (const auto& point : m_pointEntriesInitial) {
      for (const auto& entry : point.second) {
        // Compute the new expected location for the point based on the updated target location.
        auto cri = cameraRenderInfoByID[entry.cameraID];
        if (!cri) continue;
        auto target = targetInfoByID[entry.targetID];
        if (!target) continue;
        std::array<double, 2> expectedLocation;
        if (!TargetProjectedLocationNoDistortion(*cri, m_pitchFirst,
            entry.zRotationDegrees, entry.xRotationDegrees, target->position,
            expectedLocation[0], expectedLocation[1])) {
          // We hit the case where the target is not visible in the camera with the modified target location, so skip this point.
          continue;
        }
        pointEntries[entry.cameraID].emplace_back(
          pointByID[entry.targetID],
          entry.imagePoint,
          entry.zRotationDegrees,
          entry.xRotationDegrees,
          entry.cameraID,
          entry.targetID);
      }
    }

    // Solve for the optimal OpenCV camera information for each camera.
    std::vector<double> rmsVals(m_cameraRenderInfos.size(), 0.0);
#pragma omp parallel for
    for (int camIndex = 0; camIndex < m_cameraRenderInfos.size(); camIndex++) {
      auto& cri = m_cameraRenderInfos[camIndex];

      // Fill in OpenCV matrix and distortion estimates for this camera ID.
      cv::Size imageSize(cri.m_resolutionPixels[0], cri.m_resolutionPixels[1]);
      cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
      double cx = (cri.m_resolutionPixels[0] - 1) / 2.0;
      double cy = (cri.m_resolutionPixels[1] - 1) / 2.0;
      double fx = (cri.m_resolutionPixels[0] / 2.0) / tan((cri.m_fovDegrees[0] / 2.0) * M_PI / 180.0);
      double fy = fx;   ///< Assume square pixels
      //double fy = (cri.m_resolutionPixels[1] / 2.0) / tan((cri.m_fovDegrees[1] / 2.0) * M_PI / 180.0);
      cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
      cameraMatrix.at<double>(0, 0) = fx;
      cameraMatrix.at<double>(1, 1) = fy;
      cameraMatrix.at<double>(0, 2) = cx;
      cameraMatrix.at<double>(1, 2) = cy;

      // Fill in the point entries for this camera ID that will be used to optimize the camera parameters.
      // We transform the points from their original Helicopter space into the local OpenCV camera coordinate system
      // in three steps.
      // All points for both targets go into a single set of object points and image points.
      std::vector< std::vector<cv::Point3f> > objectPoints;
      objectPoints.emplace_back(); // one view (all correspondences in a single view)
      std::vector< std::vector<cv::Point2f> > imagePoints;
      imagePoints.emplace_back();
      //#define SAVE_OPENCV_INPUTS
#ifdef SAVE_OPENCV_INPUTS
      std::ofstream cvPtsFile("openCVPoints_cam" + std::to_string(cri.m_ID) + ".csv");
      cvPtsFile << "objectX,objectY,objectZ,imageX,imageY" << std::endl;
#endif
      for (auto const& entry : pointEntries[cri.m_ID]) {
        // Rotate the 3D point based on the gimbal angles.
        std::array<double, 3> rotatedPoint = HelicopterToRotatedBall(entry.point3D,
          m_pitchFirst, entry.zRotationDegrees, entry.xRotationDegrees);
        std::array<double, 3> cameraPoint = RotatedBallToCamera(rotatedPoint, cri);
        std::array<double, 3> ocvPoint = CameraToOpenCV(cameraPoint);
        // Add to the object points and image points (use float types expected by OpenCV helpers).
        objectPoints.back().emplace_back(
          static_cast<float>(ocvPoint[0]),
          static_cast<float>(ocvPoint[1]),
          static_cast<float>(ocvPoint[2]));
        imagePoints.back().emplace_back(
          static_cast<float>(entry.imagePoint[0]),
          static_cast<float>(entry.imagePoint[1]));
#ifdef SAVE_OPENCV_INPUTS
        cvPtsFile << ocvPoint[0] << "," << ocvPoint[1] << "," << ocvPoint[2] << ","
          << entry.imagePoint[0] << "," << entry.imagePoint[1] << std::endl;
#endif
      }
#ifdef SAVE_OPENCV_INPUTS
      cvPtsFile.close();
#endif

      // Validate that objectPoints/imagePoints are non-empty and correspond
      if (objectPoints.empty() || imagePoints.empty() || objectPoints.size() != imagePoints.size()) {
        std::cerr << "Warning: insufficient or mismatched calibration views for camera " << cri.m_ID << "; skipping calibration for this camera." << std::endl;
        continue;
      }
      bool hasAnyPoints = false;
      for (size_t vi = 0; vi < objectPoints.size(); ++vi) {
        if (!objectPoints[vi].empty() && !imagePoints[vi].empty() && objectPoints[vi].size() == imagePoints[vi].size()) {
          hasAnyPoints = true;
          break;
        }
      }
      if (!hasAnyPoints) {
        std::cerr << "Warning: no valid correspondence points for camera " << cri.m_ID << "; skipping calibration for this camera." << std::endl;
        continue;
      }

      // Optimize the camera intrinsic and extrinsic parameters and distortion based on the point entries.
      std::vector<cv::Mat> rvecs;
      std::vector<cv::Mat> tvecs;
      cv::TermCriteria termCrit(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 3000, 1e-7);   ///< @todo Adjust as needed

      // Make a series of flag combinations for runs (every run re-solves for intrinsic and extrinsics along with the others,
      // and every run keeps the aspect ratio fixed):
      //  The first will solve only for the principal point.
      //  The second will refine only radial distortion.
      //  The third will refine only tangential distortion.
      //int flags = cv::CALIB_USE_INTRINSIC_GUESS;  ///< CALIB_USE_INTRINSIC_GUESS is required for non-planar points
      //flags |= cv::CALIB_FIX_ASPECT_RATIO;        ///< Fixes the pixel aspect ratio (fx/fy) to the initial estimate (square pixels)
      // The flag below removes tangential distortion correction.
      //flags |= cv::CALIB_ZERO_TANGENT_DIST;
      // The below removes radial distortion correction.
      //flags |= cv::CALIB_FIX_K1 | cv::CALIB_FIX_K2 | cv::CALIB_FIX_K3 | cv::CALIB_FIX_K4 | cv::CALIB_FIX_K5 | cv::CALIB_FIX_K6;
      // The below flag fixes the principal point at the center of the image.
      //flags |= cv::CALIB_FIX_PRINCIPAL_POINT;
      std::vector<int> protocol = {
        cv::CALIB_USE_INTRINSIC_GUESS | cv::CALIB_FIX_ASPECT_RATIO | cv::CALIB_FIX_PRINCIPAL_POINT
        /*
        cv::CALIB_USE_INTRINSIC_GUESS | cv::CALIB_FIX_ASPECT_RATIO | cv::CALIB_ZERO_TANGENT_DIST |
          cv::CALIB_FIX_K1 | cv::CALIB_FIX_K2 | cv::CALIB_FIX_K3 | cv::CALIB_FIX_K4 | cv::CALIB_FIX_K5 | cv::CALIB_FIX_K6,
        cv::CALIB_USE_INTRINSIC_GUESS | cv::CALIB_FIX_ASPECT_RATIO | cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_FIX_PRINCIPAL_POINT,
        cv::CALIB_USE_INTRINSIC_GUESS | cv::CALIB_FIX_ASPECT_RATIO | cv::CALIB_FIX_PRINCIPAL_POINT |
          cv::CALIB_FIX_K1 | cv::CALIB_FIX_K2 | cv::CALIB_FIX_K3 | cv::CALIB_FIX_K4 | cv::CALIB_FIX_K5 | cv::CALIB_FIX_K6
        */
      };

      // Call calibrateCamera
      //cv::setNumThreads(1);
      //cv::setUseOptimized(false);
      double rmsError = 1e6;
      try {
        // Run all of our protocols in sequence, using the output of each as the input to the next.
        for (auto flags : protocol) {
          rmsError = cv::calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs,
            flags, termCrit);
        }
      }
      catch (const cv::Exception& e) {
        std::cerr << "Error: OpenCV exception during calibrateCamera for camera " << cri.m_ID
          << ": " << e.what() << "; skipping calibration for this camera." << std::endl;
        continue;
      }

      if (m_verbosity > 0) {
        std::cout << "Camera ID " << cri.m_ID << "  RMS error: " << rmsError << std::endl;
        //std::cout << "  Camera matrix: " << cameraMatrix << std::endl;
      }

      // Guard against empty outputs (calibration may fail and not populate rvecs/tvecs)
      if (rvecs.empty() || tvecs.empty()) {
        std::cerr << "Error: calibrateCamera did not produce rotation/translation vectors for camera " << cri.m_ID << "; skipping extrinsics update." << std::endl;
        std::cerr << "  rvecs.size()=" << rvecs.size() << " tvecs.size()=" << tvecs.size() << std::endl;
        std::cerr << "  Ensure there are sufficient, matching object/image point sets (and OpenCV built with required modules)." << std::endl;
        continue;
      }

      // Use the first view's rvec/tvec safely (check element shapes)
      if (rvecs[0].total() < 3 || (tvecs[0].rows * tvecs[0].cols) < 3) {
        std::cerr << "Error: rvecs[0] or tvecs[0] has unexpected size for camera " << cri.m_ID << "; skipping." << std::endl;
        continue;
      }

      /*
      std::cout << "  rvec: " << rvecs[0] << std::endl;
      std::cout << "  tvec: " << tvecs[0] << std::endl;
      std::cout << "  Distortion coefficients: " << distCoeffs << std::endl;
      */

      // Save the updated camera information for this camera ID to be looked up by the GetUpdate* functions.
      m_rvecs[cri.m_ID] = rvecs;
      m_tvecs[cri.m_ID] = tvecs;
      m_cameraMatrices[cri.m_ID] = cameraMatrix;
      m_distCoeffs[cri.m_ID] = distCoeffs;

      // Increment the error
      rmsVals[camIndex] = rmsError;
    }

    // Compute the average over all cameras of the RMS error.
    double rmsSum = 0.0;
    if (!rmsVals.empty()) {
      for (size_t i = 0; i < rmsVals.size(); i++) {
        rmsSum += rmsVals[i];
      }
      rmsSum /= static_cast<double>(rmsVals.size());
    }

    if (m_verbosity > 0) {
      std::cout << "Average RMS error: " << rmsSum
        << " at parameters: " << x[0] << "," << x[1] << "," << x[2] << "," << x[3] << "," << x[4] << "," << x[5]
        << std::endl;
    }

    return rmsSum;
  }

  int getDims() const override
  {
    // We have 6 parameters (dx, dy, dz for each of the two targets) that we are optimizing over.
    return 6;
  }

  /// @brief Get the updated camera render infos based on the most recent call to calc().
  /// @param camID The ID of the camera for which to get the updated render infos.
  /// @return A vector of updated rotation vectors for the specified camera. This is empty if calc() has not been called yet.
  std::vector<cv::Mat> GetUpdatedRvecs(uint16_t camID) const {
    return m_rvecs[camID];
  }

  /// @brief Get the updated translation vectors based on the most recent call to calc().
  /// @param camID The ID of the camera for which to get the updated translation vectors.
  /// @return A vector of updated translation vectors for the specified camera. This is empty if calc() has not been called yet.
  std::vector<cv::Mat> GetUpdatedTvecs(uint16_t camID) const {
    return m_tvecs[camID];
  }

  /// @brief Get the updated camera matrix based on the most recent call to calc().
  /// @param camID The ID of the camera for which to get the updated camera matrix.
  /// @return The updated camera matrix for the specified camera. This is empty if calc() has not been called yet.
  cv::Mat GetUpdatedCameraMatrix(uint16_t camID) const {
    return m_cameraMatrices[camID];
  }

  /// @brief Get the updated distortion coefficients based on the most recent call to calc().
  /// @param camID The ID of the camera for which to get the updated distortion coefficients.
  /// @return The updated distortion coefficients for the specified camera. This is empty if calc() has not been called yet.
  cv::Mat GetUpdatedDistCoeffs(uint16_t camID) const {
    return m_distCoeffs[camID];
  }

protected:
  /// Saved from constructor parameters.
  std::vector<CameraRenderInfo> const& m_cameraRenderInfos;
  std::vector<TargetInfo> m_targetInfos;
  std::map<int, std::vector<PointEntry> > m_pointEntriesInitial;
  bool m_pitchFirst;
  int m_verbosity;

  /// Output values looked up by camera ID after calc() is called.
  mutable std::map< uint16_t, std::vector<cv::Mat> > m_rvecs;
  mutable std::map< uint16_t, std::vector<cv::Mat> > m_tvecs;
  mutable std::map<uint16_t, cv::Mat> m_cameraMatrices;
  mutable std::map<uint16_t, cv::Mat> m_distCoeffs;
};

/// @brief Class to optimize using random steps within the parameter space, reducing the size over time.
class RandomOptimizer : public cv::MinProblemSolver {
public:
    RandomOptimizer(int verbosity = 1): cv::MinProblemSolver(), m_verbosity(verbosity) {}

    void setFunction(const cv::Ptr<Function>& f) override { m_function = f; }
    cv::Ptr<Function> getFunction() const override { return m_function; }

    void setTermCriteria(const cv::TermCriteria& termcrit) override { m_termCriteria = termcrit; }
    cv::TermCriteria getTermCriteria() const override { return m_termCriteria; }

    void setInitStep(cv::InputArray step) { step.copyTo(m_initialStepSizes); }
    void getInitStep(cv::OutputArray step) const { m_initialStepSizes.copyTo(step); }

    double minimize(cv::InputOutputArray x) override
    {
      // If we have no function, then we can't optimize, so just return the initial value with a very high error.
      if (m_function.empty()) {
        throw std::runtime_error("Error: No function set for optimization.");
      }

      // Ensure that the parameters are of the correct size for our function.
      if (x.total() != m_function->getDims()) {
        throw std::runtime_error("Error: Input parameter size does not match function dimensions.");
      }
      if (m_initialStepSizes.total() != m_function->getDims()) {
        throw std::runtime_error("Error: Initial step size size does not match function dimensions.");
      }

      // Solve for the initial parameter set, making this our initial best parameters and error.
      x.copyTo(m_bestParameters);
      m_bestError = m_function->calc(m_bestParameters.ptr<double>());

      // If we are not set to terminate based on iterations, then set the max count at the maximum integer value.
      int maxIterations = m_termCriteria.maxCount;
      if ((m_termCriteria.type & cv::TermCriteria::MAX_ITER) == 0) {
        maxIterations = std::numeric_limits<int>::max();
      }

      // Construct a high-quality random number generator for generating random steps from -1.0 to 1.0.
      std::random_device rd;
      std::array<std::uint32_t, 8> seed_data;
      for (auto &s : seed_data) { s = rd(); }
      std::seed_seq seed_seq(seed_data.begin(), seed_data.end());
      std::mt19937_64 rng(seed_seq);
      std::uniform_real_distribution<double> uni_dist(-1.0, 1.0);

      // Repeatedly generate a random step within the parameter space, evaluate the error function at that point,
      // and keep track of the best parameters found.  When we find a new best, we re-center around it by storing
      // it into the x parameter and we reduce the step sizes to focus the search around that area.
      // We stop when we hit the maximum number of iterations or when the step sizes get sufficiently small.
      // If we don't find a better solution within 20 steps, we increase the step sizes to try to get out of a local minima.
      int iter = 0;
      int numStepsSinceImprovement = 0;
      cv::Mat currentStepSize = m_initialStepSizes.clone();
      while (iter < maxIterations) {
        // Generate a random step within the parameter space.
        cv::Mat randomStep(m_initialStepSizes.size(), CV_64F);
        for (int i = 0; i < randomStep.total(); ++i) {
          double r = uni_dist(rng); // in [-1.0, 1.0]
          randomStep.at<double>(i) = r * currentStepSize.at<double>(i);
        }
        // Evaluate the error function at the new point.
        cv::Mat newParameters = m_bestParameters.clone() + randomStep;
        double error = m_function->calc(newParameters.ptr<double>());
        // If this is the best error we've seen, update our best parameters and error, and re-center around it.
        if (error < m_bestError) {
          m_bestError = error;
          m_bestParameters = newParameters.clone();
          // Reduce the step sizes to focus the search around this area.
          currentStepSize *= 0.9;  // Reduce step sizes by 10% each time we find a better solution.
          if (m_verbosity > 0) {
            std::cout << "Iteration " << iter << ": Found better solution with error " << m_bestError << std::endl;
            std::cout << "  Parameters: " << m_bestParameters << std::endl;
            std::cout << "  New step sizes: " << currentStepSize << std::endl;
          }
          // See if the largest of the step sizes is now smaller than the epsilon threshold, and if so, stop.
          double maxStepSize = 0;
          for (int i = 0; i < currentStepSize.total(); ++i) {
            maxStepSize = std::max(maxStepSize, currentStepSize.at<double>(i));
          }
          numStepsSinceImprovement = 0;
          if (maxStepSize < m_termCriteria.epsilon) {
            break;
          }
        } else {
          numStepsSinceImprovement++;
          if (numStepsSinceImprovement >= 20) {
            currentStepSize *= 2;  // Increase step sizes by 10% to try to get out of a local minima.
            numStepsSinceImprovement = 0;
            // If the step sizes are now 3X larger 3than the initial step sizes, clamp them to 3X the initial
            // step sizes to prevent them from growing without bound.
            for (int i = 0; i < currentStepSize.total(); ++i) {
              if (currentStepSize.at<double>(i) > 3.0 * m_initialStepSizes.at<double>(i)) {
                currentStepSize.at<double>(i) = 3.0 * m_initialStepSizes.at<double>(i);
              }
            }
            if (m_verbosity > 0) {
              std::cout << "Iteration " << iter << ": Increasing step sizes to escape local minimum: " << currentStepSize << std::endl;
            }
          }
        }
        iter++;
      }

      // Set the input parameter to the best parameters we found and return the best error.
      m_bestParameters.copyTo(x);
      return m_bestError;
    }

protected:
  int m_verbosity;
  cv::Ptr<Function> m_function;
  cv::TermCriteria m_termCriteria = cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 100, 1e-6);
  cv::Mat m_initialStepSizes;
  cv::Mat m_bestParameters;
  double m_bestError = std::numeric_limits<double>::max();
};

#endif

int main(int argc, char** argv)
{
  std::string camConfigFile, targetConfigFile, gimbalConfigFile, posesFile, imageDirectory, outputFile;
  std::string writeMapsFile, readMapsFile;
  int offsetThresholdPixels = -1;
  int targetBrightnessThreshold = 35767;
  bool invert = false; ///< Whether to invert the images (useful for dark targets).
  size_t realParams = 0;          ///< The number of non-flag parameters we've seen.

  // Parse the command line arguments, with the first non-flag argument being the
  // name of the IP address to listen on.
  for (int i = 1; i < argc; ++i) {
    if (std::string("--help") == argv[i]) {
      usage(argv[0]);
    } else if (std::string("--writeMaps") == argv[i]) {
      if (i + 1 >= argc) {
        std::cerr << "Error: --writeMaps option requires a filename." << std::endl;
        return 1;
      }
      writeMapsFile = argv[++i];
    } else if (std::string("--readMaps") == argv[i]) {
      if (i + 1 >= argc) {
        std::cerr << "Error: --readMaps option requires a filename." << std::endl;
        return 1;
      }
      readMapsFile = argv[++i];
    } else if (std::string("--offsetThresholdPixels") == argv[i]) {
      if (i + 1 >= argc) {
        std::cerr << "Error: --offsetThresholdPixels option requires an integer value." << std::endl;
        return 1;
      }
      offsetThresholdPixels = std::stoi(argv[++i]);
    } else if (std::string("--invert") == argv[i]) {
      invert = true;
    } else if (argv[i][0] == '-') {
      usage(argv[0]);
      return 1;
    } else switch (realParams++) {
    case 0:
      camConfigFile = argv[i];
      break;
    case 1:
      targetConfigFile = argv[i];
      break;
    case 2:
      gimbalConfigFile = argv[i];
      break;
    case 3:
      posesFile = argv[i];
      break;
    case 4:
      imageDirectory = argv[i];
      break;
    case 5:
      targetBrightnessThreshold = std::stoi(argv[i]);
      break;
    case 6:
      outputFile = argv[i];
      break;
    default:
      usage(argv[0]);
      return 2;
    }
  }
  if (realParams != 7) {
    usage(argv[0]);
    return 2;
  }

  if (!writeMapsFile.empty() && !readMapsFile.empty()) {
    std::cerr << "Error: Cannot specify both --writeMaps and --readMaps options." << std::endl;
    return 3;
  }

  // Run inside a block so that the destructors will be called for all objects before we exit.
  {
    std::cout << "Camera_Calibration_Estimate_Distortion_Extrinsics version " << VERSION << std::endl;

    // Read the configuration files.
    std::vector<asdp::render::CameraRenderInfo> cameraRenderInfos;
    try {
      cameraRenderInfos = GetCameraRenderInfos(camConfigFile);
    }
    catch (...) {
      std::cerr << "Error: Unable to read camera configuration file: " << camConfigFile << std::endl;
      return 10;
    }
    std::cout << "Read camera configuration from " << camConfigFile << std::endl;

    std::vector<TargetInfo> targetInfos;
    try {
      targetInfos = GetTargetInfos(targetConfigFile);
    }
    catch (...) {
      std::cerr << "Error: Unable to read target configuration file: " << targetConfigFile << std::endl;
      return 11;
    }
    std::cout << "Read target configuration from " << targetConfigFile << std::endl;

    if (offsetThresholdPixels < 0) {
      if (targetInfos.size() > 1) {
        offsetThresholdPixels = 200;
      } else {
        offsetThresholdPixels = INT_MAX;
      }
    }

    GimbalInfo gimbalInfo;
    try {
      gimbalInfo = GetGimbalInfo(gimbalConfigFile);
    }
    catch (const std::exception& e) {
      std::cerr << "Error: Unable to read gimbal configuration file: " << gimbalConfigFile
        << ": " << e.what() << std::endl;
      return 12;
    }
    std::cout << "Read gimbal configuration from " << gimbalConfigFile << std::endl;

    // Add the offset to the camera positions.
    std::cout << "Adding cameraOffset: " << gimbalInfo.cameraOffset[0] << "," << gimbalInfo.cameraOffset[1] << "," << gimbalInfo.cameraOffset[2] << std::endl;
    for (auto& camera : cameraRenderInfos) {
      camera.m_positionMeters[0] += gimbalInfo.cameraOffset[0];
      camera.m_positionMeters[1] += gimbalInfo.cameraOffset[1];
      camera.m_positionMeters[2] += gimbalInfo.cameraOffset[2];
    }

    // Read the pose information from the specified CSV file.
    std::vector<PoseInfo> poseInfos;
    try {
      poseInfos = GetPoseInfos(posesFile);
    }
    catch (const std::exception& e) {
      std::cerr << "Error: Unable to read pose information from file: " << posesFile
        << ": " << e.what() << std::endl;
      return 13;
    }
    std::cout << "Read pose information from " << posesFile << std::endl;

    // Map per targetID of a map per cameraID of a bag of mappings.
    std::map<uint16_t, std::map<uint16_t, DistortionBagOfMappings::Bag> > perTargetBags;

    /// Map from target ID to location
    std::map<int, std::array<double, 3>> pointByID;
    for (const auto& target : targetInfos) {
      pointByID[target.id] = { target.position.x, target.position.y, target.position.z };
    }

    /// Map from camera ID to a vector of point entries associated with that camera.
    std::map<int, std::vector<PointEntry> > pointEntries;

    // If we are writing the mappings to a file, open that file and write the header line.
    std::ofstream outMapFile;
    if (!writeMapsFile.empty()) {
      outMapFile = std::ofstream(writeMapsFile);
      if (!outMapFile) {
        std::cerr << "Error: Unable to open output file: " << writeMapsFile << std::endl;
        return 40;
      }
      outMapFile << "targetID,frameIndex,cameraID,rotationZ,rotationX,expectedX,expectedY,actualX,actualY" << std::endl;
    }

    // If we are reading the mappings from a file, then read them in and skip the calculations.
    if (!readMapsFile.empty()) {
      std::ifstream inFile(readMapsFile);
      if (!inFile) {
        std::cerr << "Error: Unable to open input file " << readMapsFile << std::endl;
        return 50;
      }
      std::string line;
      // Skip the header line.
      std::getline(inFile, line);
      // Read the mappings from the file. Note that mappings may not have been written for each pose, so
      // we just get the ones we find rather than looking for a specific number.
      size_t numMappings = 0;
      while (std::getline(inFile, line)) {
        // Parse the line as a mapping from expected to actual location.
        std::istringstream iss(line);
        int targetID, frameIndex, cameraID;
        double rotX, rotZ, x1, y1, x2, y2;
        char comma;
        if (!(iss >> targetID >> comma >> frameIndex >> comma >> cameraID >> comma
          >> rotZ >> comma >> rotX >> comma >> x1 >> comma >> y1 >> comma >> x2 >> comma >> y2)) {
          std::cerr << "Error: Unable to parse mapping line: " << line << std::endl;
          return 51;
        }
        // Add the mapping to the appropriate bag of mappings for the appropriate target.
        // We map from the actual (as seen) position to the ideal (expected) position.
        int whichCamera = -1;
        for (size_t i = 0; i < cameraRenderInfos.size(); ++i) {
          if (cameraRenderInfos[i].m_ID == cameraID) {
            whichCamera = i;
            break;
          }
        }
        if (whichCamera == -1) {
          std::cerr << "Error: Camera ID " << cameraID << " not found in camera configuration." << std::endl;
          return 52;
        }
        DistortionBagOfMappings::Point2D expected = PlaneIntersectionForPixelNoDistortion(cameraRenderInfos[whichCamera], { x1, y1 });
        DistortionBagOfMappings::Point2D actual = PlaneIntersectionForPixelNoDistortion(cameraRenderInfos[whichCamera], { x2, y2 });
        DistortionBagOfMappings::Mapping mapping = { actual, expected };
        perTargetBags[targetID][cameraID].push_back(mapping);

        std::array<double, 2> pixelLocation = { x2, y2 };
        pointEntries[cameraID].emplace_back(
          pointByID[targetID],
          pixelLocation,
          rotZ,
          rotX,
          cameraID, targetID);
        ++numMappings;
      }
      inFile.close();
      std::cout << "Read " << numMappings << " mappings from " << readMapsFile << std::endl;

    } else {
      // Make maps for CameraRenderInfo and TargetInfo by ID for easy lookup.
      std::map<uint16_t, const CameraRenderInfo*> cameraRenderInfoByID;
      for (const auto& cri : cameraRenderInfos) {
        cameraRenderInfoByID[cri.m_ID] = &cri;
      }
      std::map<int, const TargetInfo*> targetInfoByID;
      for (const auto& target : targetInfos) {
        targetInfoByID[target.id] = &target;
      }

      // Fill in a mapping entry for the appropriate camera and pose.
      std::atomic_int count = 0;
#pragma omp parallel for shared(count)
      for (int p = 0; p < poseInfos.size(); p++) {
        auto const& pose = poseInfos[p];

        // Read the set of images associated with this pose and average them into a double-precision
        // floating-point array in a double_image object, which will be usable by the spot-tracker
        // library.  Start by reading the first one to get the size.
        int index = 1;
        std::string filename = imageDirectory + "/" + std::to_string(pose.frameIndex)
          + "_" + std::to_string(pose.cameraID) + "_" + std::to_string(index) + ".pgm";
        int width, height;
        std::shared_ptr<double_image> avg;
        try {
          asdp::ImageSource::Image firstPGM(filename);
          width = firstPGM.getWidth();
          height = firstPGM.getHeight();
          if (invert) {
            for (size_t i = 0; i < width * height; i++) {
              uint16_t pixelValue = firstPGM.getData()->at(i);
              firstPGM.getData()->at(i) = 65535 - pixelValue; // Invert the pixel value.
            }
          }
          avg = std::make_shared<double_image>(0, width - 1, 0, height - 1);
          std::shared_ptr< std::vector<uint16_t> > data = firstPGM.getData();
          for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
              avg->write_pixel(x, y, (*data)[y * width + x]);
            }
          }
        }
        catch (const std::exception& e) {
          std::cerr << "Error: Unable to read PGM file: " << filename << ": " << e.what() << std::endl;
          exit(20);
        }
        for (index = 2; index <= pose.numFrames; ++index) {
          filename = imageDirectory + "/" + std::to_string(pose.frameIndex)
            + "_" + std::to_string(pose.cameraID) + "_" + std::to_string(index) + ".pgm";
          try {
            asdp::ImageSource::Image pgm(filename);
            if (pgm.getWidth() != width || pgm.getHeight() != height) {
              std::cerr << "Error: Image " << filename << " has different dimensions from the first image." << std::endl;
              exit(21);
            }
            if (invert) {
              for (size_t i = 0; i < width * height; i++) {
                uint16_t pixelValue = pgm.getData()->at(i);
                pgm.getData()->at(i) = 65535 - pixelValue; // Invert the pixel value.
              }
            }
            std::shared_ptr< std::vector<uint16_t> > data = pgm.getData();
            for (int y = 0; y < height; ++y) {
              for (int x = 0; x < width; ++x) {
                double value;
                if (avg->read_pixel(x, y, value)) {
                  value += (*data)[y * width + x];
                  avg->write_pixel(x, y, value);
                }
              }
            }
          }
          catch (const std::exception& e) {
            std::cerr << "Error: Unable to read PGM file" << filename << ": " << e.what() << std::endl;
            exit(22);
          }
        }
        double scale = 1 / static_cast<double>(pose.numFrames);
        for (int y = 0; y < height; ++y) {
          for (int x = 0; x < width; ++x) {
            double value;
            if (avg->read_pixel(x, y, value)) {
              avg->write_pixel(x, y, value * scale);
            }
          }
        }

        // Find the target location by looking up in the targetInfos vector for the one with the
        // matching ID.
        TargetInfo const* target = targetInfoByID[pose.targetID];
        if (target == nullptr) {
          std::cerr << "Error: Target ID " << pose.targetID << " not found in target configuration." << std::endl;
          exit(24);
        }

        // Look up the camera render info based on this pose's camera ID.
        // Then scale the offset threshold by the ratio of 40 and its horizontal FOV.
        CameraRenderInfo const* cri = cameraRenderInfoByID[pose.cameraID];
        if (cri == nullptr) {
          std::cerr << "Error: Camera ID " << pose.cameraID << " not found in camera configuration." << std::endl;
          exit(14);
        }
        double scaledOffsetThreshold = offsetThresholdPixels * (40.0 / cri->m_fovDegrees[0]);

        // Find the expected location of the target in the image in pixels based on ideal camera parameters.
        std::array<double, 2> expectedLocation;
        if (!TargetProjectedLocationNoDistortion(*cri, gimbalInfo.pitchFirst,
            pose.zRotationDegrees, pose.xRotationDegrees, target->position,
            expectedLocation[0], expectedLocation[1])) {
          // The target does not hit the image plane, there is a problem.
          std::cerr << "Error: Target " << target->id << " does not hit the image plane for camera "
            << pose.cameraID << " at pose " << pose.frameIndex << std::endl;
          std::cerr << "  Expected location: (" << expectedLocation[0] << ", " << expectedLocation[1] << ")" << std::endl;
          std::cerr << "  This happens when Camera_Calibration_Make_Scan was run with a different set of config files" << std::endl;
          std::cerr << "  than are being used here." << std::endl;
          exit(25);
        }

        // Find the pixel above threshold closest to the expected location of the point in the average image.
        int centerX = -1;
        int centerY = -1;
        double minSquaredDistance = 1e30;
        double maxVal = avg->read_pixel_nocheck(0, 0);
        for (int y = 0; y < height; ++y) {
          for (int x = 0; x < width; ++x) {
            maxVal = std::max(maxVal, avg->read_pixel_nocheck(x, y));
            if (avg->read_pixel_nocheck(x, y) >= targetBrightnessThreshold) {
              double squaredDistance = (expectedLocation[0] - x) * (expectedLocation[0] - x)
                + (expectedLocation[1] - y) * (expectedLocation[1] - y);
              if (squaredDistance < minSquaredDistance) {
                minSquaredDistance = squaredDistance;
                centerX = x;
                centerY = y;
              }
            }
          }
        }
        if (minSquaredDistance == 1e30) {
          std::cerr << "Warning: No target found in pose " << pose.frameIndex << " for camera " << pose.cameraID << std::endl;
          continue;
        }
        if (minSquaredDistance > double(scaledOffsetThreshold) * double(scaledOffsetThreshold)) {
          std::cerr << "Warning: Target found too far from expected location in pose " << pose.frameIndex
            << " for camera " << pose.cameraID << ": distance = " << sqrt(minSquaredDistance)
            << " pixels, threshold = " << scaledOffsetThreshold << " pixels." << std::endl;
          continue;
        }
        
        // Optimize a bright-centered cone tracker starting at the specified location to robustly lock onto the bright spot and
        // then optimize a symmetric spot tracker with radius 10 pixels starting there for more precision.
        cone_spot_tracker_interp coneTracker(10);
        double x, y;
        coneTracker.optimize_xy(*avg, 0, x, y, centerX, centerY);

        symmetric_spot_tracker_interp symmetrictracker(10);
        symmetrictracker.set_pixel_accuracy(0.01);
        symmetrictracker.optimize_xy(*avg, 0, x, y, x, y);
        count++;

        // If the optimized location is out of bounds, skip this pose with a warning.
        if (x < 10 || x > width - 11 || y < 10 || y > height - 11) {
          std::cerr << "Warning: Optimized target location out of bounds in pose " << pose.frameIndex
            << " for camera " << pose.cameraID << ": (" << x << ", " << y << ")" << std::endl;
          continue;
        }

        // Add a mapping entry from the actual location to the expected location in the image
        // and tell what we did.  Make a critical section to avoid thread contention during this time.
#pragma omp critical
        {
          auto& bag = perTargetBags[pose.targetID][pose.cameraID];

          DistortionBagOfMappings::Point2D expected = PlaneIntersectionForPixelNoDistortion(*cri, expectedLocation);
          DistortionBagOfMappings::Point2D actual = PlaneIntersectionForPixelNoDistortion(*cri, { x, y });

          // Map from the actual (as seen) position to the ideal (expected) position.
          DistortionBagOfMappings::Mapping mapping = { actual, expected };
          bag.push_back(mapping);
          std::array<double, 2> pixelLocation = { x, y };
          pointEntries[pose.cameraID].emplace_back(
            pointByID[pose.targetID],
            pixelLocation,
            pose.zRotationDegrees,
            pose.xRotationDegrees,
            pose.cameraID, pose.targetID);

          std::cout << count << " / " << poseInfos.size() << " processed; pose " << pose.frameIndex
            << " for camera " << pose.cameraID << "\n";
          std::cout << "  Target expected at (" << expectedLocation[0] << ", " << expectedLocation[1] << ")" << "\n";
          std::cout << "  Target initialized at (" << centerX << ", " << centerY << ")" << "\n";
          std::cout << "  Target optimized to (" << x << ", " << y << ")" << std::endl;

          // Write the mapping to the output file if requested.
          if (!writeMapsFile.empty()) {
            outMapFile << std::fixed << std::setprecision(8) << pose.targetID << "," << pose.frameIndex << "," << pose.cameraID << ","
              << pose.zRotationDegrees << "," << pose.xRotationDegrees << ","
              << expectedLocation[0] << "," << expectedLocation[1] << ","
              << x << "," << y << std::endl;
          }
        }
      }
    }
    // Close the output file if we opened it.
    if (outMapFile.is_open()) {
      outMapFile.close();
      std::cout << "Wrote mappings to " << writeMapsFile << std::endl;
    }

    // We always construct a bag-of-mappings distortion model.
    // Bag of mappings per camera, looked up by camera ID.
    // This is filled in by the optimization routines using the information from the
    // per-target mappings.
    std::map<uint16_t, DistortionBagOfMappings::Bag> bags;
    for (auto& cri : cameraRenderInfos) {
      // Create an empty bag of mappings for each camera.
      bags[cri.m_ID] = DistortionBagOfMappings::Bag();
    }

    // Perform the optimization to determine the camera models, including distortion.
    if (targetInfos.size() == 1) {
      // We have a single target, so we do a direct bag-of-mappings distortion model to make
      // all points line up at the single target's location (only works for a single depth).
      std::cout << "Using single-depth distortion model" << std::endl;

      // Just grab the bag of mappings for the first (and only) target ID.
      bags = perTargetBags[targetInfos[0].id];

    } else {
      // We have multiple targets, so do full estimation of position, orientation, and distortion
      // for each entry in cameraRenderInfos based on the ideal-camera FOV and the target locations.

      std::cout << "Using multiple-depth distortion model" << std::endl;

#ifdef USE_OPENCV

      // Create an RMSErrorFunction to use to compute the reprojection error for the optimization of the camera parameters.
      cv::Ptr<RMSErrorFunction> rmsFunction =
        cv::makePtr<RMSErrorFunction>(cameraRenderInfos, targetInfos, pointEntries, gimbalInfo.pitchFirst, 0);

      // Optimize the target locations by randomly perturbing them and re-optimizing
      // the camera parameters then checking the overall reprojection error.
      std::vector<double> params(rmsFunction->getDims(), 0.0); // Initial parameters (dx, dy, dz for each target).
      cv::Ptr<cv::DownhillSolver> solver = cv::DownhillSolver::create();
      // If you want to use the RandomOptimizer instead, comment out the above line and uncomment the below line.
      // The random optimizer got much worse results than the downhill solver in a ground-truth test.
      //std::shared_ptr<RandomOptimizer> solver = std::make_shared<RandomOptimizer>();
      cv::Ptr<cv::MinProblemSolver::Function> function_ptr(rmsFunction);
      solver->setFunction(function_ptr);
      std::vector<double> stepSizes(rmsFunction->getDims(), 0.2); // Step sizes for the optimization.
      stepSizes[1] /= 2; // Smaller step size for Y since it's more precisely measured.
      stepSizes[4] /= 2; // Smaller step size for Y since it's more precisely measured.
      solver->setInitStep(stepSizes);
      cv::TermCriteria termCrit(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 1000, 1e-6);
      solver->setTermCriteria(termCrit);
      std::cout << "Starting optimization of camera parameters..." << std::endl;
      solver->minimize(params);
      std::cout << "Optimization completed with values: ";
      for (size_t i = 0; i < params.size(); ++i) {
        std::cout << params[i];
        if (i < params.size() - 1) {
          std::cout << ", ";
        }
      }
      std::cout << std::endl;

      // Call the calc() function for the optimal parameters to get the final camera parameters and distortion mappings.
      // The optimizer will have left the parameters at the optimal values.
      double rmsError = rmsFunction->calc(params.data());
      std::cout << "Total RMS reprojection error after optimization: " << rmsError << std::endl;

      // Modify the CRI information in place and set the bags distortion for each camera.
      for (int camIndex = 0; camIndex < cameraRenderInfos.size(); camIndex++) {
        auto& cri = cameraRenderInfos[camIndex];
        cv::Size imageSize(cri.m_resolutionPixels[0], cri.m_resolutionPixels[1]);

        // Get the updated camera information for this camera from the solver.
        auto rvecs = rmsFunction->GetUpdatedRvecs(cri.m_ID);
        auto tvecs = rmsFunction->GetUpdatedTvecs(cri.m_ID);
        auto cameraMatrix = rmsFunction->GetUpdatedCameraMatrix(cri.m_ID);
        auto distCoeffs = rmsFunction->GetUpdatedDistCoeffs(cri.m_ID);

        //======================
        // Fill in the camera intrinsic and extrinsic parameters in the cri structure. Note that we need the inverse of
        // the translation and rotation given by OpenCV because they give the transformation from world to camera coordinates.
        // We then need to convert this from OpenCV's orientation to helicopter orientation (both orientation and translation).
        // OpenCV uses a right-handed coordinate system with X right, Y down, Z forward.
        // Helicopter coordinates use X right, Y forward, Z up.
        /// @todo Full coordinate transformation here for orientation and position differences

        // Target a camera matrix whose fields of view match the optimized ones above but whose center is at
        // the middle of the sensor. Use an alpha of 1 so that we provide mapping coordinates for all pixels in
        // the original image (we'll provide warps for non-existent pixels, but that's okay).
        cv::Mat targetCameraMatrix = cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs,
          cv::Size(cri.m_resolutionPixels[0], cri.m_resolutionPixels[1]), 1, cv::Size(), nullptr, true);

        // Compute the fields of view.
        cri.m_fovDegrees[0] = 2.0 * atan2(cri.m_resolutionPixels[0] / 2.0, targetCameraMatrix.at<double>(0, 0)) * 180.0 / M_PI;
        cri.m_fovDegrees[1] = 2.0 * atan2(cri.m_resolutionPixels[1] / 2.0, targetCameraMatrix.at<double>(1, 1)) * 180.0 / M_PI;

        // Find the offset in local camera space, which is the converted negative translation.
        std::array<double, 3> offsetLocal = OpenCVToCamera({ -tvecs[0].at<double>(0), -tvecs[0].at<double>(1), -tvecs[0].at<double>(2) });

        // Convert rvec to a rotation matrix.
        cv::Mat rotationMatrix;
        cv::Rodrigues(rvecs[0], rotationMatrix);
        // Convert the rotation matrix to a Quaternion, getting the row order correct.
        glm::dquat qLocal(glm::dmat3(
          rotationMatrix.at<double>(0, 0), rotationMatrix.at<double>(1, 0), rotationMatrix.at<double>(2, 0),
          rotationMatrix.at<double>(0, 1), rotationMatrix.at<double>(1, 1), rotationMatrix.at<double>(2, 1),
          rotationMatrix.at<double>(0, 2), rotationMatrix.at<double>(1, 2), rotationMatrix.at<double>(2, 2)
        ));
        qLocal = glm::conjugate(qLocal); // Invert the rotation.

        // Convert the rotation from OpenCV to camera coordinates.
        qLocal = OpenCVToCamera(qLocal);

        // Convert to global helicopter coordinates for both translation and rotation.
        // Apply the global differential rotation to the global orientation.
        // Convert the Quaternion to Euler angles in degrees.
        /// @todo Consider whether we need to adjust the position based on the rotation.
        cri.m_positionMeters = CameraToRotatedBall(offsetLocal, cri);
        glm::dquat dq = CameraToRotatedBall(qLocal, cri);
        glm::dquat q = ApplyDifferentialRotation(dq, cri);
        glm::dvec3 eulerDegrees = asdp::render::calibration::QuaternionToEulerXYZDegrees(q);
        cri.m_orientationDegrees = { eulerDegrees.x, eulerDegrees.y, eulerDegrees.z };

        //======================
        // Fill in the bags for this camera ID's distortion mapping by converting a range of actual points
        // into expected points using the OpenCV distortion.

        // Compute the undistortion rectification maps, which map from an undistorted (expected) camera pixel location to the
        // X and Y coordinates of the original (actual) pixel locations (perhaps fractional). Some points in the expected
        // image may not map to any point in the actual image, but this is okay.
        // Note that the projected region of the original image may extend past the edges of the undistorted image, in which
        // case the mapping will only accurately capture the central region.
        cv::Mat map1, map2;
        try {
          cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), targetCameraMatrix, imageSize, CV_32FC1, map1, map2);
        } catch (const cv::Exception& e) {
          std::cerr << "Error: OpenCV exception during initUndistortRectifyMap for camera " << cri.m_ID
            << ": " << e.what() << "; skipping distortion mapping for this camera." << std::endl;
          continue;
        }

        // Use a 100x100 grid of (fractional location) pixels across the image.
        bags[cri.m_ID] = DistortionBagOfMappings::Bag();
        double stepX = static_cast<double>(cri.m_resolutionPixels[0]) / 100.0;
        double stepY = static_cast<double>(cri.m_resolutionPixels[1]) / 100.0;
        for (double yf = 0.0; yf < cri.m_resolutionPixels[1]; yf += stepY) {
          int y = static_cast<int>(yf);
          for (double xf = 0.0; xf < cri.m_resolutionPixels[0]; xf += stepX) {
            int x = static_cast<int>(xf);

            // Find the expected location by looking up in the undistortion maps.
            double actualX = map1.at<float>(y, x);
            double actualY = map2.at<float>(y, x);

            // Add the mapping from expected to actual location.
            DistortionBagOfMappings::Point2D expected = PlaneIntersectionForPixelNoDistortion(cri, { float(x), float(y) });
            DistortionBagOfMappings::Point2D actual = PlaneIntersectionForPixelNoDistortion(cri, { actualX, actualY });
            DistortionBagOfMappings::Mapping mapping = { actual, expected };
            bags[cri.m_ID].push_back(mapping);
          }
        }
      }

#else
      std::cerr << "Error: Multiple target solver requires OpenCV during compilation." << std::endl;
      return 100;
#endif
    }

    // Bring the positions back to the original camera position by subtracting the offsets we added above.
    for (auto& cri : cameraRenderInfos) {
      cri.m_positionMeters[0] -= gimbalInfo.cameraOffset[0];
      cri.m_positionMeters[1] -= gimbalInfo.cameraOffset[1];
      cri.m_positionMeters[2] -= gimbalInfo.cameraOffset[2];
    }

    // Parse the JSON configuration file for the camera configuration directly, then replace
    // the extrinsic parameters and distortion correction for each camera with the optimized values
    // from the entry that has the same ID as the camera.
    json cameraConfig;
    try {
      std::ifstream configFile(camConfigFile);
      cameraConfig = json::parse(configFile);
    }
    catch (const std::exception& e) {
      std::cerr << "Error: Unable to read camera configuration file: " << camConfigFile
        << ": " << e.what() << std::endl;
      return 200;
    }
    for (auto& camera : cameraConfig["cameras"]) {
      uint16_t id = camera["id"];
      for (auto& cri : cameraRenderInfos) {
        if (cri.m_ID == id) {
          camera["positionMeters"] = cri.m_positionMeters;
          camera["orientationDegrees"] = cri.m_orientationDegrees;
          camera["fieldOfViewDegrees"] = cri.m_fovDegrees;

          // Build the JSON object for the distortion map, which has a "type" field with
          // "bagOfMappings", and a "parameters" field with a "map" field with the bag of mappings.
          // Fill this into the distortion field in the JSON structure.
          json jsonObject;
          jsonObject["type"] = "bagOfMappings";
          jsonObject["parameters"] = json::object();
          jsonObject["parameters"]["map"] = json::array();

          const auto& bag = bags[cri.m_ID];
          json mappingJson = json::array();
          for (const auto& mapping : bag) {
            // Each mapping is a pair of 2D points, so we need to convert them to JSON.
            // The first point is the ideal camera position, and the second point is the distorted camera position.
            json mappingJsonEntry = json::array();
            mappingJsonEntry.push_back({ mapping[0][0], mapping[0][1] });
            mappingJsonEntry.push_back({ mapping[1][0], mapping[1][1] });
            mappingJson.push_back(mappingJsonEntry);
          }
          jsonObject["parameters"]["map"] = mappingJson;
          camera["distortion"] = jsonObject;
          break;
        }
      }
    }

    // Write the optimized camera configuration to the specified JSON file in the root directory.
    std::cout << "Writing optimized camera configuration to " << outputFile << std::endl;
    std::ofstream outFile(outputFile);
    if (!outFile) {
      std::cerr << "Error: Unable to open output file " << outputFile << std::endl;
      return 50;
    }
    outFile << cameraConfig.dump(2) << std::endl;
    outFile.close();

  } // End of block to ensure that all objects are destructed before we exit.

  return 0;
}
