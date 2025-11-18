/*
 * Copyright (C) 2025: Arizona Board of Regents on Behalf of the University of Arizona
 */

 /**
  * @file PointCorrespondences.h
  * @brief Apache Strap-Down Pilotage point-correspondences submodule header file.
  *
 * @author ReliaSolve.
 * @date November 13, 2025.
 */

#pragma once

#ifdef WIN32
#include <windows.h>
#endif
#include <string>
#include <set>
#include <vector>
#include <array>
#include <cstdint>
#include <map>

namespace asdp {
  namespace render {

    /// @brief Provides vectors of pairs of points visible by two cameras.
    class PointCorrespondences {
    public:

      /// @brief Constructor that loads point correspondences from a geometric calibration map CSV file.
      PointCorrespondences(std::string mapFileName);

      typedef std::array<double, 2> Point2D;      ///< A 2D point (x,y) in pixel space, perhaps between two pixels.
      typedef std::array<Point2D, 2> PointPair;   ///< A pair of 2D points, one for each camera.

      /// @brief Get the vector of point pairs for the specified camera ID pair.
      /// @param cameraIDs The pair of camera IDs.
      /// @return A vector of point pairs for the specified camera ID pair, with points listed in the same
      /// order within each PointPair as they are in cameraIDs. The vector will be empty if there are no
      /// correspondences between this pair of cameras. The same points will be returned for either order
      /// of cameras in cameraIDs, just in reversed order within each PointPair.
      std::vector<PointPair> CorrespondencesForCameraPair(std::array<uint32_t, 2> cameraIDs) const;

    protected:

      /// @brief Within a single correspondence map, map from each camera ID to its list of points.
      /// @details Each camera ID maps to a vector of 2D points in pixel space.  There will be the same
      /// number of points for each camera ID in the map, and the points at each index correspond to each other.
      typedef std::map< uint32_t, std::vector<Point2D> > CameraPointsMap;

      /// @brief A set of camera IDs; we will always have two IDs in the set.
      /// @details It is important that these be order-independent, so we use a set.
      typedef std::set<uint32_t> CameraIDPairSet;

      /// @brief Map from camera ID pairs to their point correspondences.
      std::map< CameraIDPairSet, CameraPointsMap > m_cameraPairToPoints;
    };

  } // namespace render
} // namespace asdp
