/*
 * Copyright (C) 2025: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include "PointCorrespondences.h"
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>

using namespace asdp::render;

PointCorrespondences::PointCorrespondences(std::string mapFileName)
{
  // Open the CSV file and read it line by line, skipping the header, into an array.
  typedef struct {
    uint32_t targetID;
    uint32_t frameIndex;
    uint32_t cameraID1;
    double x;
    double y;
  } FileLine;
  std::vector<FileLine> fileLines;

  // Skip the header line
  std::ifstream inFile(mapFileName);
  std::string line;
  std::getline(inFile, line);

  // Read each line that has comma-separated values and pull the ones we want from the line.
  while (std::getline(inFile, line)) {
    FileLine entry;
    try {
      std::istringstream iss(line);
      std::string token;
      std::getline(iss, token, ','); // targeID
      entry.targetID = std::stoul(token);
      std::getline(iss, token, ','); // frameIndex
      entry.frameIndex = std::stoul(token);
      std::getline(iss, token, ','); // cameraID1
      entry.cameraID1 = std::stoul(token);
      std::getline(iss, token, ','); // expected X, skipping
      std::getline(iss, token, ','); // expected Y, skipping
      std::getline(iss, token, ','); // actualX, read into x
      entry.x = std::stod(token);
      // Check that the line has enough tokens. If not, skip it.
      if (!std::getline(iss, token, ',')) { continue; } // actualY, read into y
      entry.y = std::stod(token);
    } catch (...) {
      // If there was any error parsing the line, skip it.
      continue;
    }

    // Store the point in the vector.
    fileLines.push_back(entry);
  }

  // Sort the lines by targetID and frameIndex to group points from the same target and frame together.
  std::sort(fileLines.begin(), fileLines.end(),
    [](FileLine const& a, FileLine const& b) {
      if (a.targetID != b.targetID) {
        return a.targetID < b.targetID;
      }
      return a.frameIndex < b.frameIndex;
    });

  // Run through the sorted lines. Whenever we find exactly two lines with the same targetID and frameIndex,
  // we have a correspondence between two cameras.  Store it in the map.
  size_t i = 0;
  while (i < fileLines.size()) {
    size_t j = i + 1;
    while (j < fileLines.size() &&
           fileLines[j].targetID == fileLines[i].targetID &&
           fileLines[j].frameIndex == fileLines[i].frameIndex) {
      j++;
    }
    // If we have exactly two lines with the same targetID and frameIndex, store the correspondence.
    if (j == i + 2) {
      FileLine const& line1 = fileLines[i];
      FileLine const& line2 = fileLines[i + 1];
      CameraIDPairSet cameraIDPair = { line1.cameraID1, line2.cameraID1 };
      CameraPointsMap& cameraPointsMap = m_cameraPairToPoints[cameraIDPair];
      cameraPointsMap[line1.cameraID1].push_back({ line1.x, line1.y });
      cameraPointsMap[line2.cameraID1].push_back({ line2.x, line2.y });
    }
    i = j;
  }
}

std::vector<PointCorrespondences::PointPair> PointCorrespondences::CorrespondencesForCameraPair(
  std::array<uint32_t, 2> cameraIDs) const
{
  CameraIDPairSet cameraIDPair = { cameraIDs[0], cameraIDs[1] };
  auto it = m_cameraPairToPoints.find(cameraIDPair);
  std::vector<PointPair> correspondences;
  if (it != m_cameraPairToPoints.end()) {
    const CameraPointsMap& cameraPointsMap = it->second;
    const std::vector<Point2D>& points1 = cameraPointsMap.at(cameraIDs[0]);
    const std::vector<Point2D>& points2 = cameraPointsMap.at(cameraIDs[1]);
    size_t numPoints = points1.size();
    for (size_t i = 0; i < numPoints; i++) {
      PointPair pair;
      pair[0] = points1[i];
      pair[1] = points2[i];
      correspondences.push_back(pair);
    }
  }
  return correspondences;
}
