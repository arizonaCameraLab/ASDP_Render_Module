/*
 * Copyright (C) 2025: Arizona Board of Regents on Behalf of the University of Arizona
 */

/**
 * @file Compare_Configurations.cpp
 * @brief Apache Strap-Down Pilotage configuration file comparitor.
 *
* @author ReliaSolve.
*/

#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include <map>
#include <set>
#include <mutex>
#include <thread>
#include <string>
#include <filesystem>
#include <vector>
#include <list>
#include <atomic>
#include <ASDP_Core_API.h>
#include <ASDP_SpinFreeQueue.hpp>
#include <ASDP_BufferPool.h>
#include <ASDP_StreamPacketSortedQueue.h>
#include <ASDP_ClockSynchronizer.h>
#include <nlohmann/json.hpp>
#include <GL/glew.h>
#include <ToneMap.h>
#include <Composite.h>
#include "Calibration_Helpers.h"

using namespace asdp;
using namespace asdp::render;
using namespace asdp::render::calibration;
using json = nlohmann::json;

static std::string VERSION = "1.1.0";

void usage(std::string name)
{
  std::cerr << "Usage: " << name << " [options] file1.json file2.json" << std::endl;
  std::cerr << "  file1.json                         The first file to compare." << std::endl;
  std::cerr << "  file2.json                         The second file to compare." << std::endl;
  std::cerr << "  Options:" << std::endl;
  std::cerr << "  --depth <float>                    Depth to focal planes (default 900)." << std::endl;
  std::cerr << "  --dumpData <string>                Dump conparison data to the specified file." << std::endl;
  std::cerr << "  --help                             Print this information and quit." << std::endl;
};

int main(int argc, char** argv)
{
  std::string file1, file2;
  float depth = 900.0f;
  std::string dumpDataFile;
  size_t realParams = 0;          ///< The number of non-flag parameters we've seen.

  // Parse the command line arguments, with the first non-flag argument being the
  // name of the IP address to listen on.
  for (int i = 1; i < argc; ++i) {
    if (std::string("--help") == argv[i]) {
      usage(argv[0]);
    } else if (std::string("--depth") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 1;
      }
      depth = std::stof(argv[i]);
    } else if (std::string("--dumpData") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 1;
      }
      dumpDataFile = argv[i];
    } else if (argv[i][0] == '-') {
      usage(argv[0]);
      return 1;
    } else switch (realParams++) {
    case 0:
      file1 = argv[i];
      break;
    case 1:
      file2 = argv[i];
      break;
    default:
      usage(argv[0]);
      return 2;
    }
  }
  if (realParams != 2) {
    usage(argv[0]);
    return 2;
  }

  // Run inside a block so that the destructors will be called for all objects before we exit.
  {
    std::cout << "Compare_Configurations version " << VERSION << std::endl;

    // If we've been asked to dump data, open an output file and put the header line in it.
    std::ofstream dumpData;
    if (!dumpDataFile.empty()) {
      dumpData.open(dumpDataFile);
      if (!dumpData) {
        std::cerr << "Error: Unable to open output file " << dumpDataFile << std::endl;
        return 10;
      }
      dumpData << "CameraID,X1,Y1,Z1,X2,Y2,Z2,X1/Z1,Y1/Z1,X2/Z2,Y2/Z2,Distance,Mesh X,Mesh Y,Pixel distance" << std::endl;
    }

    // Construct CameraRenderInfos for each configuration file.
    std::vector<asdp::render::CameraRenderInfo> cameraRenderInfos1;
    try {
      cameraRenderInfos1 = GetCameraRenderInfos(file1);
    }
    catch (std::runtime_error const& e) {
      std::cerr << "Error: Unable to read camera configuration file: " << file1 << ": " << e.what() << std::endl;
      return 14;
    }
    std::cout << "Read configuration1 from " << file1 << std::endl;

    std::vector<asdp::render::CameraRenderInfo> cameraRenderInfos2;
    try {
      cameraRenderInfos2 = GetCameraRenderInfos(file2);
    }
    catch (std::runtime_error const& e) {
      std::cerr << "Error: Unable to read camera configuration file: " << file2 << ": " << e.what() << std::endl;
      return 15;
    }
    std::cout << "Read configuration2 from " << file2 << std::endl;

    // Compare the two configurations. If they have a different number of cameras, just
    // report that and exit.  Otherwise, compare the camera meshes and report the pairwise
    // mean and max edge vertex differences and the total mean and max across all cameras.
    if (cameraRenderInfos1.size() != cameraRenderInfos2.size()) {
      std::cout << "The two configurations have a different numbers of cameras: "
        << cameraRenderInfos1.size() << " vs. " << cameraRenderInfos2.size() << std::endl;
      return 0;
    }
    std::cout << "The files each have " << cameraRenderInfos1.size() << " cameras." << std::endl;

    // Go through the cameras and compare the meshes if the camera IDs match.  Keep track of the
    // total mean and max differences for meters and pixels across all cameras.
    double totalMeanDistDiff = 0.0, totalMeanPixelDiff = 0.0;
    double totalMaxDistDiff = 0.0, totalMaxPixelDiff = 0.0;
    double totalMeanVigDiff = 0.0, totalMaxVigDiff = 0.0;
    for (size_t c = 0; c < cameraRenderInfos1.size(); ++c) {
      if (cameraRenderInfos1[c].m_ID != cameraRenderInfos2[c].m_ID) {
        std::cout << "Camera IDs do not match: " << cameraRenderInfos1[c].m_ID
          << " vs. " << cameraRenderInfos2[c].m_ID << std::endl;
        return 0;
      }

      // Compute camera meshes for both cameras, making them the same size to sample at the same points
      // and at the requested depth.
      cameraRenderInfos1[c].ComputePlanarCameraMeshInfo(100, 100, depth);
      cameraRenderInfos2[c].ComputePlanarCameraMeshInfo(100, 100, depth);
      MeshInfo const &mesh1 = cameraRenderInfos1[c].m_mesh;
      MeshInfo const &mesh2 = cameraRenderInfos2[c].m_mesh;
      if (mesh1.nx != mesh2.nx || mesh1.ny != mesh2.ny) {
        std::cout << "Camera " << cameraRenderInfos1[c].m_ID << " meshes have different numbers of vertices: "
          << mesh1.nx << "," << mesh1.ny << " vs. " << mesh2.nx << "," << mesh2.ny << std::endl;
        return 0;
      }

      // Compare all vertices on the meshes, determining their differences in meters and in
      // projected pixel location differences. Also determine the differences in vignette brightness.
      double meanDistDiff = 0.0, meanPixelDiff = 0.0;
      double maxDistDiff = 0.0, maxPixelDiff = 0.0;
      double meanVigDiff = 0.0, maxVigDiff = 0.0;
      int count = 0;
      for (int y = 0; y < mesh1.ny; ++y) {
        for (int x = 0; x < mesh1.nx; ++x) {
          size_t index = y * mesh1.nx + x;
          // Add the offset to the camera's base location to get the actual vertex location for each camera.
          // Then subtract the two to get the difference in meters.
          glm::dvec3 vertex1, vertex2;
          for (int i = 0; i < 3; ++i) {
            vertex1[i] = cameraRenderInfos1[c].m_positionMeters[i] + mesh1.vertexInfo[index].offset[i];
            vertex2[i] = cameraRenderInfos2[c].m_positionMeters[i] + mesh2.vertexInfo[index].offset[i];
          }
          glm::vec3 diff = vertex1 - vertex2;
          double dist = glm::length(diff);

          meanDistDiff += dist;
          maxDistDiff = std::max(maxDistDiff, dist);

          // Find the distance in world units projected onto the plane normal to one of the offset vectors,
          // which will be essentially the viewing plane.  Do this by finding the normalized dot product with
          // the offset vector (which is the cosine of the angle between them) and then use the Pythagorean
          // identity to find its sine, which is the component of the distance in the plane.
          double projectedDist = 0;
          if (glm::length(diff) > 0.0) {
            double normDot = glm::dot(glm::normalize(mesh1.vertexInfo[index].offset), glm::normalize(diff));
            double projectedSine = std::sqrt(1.0 - normDot * normDot);
            projectedDist = dist * projectedSine;
          }

          // Compute the projected pixel location for each camera and compare the differences.
          // Find the size of a pixel on the first camera when it is projected to a plane at the
          // specified depth.  This is 1/xPixels times the width of the plane at the depth the vertex.
          // We ignore distortion for this calculation, assuming that the FOV is specified reasonably.
          double depth = cameraRenderInfos1[c].m_mesh.vertexInfo[index].depth;
          double width = 2 * depth * std::tan(glm::radians(cameraRenderInfos1[c].m_fovDegrees[0])/2);
          double pixelSize = width / cameraRenderInfos1[c].m_resolutionPixels[0];
          meanPixelDiff += projectedDist / pixelSize;
          maxPixelDiff = std::max(maxPixelDiff, projectedDist / pixelSize);

          // If we're dumping data, write the camera ID and the two vertex locations to the file.
          // Also dump the normalized locations of the two vertices.
          if (dumpData) {
            dumpData << cameraRenderInfos1[c].m_ID << ","
              << vertex1[0] << "," << vertex1[1] << "," << vertex1[2] << ","
              << vertex2[0] << "," << vertex2[1] << "," << vertex2[2] << ","
              << vertex1[0] / vertex1[2] << "," << vertex1[1] / vertex1[2] << ","
              << vertex2[0] / vertex2[2] << "," << vertex2[1] / vertex2[2] << ","
              << dist << ","
              << x << "," << y << "," << projectedDist / pixelSize
              << std::endl;
          }

          // Find the difference in vignette scaling.
          double xNorm = 2.0 * x / (mesh1.nx - 1) - 1.0;
          double yNorm = 2.0 * y / (mesh1.ny - 1) - 1.0;
          double vig1 = cameraRenderInfos1[c].m_vignette->EvaluateAtPoint({ xNorm, yNorm });
          double vig2 = cameraRenderInfos2[c].m_vignette->EvaluateAtPoint({ xNorm, yNorm });
          double vigDiff = fabs(vig1 - vig2);
          meanVigDiff += vigDiff;
          maxVigDiff = std::max(maxVigDiff, vigDiff);

          ++count;
        }
      }

      meanDistDiff /= count;
      meanPixelDiff /= count;
      meanVigDiff /= count;

      std::cout << "  Camera " << cameraRenderInfos1[c].m_ID << " mean dist: " << meanDistDiff
        << " max dist: " << maxDistDiff << "; mean pixel dist: " << meanPixelDiff
        << " max pixel dist: " << maxPixelDiff << "; mean vig diff: " << meanVigDiff
        << std::endl;

      totalMeanDistDiff += meanDistDiff;
      totalMeanPixelDiff += meanPixelDiff;
      totalMaxDistDiff = std::max(totalMaxDistDiff, maxDistDiff);
      totalMaxPixelDiff = std::max(totalMaxPixelDiff, maxPixelDiff);
      totalMeanVigDiff += meanVigDiff;
      totalMaxVigDiff = std::max(totalMaxVigDiff, maxVigDiff);
    }

    totalMeanDistDiff /= cameraRenderInfos1.size();
    totalMeanPixelDiff /= cameraRenderInfos1.size();
    totalMeanVigDiff /= cameraRenderInfos1.size();

    std::cout << "Avg mean dist: " << totalMeanDistDiff << " max dist: " << totalMaxDistDiff
      << "; avg mean pixel dist: " << totalMeanPixelDiff << " max pixel dist: " << totalMaxPixelDiff
      << "; avg mean vig diff: " << totalMeanVigDiff << " max vig diff: " << totalMaxVigDiff
      << std::endl;
  }

  return 0;
}
