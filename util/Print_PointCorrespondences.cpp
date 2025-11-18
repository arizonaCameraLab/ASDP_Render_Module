/*
 * Copyright (C) 2025: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include <iostream>
#include <vector>
#include <string>
#include <PointCorrespondences.h>

static void usage(const char* progName)
{
  std::cerr << "Usage: " << progName << " <CSVMapFile>" << std::endl;
}

int main(int argc, char** argv)
{
  std::string CSVFileName;
  if (argc != 2) {
    usage(argv[0]);
    exit(1);
  }
  CSVFileName = argv[1];

  // Run inside a block so that objects are cleaned up before exiting.
  {
    // Open a PointCorrespondences object using the specified CSV file.
    asdp::render::PointCorrespondences pointCorrs(CSVFileName);

    // Try each pair of camera IDs from 1 to 21 against each other one and print any non-empty results.
    for (uint32_t camID1 = 1; camID1 <= 21; camID1++) {
      for (uint32_t camID2 = camID1 + 1; camID2 <= 21; camID2++) {
        std::array<uint32_t, 2> camIDPair = { camID1, camID2 };
        std::vector<asdp::render::PointCorrespondences::PointPair> correspondences =
          pointCorrs.CorrespondencesForCameraPair(camIDPair);
        if (!correspondences.empty()) {
          std::cout << "Camera ID Pair: (" << camID1 << ", " <<
            camID2 << "), Number of Correspondences: " <<
            correspondences.size() << std::endl;
        }
      }
    }
  }
  
  // Done
  return 0;
}
