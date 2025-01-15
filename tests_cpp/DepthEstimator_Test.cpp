/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include <string>
#include <iostream>
#include <DepthEstimator.h>

int main()
{
  std::string ret = asdp::render::DepthEstimator::Test();
  if (ret.size() > 0) {
    std::cerr << "Error in DepthEstimator::Test(): " << ret << std::endl;
    return 1;
  }

  // Run a speed test with a reasonable image size and report on the results.
  float fps = asdp::render::DepthEstimator::SpeedTestSingleEstimation(2000, 1200, 20, 12);
  if (fps < 0) {
    std::cerr << "Error in DepthEstimator::SpeedTestSingleEstimation()"<< std::endl;
    return 2;
  }
  std::cout << "Speed test: " << fps*1.0e3 << "ms per estimation for 2000x1200 image with 20x12 total regions" << std::endl;

  // Clean up resources and exit
  std::cout << "Success" << std::endl;
  return 0;
}
