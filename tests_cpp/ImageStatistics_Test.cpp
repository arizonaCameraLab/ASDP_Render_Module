/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include <string>
#include <iostream>
#include <ImageStatistics.h>

int main()
{
  // Run a speed test with a reasonable image size and report on the results.
  float fps = asdp::render::imageStatistics::MeanStd::SpeedTestSingleCalculation(1280, 1024);
  if (fps < 0) {
    std::cerr << "Error in MeanStd::SpeedTestSingleEstimation()" << std::endl;
    return 2;
  }
  std::cout << "Speed test: " << fps * 1.0e3 << "ms per estimation for 1280x1024 image" << std::endl;

  std::string ret = asdp::render::imageStatistics::MeanStd::Test();
  if (ret.size() > 0) {
    std::cerr << "Error in MeanStd::Test(): " << ret << std::endl;
    return 1;
  }

  // Clean up resources and exit
  std::cout << "Success" << std::endl;
  return 0;
}
