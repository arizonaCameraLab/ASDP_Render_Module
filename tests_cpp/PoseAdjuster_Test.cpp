/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include <string>
#include <iostream>
#include <PoseAdjuster.h>

int main()
{
  std::string ret = asdp::render::PoseAdjuster::Test();
  if (ret.size() > 0) {
    std::cerr << "Error in PoseAdjuster::Test(): " << ret << std::endl;
    return 1;
  }

  // Clean up resources and exit
  std::cout << "Success" << std::endl;
  return 0;
}
