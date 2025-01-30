/*
 * Copyright (C) 2025: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include <string>
#include <iostream>
#include <RangeEstimator.h>

int main()
{
  std::string error = asdp::render::RangeEstimator::Test();
  if (!error.empty()) {
    std::cout << "Error: " << error << std::endl;
    return 1;
  }

  std::cout << "Success" << std::endl;
  return 0;
}
