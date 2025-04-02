/*
 * Copyright (C) 2025: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include <Calibration_Helpers.h>
#include <iostream>

int main()
{
  std::string ret = asdp::render::calibration::Test();
  if (!ret.empty()) {
    std::cerr << "Error: " << ret << std::endl;
    return 1;
  }
  return 0;
}
