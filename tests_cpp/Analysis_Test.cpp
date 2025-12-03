/*
 * Copyright (C) 2025: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include <Analysis.h>
#include <iostream>

using namespace asdp::analysis;

int main()
{
  // Call the test program and report if there is a failure.
  std::string ret = AnalysisReport::Test();
  if (!ret.empty()) {
    std::cerr << "Error: " << ret << std::endl;
    return 1;
  }

  // It worked
  std::cout << "Success!" << std::endl;
  return 0;
}
