/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include <string>
#include <iostream>
#include <ImageQueue.h>

int main()
{
  std::string ret = asdp::render::ImageQueue::Test();
  if (ret.size() > 0) {
    std::cerr << "Error in ImageQueue::Test(): " << ret << std::endl;
    return 1;
  }

  // Clean up resources and exit
  return 0;
}
