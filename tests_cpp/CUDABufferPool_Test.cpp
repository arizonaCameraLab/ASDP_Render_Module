/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include <string>
#include <iostream>
#include <CUDABufferPool.h>

int main()
{
  std::string ret = asdp::render::CUDABufferPool::Test();
  if (ret.size() > 0) {
    std::cerr << "Error in CUDABufferPool::Test(): " << ret << std::endl;
    return 1;
  }

  // Clean up resources and exit
  std::cout << "Success" << std::endl;
  return 0;
}
