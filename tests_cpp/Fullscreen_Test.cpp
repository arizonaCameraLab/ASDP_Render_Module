/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include <iostream>
#include <vector>
#include <memory>
#include <Composite.h>
#include <Display.h>
#include <ASDP_Core_API.h>

int main()
{
  int width = 1280;
  int height = 1024;
  float fps = 60.0f;

  asdp::render::ViewRenderInfo viewRenderInfo;
  viewRenderInfo.width = width;
  viewRenderInfo.height = height;
  std::vector<asdp::render::ViewRenderInfo> views;
  views.push_back(viewRenderInfo);

  // Do the remainder inside of a block so that the objects will be destroyed before
  // exiting.
  {
    // Create a client to use the ASDP API that listens on the loopback interface
    std::shared_ptr<asdp::CoreClient> client = std::make_shared<asdp::CoreClient>("localhost");

    // Create a CompositeCube object to render once the window is open and the context is active.
    std::shared_ptr<asdp::render::CompositeCube> composite = std::make_shared<asdp::render::CompositeCube>(10);

    // Create a full-screen Display window to show the CompositeCube object.
    asdp::render::DisplayWindow window("Fullscreen_Test", composite, client, 0, 0, 0, fps, 2500, width, height,
      90, "", nullptr, true);
    if (window.GetStatus() != "") {
      std::cerr << "Error opening first display: " << window.GetStatus() << std::endl;
      return 1;
    }

    // Done with the composite object -- let the display objects take over destroying it+.
    composite.reset();

    // Loop until the user closes both windows.
    std::cout << "You should see a square of varying-brightness green squares in a full-screen window." << std::endl;
    std::cout << "You should be able to rotate the views by pressing the arrow keys." << std::endl;
    std::cout << "Close the window using the keyboard shortcut to exit." << std::endl;
    auto start = std::chrono::steady_clock::now();
    while (window.GetStatus() == "") {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  // Done
  return 0;
}
