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
  int width = 640;
  int height = 640;

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

    // Create a Display window to show the CompositeCube object.
    asdp::render::DisplayWindow window("Display_Test", composite, client, 0, 0, 60.0f, 2500, width, height);
    if (window.GetStatus() != "") {
      std::cerr << "Error opening first display: " << window.GetStatus() << std::endl;
      return 1;
    }

    // Create a second Display window to show the same CompositeCube object that shares objects
    // with the original window's context.
    asdp::render::DisplayWindow window2("Display_Test2", composite, client, 0, 0, 60.0f, 2500,
      width, height,
      90, "", &window);
    if (window2.GetStatus() != "") {
      std::cerr << "Error opening second display: " << window2.GetStatus() << std::endl;
      return 2;
    }

    // Create a third Display to handle textures, sharing with the first window.
    asdp::render::DisplayTexture window3(&window);
    if (window3.GetStatus() != "") {
      std::cerr << "Error opening third display: " << window3.GetStatus() << std::endl;
      return 3;
    }

    // Borrow the OpenGL context from the third window.
    if (!window3.BorrowContext()) {
      std::cerr << "Error borrowing context from third display." << std::endl;
      return 4;
    }

    /// @todo Verify that we can write textures in the third window and have them visible in the
    /// other two.

    // Done with the composite object -- let the display objects take over destroying it+.
    composite.reset();

    // Loop until the user closes both windows.
    std::cout << "You should see a square of varying-brightness green squares in two windows." << std::endl;
    std::cout << "You should be able to move and resize the windows, with the display updating." << std::endl;
    std::cout << "You should be able to rotate the views by pressing the arrow keys." << std::endl;
    std::cout << "Close the windows to exit." << std::endl;
    auto start = std::chrono::steady_clock::now();
    while ((window.GetStatus() == "") || (window2.GetStatus() == "")) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  // Done
  return 0;
}
