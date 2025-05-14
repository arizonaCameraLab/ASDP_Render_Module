/*
 * Copyright (C) 2024-2025: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <Composite.h>
#include <Display.h>
#include <ASDP_Core_API.h>

static void usage(const char* progName)
{
  std::cerr << "Usage: " << progName << " [--openXR] [--xSight <NIC name> <display>]" << std::endl;
}

int main(int argc, char** argv)
{
  int width = 640;
  int height = 640;

  bool useOpenXR = false;
  std::string xSightNICName = "";
  int xSightDisplay = 0;
  for (int i = 1; i < argc; ++i) {
    if (std::string("--openXR") == argv[i]) {
      useOpenXR = true;
    }
    else if (std::string("--xSight") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 1;
      }
      xSightNICName = argv[i];
      if (++i >= argc) {
        usage(argv[0]);
        return 1;
      }
      xSightDisplay = std::stoi(argv[i]);
    }
    else {
      usage(argv[0]);
      return 2;
    }
  }

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

    // Create a Display to handle textures, sharing its context with the other windows.
    asdp::render::DisplayTexture texWindow;
    if (texWindow.GetStatus() != "") {
      std::cerr << "Error opening third display: " << texWindow.GetStatus() << std::endl;
      return 3;
    }

    std::vector< std::shared_ptr<asdp::render::Display> > displays;

    // Create the appropriate Display object(s) based on the command-line arguments.
    if (useOpenXR) {
      displays.push_back(std::make_shared<asdp::render::DisplayOpenXR>(composite, &texWindow, client,
        0, 0, 0, 2500, 0, nullptr, nullptr, nullptr, false));
    } else if (xSightNICName != "") {
      displays.push_back(std::make_shared<asdp::render::DisplayXSight>(xSightNICName, composite, &texWindow, client,
        0, 0, 0,
        2500, nullptr, nullptr, nullptr, false, xSightDisplay));
    } else {
      // Create a Display window to show the CompositeCube object that shares objects with the texWindow.
      // Control it using joystick 0.
      displays.push_back(std::make_shared<asdp::render::DisplayWindow>("Display_Test", composite, client,
        0, 0, 0, 60.0f, 2500, width, height, 90, "GLFW::0", &texWindow));
      if (displays.back()->GetStatus() != "") {
        std::cerr << "Error opening first display: " << displays.back()->GetStatus() << std::endl;
        return 1;
      }

      // Create a second Display window to show the same CompositeCube object that shares objects
      // with the texWindow (and therefore the first Display window).
      // Control it using joystick 1.
      displays.push_back(std::make_shared<asdp::render::DisplayWindow>("Display_Test2", composite, client,
        0, 0, 0, 60.0f, 2500, width, height,
        90, "GLFW::1", &texWindow));
      if (displays.back()->GetStatus() != "") {
        std::cerr << "Error opening second display: " << displays.back()->GetStatus() << std::endl;
        return 2;
      }

      // Loop until the user closes all displays.
      std::cout << "You should see a square of varying-brightness green squares in two windows." << std::endl;
      std::cout << "You should be able to move and resize the windows, with the display updating." << std::endl;
      std::cout << "You should be able to rotate the views by pressing the arrow keys." << std::endl;
      std::cout << "Close the windows to exit." << std::endl;
    }

    // Done with the composite object -- let the display objects take over destroying it.
    composite.reset();

    bool done = false;
    while (!done) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));

      // If all of our Displays have been closed (or are broken), then we're done.
      bool allClosed = true;
      for (auto& display : displays) {
        if (display->GetStatus() == "") {
          allClosed = false;
          break;
        }
      }
      if (allClosed) {
        done = true;
      }
    }
    std::cout << "Final display status: " << displays[0]->GetStatus() << std::endl;
  }

  // Done
  return 0;
}
