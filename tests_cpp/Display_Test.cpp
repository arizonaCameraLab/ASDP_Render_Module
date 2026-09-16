/*
 * Copyright (C) 2024-2026: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <algorithm>
#include <Composite.h>
#include <Display.h>
#include <ASDP_Core_API.h>

static void usage(const char* progName)
{
  std::cerr << "Usage: " << progName << " [--openXR] [--xSight <NIC name> <display>]"
    << " [--xSight2 <NIC name> <display>]"
    << " [--xSightG <NIC name> <display> <width> <height> <fps> <hFOV> <monochrome> <port>]"
    << " [--viewpointOffset <x> <y> <z>] [--viewpointRotation <dx> <dy> <dz>]"
    << std::endl;
}

int main(int argc, char** argv)
{
  int width = 640;
  int height = 640;

  bool useOpenXR = false;

  std::string xSightNICName = "";
  int xSightDisplay = 0;
  float xSightFPS = 50.0f;
  float xSightHorizontalFOV = 70.0f;
  bool xSightMonochrome = true;
  uint16_t xSightPort = 5535;

  std::array<float, 3> viewpointOffset = { 0.0f, 0.0f, 0.0f };
  std::array<float, 3> viewpointRotation = { 0.0f, 0.0f, 0.0f };

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
      width = 2560;
      height = 2048;
      xSightFPS = 50.0f;
      xSightHorizontalFOV = 70.0f;
      xSightMonochrome = true;
      xSightPort = 5535;
    }
    else if (std::string("--xSight2") == argv[i]) {
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
      width = 1920;
      height = 1200;
      xSightFPS = 50.0f;
      xSightHorizontalFOV = 70.0f;
      xSightMonochrome = false;
      xSightPort = 5540;
    }
    else if (std::string("--xSightG") == argv[i]) {
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
      if (++i >= argc) {
        usage(argv[0]);
        return 1;
      }
      width = std::stoi(argv[i]);
      if (++i >= argc) {
        usage(argv[0]);
        return 1;
      }
      height = std::stoi(argv[i]);
      if (++i >= argc) {
        usage(argv[0]);
        return 1;
      }
      xSightFPS = static_cast<float>(atof(argv[i]));
      if (++i >= argc) {
        usage(argv[0]);
        return 1;
      }
      xSightHorizontalFOV = static_cast<float>(atof(argv[i]));
      if (++i >= argc) {
        usage(argv[0]);
        return 1;
      }
      xSightMonochrome = (std::string(argv[i]) == "true");
      if (++i >= argc) {
        usage(argv[0]);
        return 1;
      }
      xSightPort = static_cast<uint16_t>(std::stoi(argv[i]));
    }
    else if (std::string ("--viewpointOffset") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 1;
      }
      viewpointOffset[0] = static_cast<float>(atof(argv[i]));
      if (++i >= argc) {
        usage(argv[0]);
        return 1;
      }
      viewpointOffset[1] = static_cast<float>(atof(argv[i]));
      if (++i >= argc) {
        usage(argv[0]);
        return 1;
      }
      viewpointOffset[2] = static_cast<float>(atof(argv[i]));
    }
    else if (std::string("--viewpointRotation") == argv[i]) {
      if (++i >= argc) {
        usage(argv[0]);
        return 1;
      }
      viewpointRotation[0] = static_cast<float>(atof(argv[i]));
      if (++i >= argc) {
        usage(argv[0]);
        return 1;
      }
      viewpointRotation[1] = static_cast<float>(atof(argv[i]));
      if (++i >= argc) {
        usage(argv[0]);
        return 1;
      }
      viewpointRotation[2] = static_cast<float>(atof(argv[i]));
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
        0, 0, 0, viewpointOffset, viewpointRotation, 2500, 0, nullptr, nullptr, nullptr, false));
    } else if (xSightNICName != "") {
      // XSight configuration
      displays.push_back(std::make_shared<asdp::render::DisplayXSight>(xSightNICName, composite, &texWindow, client,
        0, 0, 0, viewpointOffset, viewpointRotation,
        2500, nullptr, nullptr, nullptr, false, xSightDisplay,
        width, height, xSightFPS, xSightHorizontalFOV,
        xSightMonochrome, xSightPort));
    } else {
      // Create a Display window to show the CompositeCube object that shares objects with the texWindow.
      // Control it using joystick 0.
      displays.push_back(std::make_shared<asdp::render::DisplayWindow>("Display_Test", composite, client,
        0, 0, 0, viewpointOffset, viewpointRotation, 60.0f, 2500, width, height, 90.0f, "GLFW::0", &texWindow));
      if (displays.back()->GetStatus() != "") {
        std::cerr << "Error opening first display: " << displays.back()->GetStatus() << std::endl;
        return 1;
      }

      // Create a second Display window to show another CompositeCube object that shares objects
      // with the texWindow (and therefore the first Display window).  We need it to be a different
      // Composite because the two will have different viewpoints in general.
      // Control it using joystick 1.
      std::shared_ptr<asdp::render::CompositeCube> composite2 = std::make_shared<asdp::render::CompositeCube>(10);
      displays.push_back(std::make_shared<asdp::render::DisplayWindow>("Display_Test2", composite2, client,
        0, 0, 0, viewpointOffset, viewpointRotation, 60.0f, 2500, width, height,
        90.0f, "GLFW::1", &texWindow));
      if (displays.back()->GetStatus() != "") {
        std::cerr << "Error opening second display: " << displays.back()->GetStatus() << std::endl;
        return 2;
      }

      // Loop until the user closes all displays.
      std::cout << "You should see a square of varying-brightness green squares in two windows." << std::endl;
      std::cout << "You should be able to move and resize the windows, with the display updating." << std::endl;
      std::cout << "You should be able to rotate the views by pressing the arrow keys" << std::endl;
      std::cout << "or rubber-band dragging with the mouse." << std::endl;
      std::cout << "Close all windows (ESC, q, or window close button) to exit." << std::endl;
    }

    // Done with the composite object -- let the display objects take over destroying it.
    composite.reset();

    bool done = false;
    while (!done) {
      // Very brief sleep to avoid busy-waiting but also to catch joystick and mouse events quickly.
      std::this_thread::sleep_for(std::chrono::milliseconds(1));

      // Poll for events on all of the Displays
      for (auto& display : displays) {
        display->PollEvents();
      }

      // Remove any Displays that are broken (have a non-empty status string) from the vector.
      displays.erase(std::remove_if(displays.begin(), displays.end(),
        [](const std::shared_ptr<asdp::render::Display>& display) {
          return display->GetStatus() != "";
        }), displays.end());

      // If all of our Displays have been closed (or are broken), then we're done.
      if (displays.empty()) {
        done = true;
      }
    }
  }

  // Done
  return 0;
}
