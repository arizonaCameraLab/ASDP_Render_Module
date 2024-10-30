# ASDP_Render_Module

This repository contains the source code for a Render Module 
for the Apache Strap-Down Pilotage program.

## Getting Started

This Render module must be cloned recursively so that it pulls in all of its submodules:
`git clone --recursive https://github.com/arizonaCameraLab/ASDP_Render_Module`

**Summary:** Quick instructions to get set up on Linux for Storage and Render Server systems
(replace X with the serial number of the systems being used [1-4] in the IP addresses):

- Storage Server:
    - Build and install ASDP_Core_API, ASDP_Core_Module, and ASDP_Camera_Simulator (in that order).
    - ASDP_Core_Module 10.10.10.X1 --verbosity 100
- Render Server:
    - Build and install ASDP_Core_API and ASDP_Render_Module (in that order).
    - cd /usr/local/bin/ASDP_Render_Module_Configs; sudo cp 0_21cam.json 0.json
    - ASDP_Render_Module 10.10.10.X2

**Detailed requirements:** The ASDP_Render_Module requires the ASDP_Core_API library to have been
installed before it is built.  This installs by default in a known system location that can
be found automatically.  The library source can be obtained using:
`git clone https://github.com/arizonaCameraLab/ASDP_Core_API` and instructions in the
repository tell how to install it.

The following packages are required (apt install) to build on Linux:
- libglfw3-dev
- libglew-dev

To upgrade a server-only Mint distribution (with added nVidia drivers) on a Render Server to a
desktop environment that uses the light-weight XFCE desktop but does not include the printer daemon,
use the following commands:

    sudo apt install xfce4
    sudo apt remove cups
    sudo apt autoremove

and then make (as root) the file */etc/lightdm/lightdm.conf* with the following contents:

    [SeatDefaults]
    user-session=xfce

and then reboot.

Then run the following: `sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target`
to prevent the system from suspending or sleeping when inactive.

**Note:** On Linux, joysticks must be plugged into the USB ports at the top back of the Render
Server to be recognized by the system.  This is also true of the keyboard and mouse.

On Windows this module requires GLEW to be installed. Pre-built binaries are available for many systems at
https://github.com/nigels-com/glew/releases/tag/glew-2.2.0 and these can be unzipped anywhere on
the system and the path to the include and lib directories specified in the CMakeLists.txt file
by adding space-separated entries to the CMAKE_PREFIX_PATH on line 7. If there are spaces in the
path, the entry must be enclosed in double quotes.  For example. if GLEW is unzipped to
C:/glew-2.2.0, the line would be:

    list(APPEND CMAKE_PREFIX_PATH "C:/glew-2.2.0" F:/Packages/GLEW/glew-2.2.0)

Multiple entries are allowed in this line, so feel free to keep adding entries as needed
on different machines and leave the existing ones in the file.

**Build:** ASDP_Render_Module uses CMake to configure the builds (though other build
systems could be used).  On Ubuntu Linux, this can be done as follows

    sudo apt install cmake
    cd; mkdir src; cd src; git clone https://github.com/arizonaCameraLab/ASDP_Render_Module
    cd; mkdir -p build/ASDP_Render_Module; cd build/ASDP_Render_Module
    cmake ../../src/ASDP_Render_Module
    make

**Run:** The *ASDP_Render_Module* program runs a render module and displays output on one
or more devices.  It takes a required command-line argument that specifies the IP address
of the network interface card to listen on (perhaps "localhost" for testing).  It has the
following optional arguments:
- **--frameStride** The number of frames to skip between rendering frames.  This can be used to
  reduce the load on the system.  A value of 1 renders every frame, 2 renders every other frame.
- **--width** The width of the window, default 1280.
- **--height** The height of the window, default 1024.
- **--fullscreen** If present, the window will be full-screen.  The display number to use is given
  as an argument.  For example, "--fullscreen 0" will make the window full-screen on display 1.
- **--fps** The number of frames per second to render, default 60.
- **--joystick** The joystick to use for control, for example GLFW::0 will use the first joystick.
- **--hFOV** The horizontal field of view of the camera in degrees, default 40.
- **--toneMap** The tone mapping to use, default linear.  Other options are blackbody and bluesky.
- **--addDisplay** This will add a second display window with default settings; these settings can
  be overridden with additional arguments.  When making one of the displays full-screen, it must be
  the last one added so that it maintains focus.
- **--replay** Rather than running live, replay the specified stored stream ID (1+).
- **--lineBatchesPerGPUSend** The number of line batches to send to the GPU at a time, default 16
  under Linux and 32 under Windows. This trades throughput for latency.
- **--openXR** Display using an OpenXR device, using its specified resolution and field of view.

For example, to run the program on interface 10.10.10.22 with two windows, one full-screen on
display 1 and the other having default parameters with both using different joysticks,
the command might be:

    ASDP_Render_Module 10.10.10.22 --joystick GLFW::0 --addDisplay --fullscreen 1 --width 7680 --height 4320 --fps 60 --hFOV 90 --joystick GLFW::1

**Documentation:** The primary documentation is available in DOxygen, and
is generated by as part of the build process when DOxygen is available.  On Ubuntu
Linux, this can be generated as follows:
* `sudo apt install doxygen`
* When you build using CMake, it will build DOxygen by default.
* Open doc_doxygen/html/index.html with a web browser to view the documentation.

**Test:** CMake includes the concept of test applications. You can run the tests
by running `make test` in the build directory.

## Configuration

The **ASDP_Render_Module_Configs** directory contains a set of JSON-format camera-configuration files named
after the serial number of the camera they represent. For camera 1, the file is 1.json.  This directory
is copied into the same directory as the binary files during the build and install process.  The format
of each file is as follows, with an entry for each camera.  This example file is for a sample camera with
serial number 0 that has two microcameras so it would be saved in *0.json*.

There are several files with different numbers of cameras for the 0th system, named like *0_8cam.json*.
These have layouts for different numbers of cameras from 4 through 25. To use one of these layouts, copy
the file to *0.json* in the install directory before running the program.

```
{
  "serialNumber" : 0,
  "cameras" : [
    {
      "id" : 1,
      "positionMeters" : [0.0, 0.0, 0.0],
      "orientationDegrees" : [0.0, 0.0, 0.0],
      "resolutionPixels" : [1280, 1024],
      "fieldOfViewDegrees" : [40.0, 32.5],
      "distortion" : {
        "type" : "radial",
        "parameters" : {
          "COP" : [0.0, 0.0],
          "map" : [[0, 0], [1, 1]]
        }
      }
    },
    {
      "id" : 2,
      "positionMeters" : [0.0, 0.0, 0.0],
      "orientationDegrees" : [30.0, 0.0, 0.0],
      "resolutionPixels" : [1280, 1024],
      "fieldOfViewDegrees" : [40.0, 32.5],
      "distortion" : {
        "type" : "none",
        "parameters" : {}
      }
    }
  ]
}
```

The cameras are placed in the helicopter coordinate system described below.

The **distortion** field is an object that has a *type* field that specifies the type of distortion
and a *parameters* field that specifies the parameters of the distortion.  The type field can be
"none" for no distortion or "radial" for radial distortion (other approaches may be added).  The
parameters field depends on the type of distortion.  For none distortion, the parameters are empty.
For radial distortion, the parameters are as follows:
- **COP:** The center of projection of the camera in fraction of the sensor in the range [-1..1] for
  each axis.  This is the point in the image that is not distorted.  A value of [0.0, 0.0] is the center of the image.
  A value of [1.0, 1.0] is the upper right corner of the image.  A value of [-1.0, -1.0] is the lower left corner.
- **map:** A list of points with the first one being [0,0] and the later ones in increasing order that specify
  the ideal-camra radius and its distorted radius.  These are for points that are projected onto the 
  Z = -1 plane.  They must span the entire range of the image (including the corners).  For example, a
  distortion that increased the distance by a factor of 2 could be specified by the list [[0,0], [1,2]] for
  a camera whose field of view is less than 45 degrees at its corners, with the second entry changed to
  [3, 6] for a wider field of view.

## Utilities

The **util** directory contains a number of utilities.
- **Time_CUDA_Writes** is a utility to measure the time it takes to write frames from pinned CPU memory to to GPU memory.

## Coordinate systems

**Helicopter:** The figure below shows the coordinate system of the helicopter and how it relates to the
position and orientation reported by the API.  The latitude, longitude, and altitude determine the
position of the helicopter.  The local orientation is reported with respect to a coordinate system
that has +X pointing East, +Y pointing North, and +Z pointing up.
(This coordinate system fails at the North and South poles.)

The velocity and rotational velocity are reported in the local helicopter coordinate system.
This means that translation in +Y is always straight ahead and a positive rotation around X is always
tipping backwards no matter the orientation of the helicopter.

![Helicopter Coordinate System](helicopter_coordinates.png "Helicopter coordinates")

**CameraRenderInfo:** The center of the coordinate system is the origin point of the camera array
(the entire chassis that holds all cameras).  The orientation is with respect to the local helicopter
coordinate system, with +X pointing right, +Y pointing forwards, and +Z pointing up.  (This system
is nested with the helicopter coordinate system.)  The camera center of projection is first translated
by the specified offset and then rotated about this new center, first around X then around the new
Y, then around the new Z axis.  For example, a camera that is in portrait mode that is slightly ahead
of the camera center looking straight forward with its X axis down would have an offset of (0, 0.1, 0)
and a rotation of (0, 90, 0). If its X axis is pointing up, then its rotation would be (0, -90, 0).
The camera's local coordinate system has it looking along the -Y axis with the +Z axis up and the +X
axis to the right.

**ViewRenderInfo:** These transformations are also specified in the local helicopter space.
The center of projection of the camera is specified by the offset and its orientation by the rotation.
The origin of the coordinate system is the center of the camera array.
Translations in -Y move the virtual camera backwards (into the helicopter) and translations of +Z
move the virtual camera up.
A rotation around the +X axis will tip the camera's view up, and a rotation around the +Z axis will
pan the camera's view left.
The camera is looking out the front of the helicopter (along the +Y axis) with its "up" vector
pointing above the helicopter (along the +Z axis) when the orientation is (0,0,0).

**PoseAdjuster:** This class estimates the differential motion points in helicopter space
between two times.  It moves points according to how the helicopter has moved and rotated between
these times in its own local coordinate system.  It provides a transformation describing how to
transform points in the space of the earlier time parameter into the space of the later time parameter.
If the second time parameter is the expected time of scan-out and the first is the time an image was
acquired (an earlier time), then this transformation can be used to transform the vertices of the
image representation to remove the effects of the helicopter motion.

## Validation

The **tests_cpp** directory contains a number of tests.  Some of these tests require a viewer to
examine a graphical output; these, along with their expected outputs, are described below.

**Composite_Test:** This program displays a spinning cube that has varying-brightness colored
sides. Red = +X, Green = +Y, Blue = +Z, Magenta = -X, Yellow = -Y, Cyan = -Z.  The initial frame
is looking in +Y at green and it rotates around the horizontal axis rapidly and the vertical axis slowly.
It center of rotation is closer to the magenta wall than to the red wall.  One frame is shown below.

![Test of the Composite class](Composite_Test.png "Test of the Composite class")

**CompositeCameras_Test:** This displays a set of cameras that are arranged in three rows and
three columns.
You should see a row of three distorted dark boxes horizontally across the center of the view,
the first and third brighter on the left and the second brighter on the right.
Above should be brighter extensions and below should be darker ones.
The extensions meet at dark and then bright boundaries from left to right.
The expected image is shown below.

![Test of the CompositeCameras class](CompositeCameras_Test.png "Test of the CompositeCameras class")

**SharedContext_Test:** This displays a red ramp from dark to bright from the top of the image to the
bottom. The expected image is shown below.

![Test of shared OpenGL contexts](SharedContext_Test.png "Test of shared OpenGL contexts")

**Display_Test:** This displays two windows, each with a keyboard-controllable cube.  The arrow keys
control cube rotation, as do up to two plugged-in joysticks.
The expected initial image in each window is shown below.

![Test of the Display class](Display_Test.png "Test of the Display class")

**Fullscreen_Test:** This a full-screen 1280x1024 window at 60Hz with a keyboard-controllable cube.  The arrow keys
control cube rotation, as do up to two plugged-in joysticks.
The expected initial image matches that of one window in the Display_Test (shown above).
