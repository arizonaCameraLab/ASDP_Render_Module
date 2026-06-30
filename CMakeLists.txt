cmake_minimum_required(VERSION 3.16.0)

if(UNIX)
# @todo Add 100;120 once we've upgraded to a newer version of Linux that has CUDA 13.0 available
set(CMAKE_CUDA_ARCHITECTURES "75;80;89")
else(UNIX)
set(CMAKE_CUDA_ARCHITECTURES "75;80;89;100;120")
endif(UNIX)
project(ASDP_Render_Module LANGUAGES CUDA CXX C)

# We require C++17 for the use of std::filesystem.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
list(APPEND CMAKE_PREFIX_PATH
  F:/Packages/GLEW/glew-2.2.0
  "C:/Program Files/opencv/build/x64/vc16/lib"
  C:/Qt/5.15.2/msvc2019_64/lib/cmake/Qt5
)

#-----------------------------------------------------------------------------
if(WIN32)
    add_definitions(-DNOMINMAX)
endif(WIN32)
if(UNIX)
    add_definitions(-DOS_LINUX_XCB_GLX)
endif(UNIX)

#-----------------------------------------------------------------------------
# Local CMake Modules

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

#-----------------------------------------------------------------------------
# Enable sorting projects within the solution on Visual Studio, to group
# Test and Example files together.

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

#-----------------------------------------------------------------------------
# Fetch and build packages

include(FetchContent)
FetchContent_Declare(
  json
  GIT_REPOSITORY https://github.com/arizonaCameraLab/json.git
  GIT_TAG        9cca280a4d0ccf0c08f47a99aa71d1b0e52f8d03 # Master as of 5/20/2024
)
FetchContent_MakeAvailable(json)

# Set Geogram cache variables explicitly before running FetchContent_Declare
set(GEOGRAM_LIB_ONLY ON CACHE BOOL "Build Geogram as a library only")
set(GEOGRAM_WITH_GRAPHICS OFF CACHE BOOL "Disable Geogram graphics")
set(GEOGRAM_WITH_LEGACY_NUMERICS OFF CACHE BOOL "Disable Geogram legacy numerics")
set(GEOGRAM_WITH_HLBFGS OFF CACHE BOOL "Disable Geogram HLBFGS")
set(GEOGRAM_WITH_TETGEN OFF CACHE BOOL "Disable Geogram Tetgen")
set(GEOGRAM_WITH_TRIANGLE OFF CACHE BOOL "Disable Geogram Triangle")
set(GEOGRAM_WITH_LUA OFF CACHE BOOL "Disable Geogram Lua")
set(GEOGRAM_WITH_FPG OFF CACHE BOOL "Disable Geogram FPG")
set(GEOGRAM_USE_SYSTEM_GLFW3 OFF CACHE BOOL "Use system GLFW3")
set(GEOGRAM_WITH_GARGANTUA OFF CACHE BOOL "Disable Geogram Gargantua")
set(GEOGRAM_WITH_TBB OFF CACHE BOOL "Disable Geogram TBB")
if(UNIX)
set(VORPALINE_PLATFORM "Linux64-gcc" CACHE STRING "Set the Vorpaline platform")
set(VORPALINE_BUILD_DYNAMIC OFF CACHE BOOL "Build Geogram with dynamic library")
endif()

FetchContent_Declare(
  Geogram
  GIT_REPOSITORY https://github.com/BrunoLevy/geogram.git
  GIT_TAG        v1.9.5
  GIT_SUBMODULES_RECURSE TRUE
)
FetchContent_MakeAvailable(Geogram)

# Geogram built GLFW3 for us as part of its build so we just need to find it.
find_package(glfw3 REQUIRED)

# Before we used Geogram, we built GLFW3 ourselves on Windows and used a system library
# on Linux.
#if(WIN32)
#FetchContent_Declare(
#  glfw3
#  GIT_REPOSITORY https://github.com/arizonaCameraLab/glfw.git
#  GIT_TAG        b35641f4a3c62aa86a0b3c983d163bc0fe36026d # Master as of 4/26/2024
#)
#FetchContent_MakeAvailable(glfw3)
#else(WIN32)
#find_package(glfw3 REQUIRED)
#endif(WIN32)

# We build FreeType ourselves to ensure compatibility across platforms.
add_subdirectory(dependencies/freetype)

#-----------------------------------------------------------------------------
# Checking for optional and required packages

find_package(Threads REQUIRED)
find_package(ASDP_Core_API REQUIRED)
find_package(OpenGL REQUIRED COMPONENTS OpenGL)
find_package(GLEW REQUIRED)
find_package(Doxygen)
find_package(OpenMP)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()
find_package(OpenXR)
find_package(OpenCV COMPONENTS core imgproc calib3d)

# Needed to enable compiling on Ubuntu 22.04
find_package(CUDAToolkit REQUIRED)
include_directories(${CUDAToolkit_INCLUDE_DIRS})

#-----------------------------------------------------------------------------
# Define build options.

option(BUILD_TESTS "Build test programs" ON)
option(BUILD_UTILITIES "Build utility programs" ON)

if (DOXYGEN_FOUND)
    option(BUILD_DOXYGEN "Build DOxygen documentation" ON)
endif (DOXYGEN_FOUND)

if (BUILD_DOXYGEN)
    # set input and output files
    set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/docs/Doxyfile.in)
    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

    # request to configure the file
    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)

    # note the option ALL which allows to build the docs together with the application
    add_custom_target( doc_doxygen ALL
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM )
endif (BUILD_DOXYGEN)

#-----------------------------------------------------------------------------
# Build the libraries.

add_subdirectory(Spot_Tracker)
add_subdirectory(Gimbal)

set (Render_Module_HEADERS
  CameraRenderInfo.h
  Composite.h
  Display.h
  ImageQueue.h
  Distortion.h
  ToneMap.h
  CPUDataToTextureHandler.h
  CUDABufferPool.h
  PoseAdjuster.h
  DepthEstimator.h
  Vignette.h
  ImageStatistics.h
  RangeEstimator.h
  RenderTimingInfo.h
  PointCorrespondences.h
  RenderText.h
  RenderHaloedLines.h
  Analysis.h
)
set (Render_Module_SOURCES
  CameraRenderInfo.cpp
  Composite.cpp
  Display.cpp
  ImageQueue.cpp
  Distortion.cpp
  ToneMap.cpp
  CPUDataToTextureHandler.cu
  CUDABufferPool.cpp
  PoseAdjuster.cpp
  DepthEstimator.cu
  Vignette.cpp
  ImageStatistics.cu
  RangeEstimator.cpp
  PointCorrespondences.cpp
  RenderText.cpp
  RenderHaloedLines.cpp
  Analysis.cpp
)
add_library(Render_Module STATIC ${Render_Module_SOURCES} ${Render_Module_HEADERS})
target_include_directories(Render_Module PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>
  "${CMAKE_CURRENT_BINARY_DIR}/Install/include"
  "${CMAKE_CURRENT_SOURCE_DIR}/dependencies/glm"
)
target_include_directories(Render_Module PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/dependencies/glm"
)
target_link_libraries(Render_Module PUBLIC
  ASDP_Core_API::ASDP_Core_API glfw GLEW::glew OpenGL::GL geogram freetype nlohmann_json::nlohmann_json
)
if(OPENXR_FOUND)
  target_compile_definitions(Render_Module PRIVATE XR_USE_GRAPHICS_API_OPENGL)
  if(WIN32)
      target_compile_definitions(Render_Module PRIVATE USE_OPENXR=1 XR_USE_PLATFORM_WIN32)
  elseif(PRESENTATION_BACKEND MATCHES "xlib")
      target_compile_definitions(Render_Module PRIVATE USE_OPENXR=1 XR_USE_PLATFORM_XLIB)
  elseif(PRESENTATION_BACKEND MATCHES "xcb")
      target_compile_definitions(Render_Module PRIVATE USE_OPENXR=1 XR_USE_PLATFORM_XCB)
  elseif(PRESENTATION_BACKEND MATCHES "wayland")
      target_compile_definitions(Render_Module PRIVATE USE_OPENXR=1 XR_USE_PLATFORM_WAYLAND)
  endif()
  target_include_directories(Render_Module PRIVATE
      "${CMAKE_CURRENT_SOURCE_DIR}/openxr-sdk-files"
  )
  target_link_libraries(Render_Module PRIVATE
    OpenXR::openxr_loader
  )
endif()

add_library(Calibration_Helpers STATIC util/Calibration_Helpers.cpp util/Calibration_Helpers.h)
include_directories(Calibration_Helpers PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/util"
  "${CMAKE_CURRENT_SOURCE_DIR}/dependencies/glm"
)
target_link_libraries(Calibration_Helpers PUBLIC
  Render_Module
  nlohmann_json::nlohmann_json
  Gimbal
)

#-----------------------------------------------------------------------------
# Build the Render module application.

set (ASDP_Render_Module_HEADERS
)

set (ASDP_Render_Module_SOURCES
    ASDP_Render_Module.cpp
)

add_executable(ASDP_Render_Module ${ASDP_Render_Module_SOURCES} ${ASDP_Render_Module_HEADERS})
target_compile_definitions(ASDP_Render_Module PRIVATE
  -DCONFIG_FILE_PATH="${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}/ASDP_Render_Module_Configs")
target_link_libraries(ASDP_Render_Module Render_Module Calibration_Helpers nlohmann_json::nlohmann_json)
get_property(isMultiConfig GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
if(isMultiConfig)
  # Multi-config (e.g., Visual Studio, Xcode)
  target_compile_definitions(ASDP_Render_Module PRIVATE BUILD_TYPE="$<CONFIG>")
else()
  # Single-config (e.g., Makefiles, Ninja)
  if(CMAKE_BUILD_TYPE AND NOT CMAKE_BUILD_TYPE STREQUAL "")
    target_compile_definitions(ASDP_Render_Module PRIVATE BUILD_TYPE="${CMAKE_BUILD_TYPE}")
  else()
    target_compile_definitions(ASDP_Render_Module PRIVATE BUILD_TYPE="Default")
  endif()
endif()

install(TARGETS ASDP_Render_Module EXPORT ${PROJECT_NAME}
  RUNTIME DESTINATION bin COMPONENT bin
  LIBRARY DESTINATION lib${LIB_SUFFIX} COMPONENT lib
  ARCHIVE DESTINATION lib${LIB_SUFFIX} COMPONENT lib
  INCLUDES DESTINATION include
  PUBLIC_HEADER DESTINATION include
)

# Install the directory of configuration files into the install binary directory
#install(DIRECTORY ${CMAKE_SOURCE_DIR}/ASDP_Render_Module_Configs DESTINATION ${CMAKE_BINARY_DIR})
install(DIRECTORY ${CMAKE_SOURCE_DIR}/ASDP_Render_Module_Configs DESTINATION ${CMAKE_INSTALL_BINDIR})

if(WIN32)
  # Function to find an element in a list that contains a specified substring
  function(find_element_with_substring result list substring)
      set(found_element "")
      foreach(element IN LISTS list)
          string(FIND "${element}" "${substring}" pos)
          if(NOT pos EQUAL -1)
              set(found_element "${element}")
              break()
          endif()
      endforeach()
      set(${result} "${found_element}" PARENT_SCOPE)
  endfunction()

  # Assuming GLEW::glew is the imported target from find_package(GLEW REQUIRED)
  # Use that to find all of the glew32.dll files in neighboring directories and then pick the one
  # that has "x64" in the path.
  get_target_property(GLEW_INCLUDE_DIRS GLEW::glew INTERFACE_INCLUDE_DIRECTORIES)
  file(GLOB_RECURSE GLEW_DLL "${GLEW_INCLUDE_DIRS}/../*glew32.dll")
  find_element_with_substring(GLEW_DLL2 "${GLEW_DLL}" "x64")

  if(GLEW_DLL2)
    # Copy the GLEW DLL to the build directory after building the executable
    add_custom_command(TARGET ASDP_Render_Module POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different ${GLEW_DLL2} $<TARGET_FILE_DIR:ASDP_Render_Module>
      COMMENT "Copying ${GLEW_DLL} to build directory"
    )

    # Install the GLEW DLL to the install directory
    install(FILES ${GLEW_DLL2} DESTINATION ${CMAKE_INSTALL_BINDIR})
    MESSAGE("Installing ${GLEW_DLL2} to ${CMAKE_INSTALL_BINDIR}")
  else()
    message(WARNING "GLEW DLL not found. Make sure glew32.dll is in the same path as your GLEW include directory.")
  endif()
endif()

#-----------------------------------------------------------------------------
# Build utilities if we've been asked to.

if(BUILD_UTILITIES)

  set (CPP_UTILS
    Texture_Coordinates
    Compare_Configurations
    Target_Calibration_Make_Scan
    Target_Calibration_Estimate_Lateral
    Camera_Calibration_Make_Scan
    Collect_Calibration_Data
    Camera_Calibration_Estimate_Distortion_Extrinsics
    Camera_Calibration_Gain_And_Offsets
    Camera_Calibration_Per_Pixel_Gain_And_Offsets
    Print_PointCorrespondences
    Parse_Analysis_JSON
  )
  foreach (BASE ${CPP_UTILS})
    set (APP ${BASE})
    add_executable (${APP} util/${BASE}.cpp)
    set_target_properties(${APP} PROPERTIES FOLDER util/C++)
    target_link_libraries(${APP} PRIVATE
      Render_Module
      spot_tracker_library
      ASDP_Core_API::ASDP_Core_API
      glfw
      GLEW::glew
      OpenGL::GL
      Calibration_Helpers
      Gimbal
    )
    if(OPENCV_FOUND)
      target_compile_definitions(${APP} PRIVATE USE_OPENCV)
      target_link_libraries(${APP} PRIVATE opencv_calib3d opencv_imgproc opencv_core)
    endif()
    install(TARGETS ${APP} EXPORT ${PROJECT_NAME}
      RUNTIME DESTINATION bin
    )
  endforeach (BASE)

  set (CUDA_UTILS
	  Time_CUDA_Writes
    Speed_Test_Receiver_CUDA
  )
  foreach (BASE ${CUDA_UTILS})
    set (APP ${BASE})
    add_executable (${APP} util/${BASE}.cu)
    set_target_properties(${APP} PROPERTIES FOLDER util/C++)
    # We need to explicitly state that we need all CUDA files in the 
    # project folder to be built with -dc as the option to nvcc so 
    # CUDA separable compilation is enabled.
    set_property(TARGET ${APP} 
                 PROPERTY CUDA_SEPARABLE_COMPILATION ON)
    target_include_directories(${APP} PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}
      ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/glm
    )
    target_link_libraries(${APP} PRIVATE
	    Render_Module
    )
    install(TARGETS ${APP} EXPORT ${PROJECT_NAME}
      RUNTIME DESTINATION bin
    )
  endforeach (BASE)

endif(BUILD_UTILITIES)

#-----------------------------------------------------------------------------
# Build tests if we've been asked to.

if(BUILD_TESTS)

  enable_testing()

  set (CPP_TESTS
    ImageQueue_Test
    Composite_Test
    CompositeCameras_Test
    CompositeLineRawData_Test
    CompositePackXSightFrame_Test
    SharedContext_Test
    Display_Test
    Fullscreen_Test
    Distortion_Test
    ToneMap_Test
    CPUDataToTextureHandler_Test
    CUDABufferPool_Test
    PoseAdjuster_Test
    DepthEstimator_Test
    Vignette_Test
    ImageStatistics_Test
    RangeEstimator_Test
    XSight_Simulator
    Spot_Tracker_Test
    Calibration_Helpers_Test
    Gimbal_Test
    RenderText_Test
    RenderHaloedLines_Test
    Annotation_Test
    Analysis_Test
  )
  foreach (BASE ${CPP_TESTS})
    set (APP ${BASE})
    add_executable (${APP} tests_cpp/${BASE}.cpp)
    set_target_properties(${APP} PROPERTIES FOLDER tests/C++)
    target_include_directories(${APP} PRIVATE
      "${CMAKE_CURRENT_SOURCE_DIR}"
      "${CMAKE_CURRENT_SOURCE_DIR}/dependencies/glm"
    )
    target_link_libraries(${APP}
      Render_Module
      spot_tracker_library
      Calibration_Helpers
      Gimbal
    )
    install(TARGETS ${APP} EXPORT ${PROJECT_NAME}
      RUNTIME DESTINATION bin
    )
    add_test(${APP} ${APP})
  endforeach (BASE)

  set (CU_TESTS
    CUDA_To_Texture_Test
    Sum_and_Squared_Speed_Test
  )
  foreach (BASE ${CU_TESTS})
    set (APP ${BASE})
    add_executable (${APP} tests_CUDA/${BASE}.cu)
    set_target_properties(${APP} PROPERTIES FOLDER tests/CUDA)
    target_include_directories(${APP} PRIVATE
      "${CMAKE_CURRENT_SOURCE_DIR}"
      "${CMAKE_CURRENT_SOURCE_DIR}/dependencies/glm"
    )
    target_link_libraries(${APP}
      Render_Module
    )
    install(TARGETS ${APP} EXPORT ${PROJECT_NAME}
      RUNTIME DESTINATION bin
    )
    add_test(${APP} ${APP})
  endforeach (BASE)

endif(BUILD_TESTS)

#-----------------------------------------------------------------------------
# Build the Storage Manager if we find Qt version 5.

# Find the Qt5 package
find_package(Qt5 COMPONENTS Core Widgets Gui)

if (Qt5_FOUND)
  # Add the Storage Manager project
  add_subdirectory(Storage_Manager_app)
endif()
