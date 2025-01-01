cmake_minimum_required(VERSION 3.16.0)
project(ASDP_Render_Module LANGUAGES CUDA CXX C)

# We require C++17 for the use of std::filesystem.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
list(APPEND CMAKE_PREFIX_PATH F:/Packages/GLEW/glew-2.2.0)

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
if(WIN32)
FetchContent_Declare(
  glfw3
  GIT_REPOSITORY https://github.com/arizonaCameraLab/glfw.git
  GIT_TAG        b35641f4a3c62aa86a0b3c983d163bc0fe36026d # Master as of 4/26/2024
)
FetchContent_MakeAvailable(glfw3)
else(WIN32)
find_package(glfw3 REQUIRED)
endif(WIN32)

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

set (Render_Module_HEADERS
  Composite.h
  Display.h
  ImageQueue.h
  Distortion.h
  ToneMap.h
  CPUDataToTextureHandler.h
  PinnedBufferPool.h
  GPUBufferPool.h
  PoseAdjuster.h
  DepthEstimator.h
  Vignette.h
)
set (Render_Module_SOURCES
  Composite.cpp
  Display.cpp
  ImageQueue.cpp
  Distortion.cpp
  ToneMap.cpp
  CPUDataToTextureHandler.cu
  PinnedBufferPool.cpp
  GPUBufferPool.cpp
  PoseAdjuster.cpp
  DepthEstimator.cu
  Vignette.cpp
)
add_library(Render_Module STATIC ${Render_Module_SOURCES} ${Render_Module_HEADERS})
target_include_directories(Render_Module PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>
  "${CMAKE_CURRENT_BINARY_DIR}/Install/include"
  "${CMAKE_CURRENT_SOURCE_DIR}/dependencies/glm"
)
target_include_directories(Render_Module PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/dependencies/glm"
)
target_link_libraries(Render_Module PUBLIC
  ASDP_Core_API::ASDP_Core_API glfw GLEW::glew OpenGL::GL
)
set_property(TARGET Render_Module PROPERTY CUDA_ARCHITECTURES 60 75 80)
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
target_link_libraries(ASDP_Render_Module Render_Module nlohmann_json::nlohmann_json)

install(TARGETS ASDP_Render_Module EXPORT ${PROJECT_NAME}
  RUNTIME DESTINATION bin COMPONENT bin
  LIBRARY DESTINATION lib${LIB_SUFFIX} COMPONENT lib
  ARCHIVE DESTINATION lib${LIB_SUFFIX} COMPONENT lib
  INCLUDES DESTINATION include
  PUBLIC_HEADER DESTINATION include
)

# Install the directory of configuration files into the build and install binary directories
install(DIRECTORY ${CMAKE_SOURCE_DIR}/ASDP_Render_Module_Configs DESTINATION ${CMAKE_BINARY_DIR})
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

  if(GLEW_DLL)
    # Copy the GLEW DLL to the build directory after building the executable
    add_custom_command(TARGET ASDP_Render_Module POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
      ${GLEW_DLL} $<TARGET_FILE_DIR:ASDP_Render_Module>
      COMMENT "Copying ${GLEW_DLL} to build directory"
    )

    # Install the GLEW DLL to the install directory
    install(FILES ${GLEW_DLL} DESTINATION ${CMAKE_INSTALL_BINDIR})
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
  )
  foreach (BASE ${CPP_UTILS})
    set (APP ${BASE})
    add_executable (${APP} util/${BASE}.cpp)
    set_target_properties(${APP} PROPERTIES FOLDER util/C++)
    target_link_libraries(${APP} PRIVATE
	    Render_Module nlohmann_json::nlohmann_json
      ASDP_Core_API::ASDP_Core_API
      glfw GLEW::glew OpenGL::GL
    )
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
    set_property(TARGET ${APP} PROPERTY CUDA_ARCHITECTURES 60 75 80)
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
    SharedContext_Test
    Display_Test
    Fullscreen_Test
    Distortion_Test
    ToneMap_Test
    CPUDataToTextureHandler_Test
    PinnedBufferPool_Test
    GPUBufferPool_Test
    PoseAdjuster_Test
    DepthEstimator_Test
    Vignette_Test
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
    )
    install(TARGETS ${APP} EXPORT ${PROJECT_NAME}
      RUNTIME DESTINATION bin
    )
    add_test(${APP} ${APP})
  endforeach (BASE)

  set (CU_TESTS
    CUDA_To_Texture_Test
  )
  foreach (BASE ${CU_TESTS})
    set (APP ${BASE})
    add_executable (${APP} tests_CUDA/${BASE}.cu)
    set_target_properties(${APP} PROPERTIES FOLDER tests/CUDA)
    target_include_directories(${APP} PRIVATE
      "${CMAKE_CURRENT_SOURCE_DIR}"
      "${CMAKE_CURRENT_SOURCE_DIR}/dependencies/glm"
    )
    set_property(TARGET ${APP} PROPERTY CUDA_ARCHITECTURES 60 75 80)
    target_link_libraries(${APP}
      Render_Module
    )
    install(TARGETS ${APP} EXPORT ${PROJECT_NAME}
      RUNTIME DESTINATION bin
    )
    add_test(${APP} ${APP})
  endforeach (BASE)

endif(BUILD_TESTS)
