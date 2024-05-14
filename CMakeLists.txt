cmake_minimum_required(VERSION 3.16.0)
project(ASDP_Render_Module LANGUAGES C CXX CUDA)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
list(APPEND CMAKE_PREFIX_PATH C:/Packages/GLEW/glew-2.2.0)

#-----------------------------------------------------------------------------
if(WIN32)
    add_definitions(-DNOMINMAX)
else(WIN32)
    add_definitions(-DOS_LINUX_XCB_GLX)
endif(WIN32)

#-----------------------------------------------------------------------------
# Local CMake Modules
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

#-----------------------------------------------------------------------------
# Enable sorting projects within the solution on Visual Studio, to group
# Test and Example files together.
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

#-----------------------------------------------------------------------------
# Checking for optional and required packages

find_package(Threads REQUIRED)
find_package(ASDP_Core_API REQUIRED)
find_package(OpenGL REQUIRED COMPONENTS OpenGL)
find_package(Doxygen)
find_package(OpenMP)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

#-----------------------------------------------------------------------------
# Build options.
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
  #Display.h
  ImageQueue.h
)
set (Render_Module_SOURCES
	Composite.cpp
  #Display.cpp
  ImageQueue.cpp
  gfxwrapper/gfxwrapper_opengl.c
)
add_library(Render_Module STATIC ${Render_Module_SOURCES} ${Render_Module_HEADERS})
target_include_directories(Render_Module PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>
  "${CMAKE_CURRENT_BINARY_DIR}/Install/include"
)
target_include_directories(Render_Module PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/gfxwrapper"
  "${CMAKE_CURRENT_SOURCE_DIR}/dependencies/glm"
)
target_link_libraries(Render_Module PUBLIC
  ASDP_Core_API::ASDP_Core_API OpenGL::GL
)

#-----------------------------------------------------------------------------
# Build utilities if we've been asked to.

if(BUILD_UTILITIES)

  set (CPP_UTILS
  )
  foreach (BASE ${CPP_UTILS})
    set (APP ${BASE})
    add_executable (${APP} util/${BASE}.cpp)
    set_target_properties(${APP} PROPERTIES FOLDER util/C++)
    target_link_libraries(${APP} PRIVATE
	    ASDP_Core_API::ASDP_Core_API
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
    set_property(TARGET ${APP} PROPERTY CUDA_ARCHITECTURES 75 80)
    #set_property(TARGET ${APP} PROPERTY CUDA_ARCHITECTURES 70)
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
    SharedContext_Test
    Composite_Test
    CompositeCameras_Test
  )
  foreach (BASE ${CPP_TESTS})
    set (APP ${BASE})
    add_executable (${APP} tests_cpp/${BASE}.cpp)
    set_target_properties(${APP} PROPERTIES FOLDER tests/C++)
    target_include_directories(${APP} PRIVATE
      "${CMAKE_CURRENT_SOURCE_DIR}"
      "${CMAKE_CURRENT_SOURCE_DIR}/gfxwrapper"
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
