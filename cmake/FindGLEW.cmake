# FindGLEW.cmake
#
# Drop‑in CMake module that detects static or shared GLEW automatically
# and exports a proper imported target: GLEW::GLEW
#
# Supports:
#   - libGLEW.a (static)
#   - libGLEW.so (shared)
#   - Custom GLEW install locations
#   - System GLEW packages
#
# Usage:
#   find_package(GLEW REQUIRED)
#   target_link_libraries(MyTarget PRIVATE GLEW::GLEW)

include(FindPackageHandleStandardArgs)

# Allow user override:
#   -DGLEW_ROOT=/path/to/glew
#   -DGLEW_USE_STATIC=ON
set(GLEW_ROOT "" CACHE PATH "Root directory of a custom GLEW installation")
set(GLEW_USE_STATIC OFF CACHE BOOL "Force static GLEW linking")

# Search paths
set(_GLEW_SEARCH_DIRS
    ${CMAKE_PREFIX_PATH}
    ${GLEW_ROOT}
    ${GLEW_ROOT}/lib
    ${GLEW_ROOT}/lib64
    /usr
    /usr/local
    /usr/lib
    /usr/lib64
    /usr/local/lib
    /usr/local/lib64
)

# Locate headers
find_path(GLEW_INCLUDE_DIR
    NAMES GL/glew.h
    PATH_SUFFIXES include
    PATHS ${_GLEW_SEARCH_DIRS}
)

# Locate static and shared libs
find_library(GLEW_STATIC_LIB
    NAMES GLEW glew libGLEW glew32s
    PATH_SUFFIXES lib lib64 lib/Release/x64
    PATHS ${_GLEW_SEARCH_DIRS}
    NO_DEFAULT_PATH
)

find_library(GLEW_SHARED_LIB
    NAMES GLEW glew libGLEW glew32
    PATH_SUFFIXES lib lib64 lib/Release/x64
    PATHS ${_GLEW_SEARCH_DIRS}
)

# Decide which library to use
set(GLEW_LIBRARY "")

if(GLEW_USE_STATIC AND GLEW_STATIC_LIB)
    set(GLEW_LIBRARY ${GLEW_STATIC_LIB})
elseif(GLEW_STATIC_LIB AND NOT GLEW_SHARED_LIB)
    set(GLEW_LIBRARY ${GLEW_STATIC_LIB})
elseif(GLEW_SHARED_LIB)
    set(GLEW_LIBRARY ${GLEW_SHARED_LIB})
endif()

# Validate
find_package_handle_standard_args(GLEW
    REQUIRED_VARS GLEW_LIBRARY GLEW_INCLUDE_DIR
)

# Create imported target
if(NOT TARGET GLEW::GLEW)
    add_library(GLEW::GLEW UNKNOWN IMPORTED)
    set_target_properties(GLEW::GLEW PROPERTIES
        IMPORTED_LOCATION "${GLEW_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${GLEW_INCLUDE_DIR}"
    )

    # If static, define GLEW_STATIC
    if(GLEW_LIBRARY MATCHES "\\.a$")
        target_compile_definitions(GLEW::GLEW INTERFACE GLEW_STATIC)
    endif()
endif()

