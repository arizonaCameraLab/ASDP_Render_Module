# Define the source files for the application
set(SOURCES
    main.cpp
    mainwindow.cpp
)

set(HEADERS
    mainwindow.h
)

set(UI_FILES
    mainwindow.ui
)

# Enable automatic MOC generation
set(CMAKE_AUTOMOC ON)

# Enable automatic UIC generation
set(CMAKE_AUTOUIC ON)

# Enable automatic RCC generation
set(CMAKE_AUTORCC ON)

# Use the Qt5 wrap UI command to process the .ui files
qt5_wrap_ui(UI_HEADERS ${UI_FILES})

# Add the executable target
add_executable(Storage_Manager ${SOURCES} ${HEADERS} ${UI_HEADERS})
get_property(isMultiConfig GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
if(isMultiConfig)
  # Multi-config (e.g., Visual Studio, Xcode)
  target_compile_definitions(Storage_Manager PRIVATE BUILD_TYPE="$<CONFIG>")
else()
  # Single-config (e.g., Makefiles, Ninja)
  if(CMAKE_BUILD_TYPE AND NOT CMAKE_BUILD_TYPE STREQUAL "")
    target_compile_definitions(Storage_Manager PRIVATE BUILD_TYPE="${CMAKE_BUILD_TYPE}")
  else()
    target_compile_definitions(Storage_Manager PRIVATE BUILD_TYPE="Default")
  endif()
endif()

# Link the Qt5 libraries to the target
target_link_libraries(Storage_Manager PRIVATE Qt5::Core Qt5::Widgets Qt5::Gui Render_Module ASDP_Core_API::ASDP_Core_API)
target_include_directories(Storage_Manager PRIVATE
  "${CMAKE_CURRENT_BINARY_DIR}"
  "${CMAKE_CURRENT_SOURCE_DIR}"
)

# Post-build step to run windeployqt
if(WIN32)
  add_custom_command(TARGET Storage_Manager POST_BUILD
      COMMAND ${Qt5_DIR}/../../../bin/windeployqt.exe $<TARGET_FILE:Storage_Manager>
      COMMENT "Running windeployqt to deploy Qt DLLs"
  )

  # Assuming GLEW::GLEW is the imported target from find_package(GLEW REQUIRED)
  # Use that to find all of the glew32.dll files in neighboring directories and then pick the one
  # that has "x64" in the path.
  get_target_property(GLEW_INCLUDE_DIRS GLEW::GLEW INTERFACE_INCLUDE_DIRECTORIES)
  file(GLOB_RECURSE GLEW_DLL "${GLEW_INCLUDE_DIRS}/../*glew32.dll")
  find_element_with_substring(GLEW_DLL2 "${GLEW_DLL}" "x64")

  if(GLEW_DLL2)
    # Copy the GLEW DLL to the build directory after building the executable
    add_custom_command(TARGET Storage_Manager POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different ${GLEW_DLL2} $<TARGET_FILE_DIR:Storage_Manager>
      COMMENT "Copying ${GLEW_DLL} to build directory"
    )
  else()
    message(WARNING "GLEW DLL not found. Make sure glew32.dll is in the same path as your GLEW include directory.")
  endif()

endif()

# Install the executable
install(TARGETS Storage_Manager
    RUNTIME DESTINATION bin
)

# Custom installation step to run windeployqt
if(WIN32)
    install(FILES
        $<TARGET_FILE_DIR:Storage_Manager>/D3Dcompiler_47.dll
        $<TARGET_FILE_DIR:Storage_Manager>/libEGL.dll
        $<TARGET_FILE_DIR:Storage_Manager>/libGLESv2.dll
        $<TARGET_FILE_DIR:Storage_Manager>/opengl32sw.dll
        $<TARGET_FILE_DIR:Storage_Manager>/Qt5Core.dll
        $<TARGET_FILE_DIR:Storage_Manager>/Qt5Gui.dll
        $<TARGET_FILE_DIR:Storage_Manager>/Qt5Svg.dll
        $<TARGET_FILE_DIR:Storage_Manager>/Qt5Widgets.dll
        ${CMAKE_CURRENT_SOURCE_DIR}/README.txt
      DESTINATION ${CMAKE_INSTALL_BINDIR})
    install(FILES
        $<TARGET_FILE_DIR:Storage_Manager>/platforms/qwindows.dll
      DESTINATION "${CMAKE_INSTALL_BINDIR}/platforms")
endif()