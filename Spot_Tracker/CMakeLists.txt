
#-----------------------------------------------------------------------------
# Spot tracker library
set(STLib_SOURCES image_wrapper.cpp spot_math.cpp spot_tracker.cpp base_camera_server.cpp)
set(STLib_PUBLIC_HEADERS image_wrapper.h spot_math.h spot_tracker.h)
ADD_LIBRARY (spot_tracker_library
	${STLib_SOURCES} ${STLib_PUBLIC_HEADERS}
)
set_property(TARGET spot_tracker_library PROPERTY PUBLIC_HEADER ${STLib_PUBLIC_HEADERS})
target_include_directories(spot_tracker_library PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>
  "${CMAKE_CURRENT_BINARY_DIR}/Install/include"
)

install(TARGETS spot_tracker_library
	ARCHIVE DESTINATION lib
	PUBLIC_HEADER DESTINATION include)
