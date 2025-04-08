
#-----------------------------------------------------------------------------
# VRPN serial-port library

set(Lib_SOURCES vrpn_Serial.C vrpn_Shared.C)
set(Lib_PUBLIC_HEADERS vrpn_Serial.h vrpn_Shared.h vrpn_Types.h)
ADD_LIBRARY (vrpn_Serial
	${Lib_SOURCES} ${Lib_PUBLIC_HEADERS}
)
set_property(TARGET vrpn_Serial PROPERTY PUBLIC_HEADER ${Lib_PUBLIC_HEADERS})
target_include_directories(vrpn_Serial PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>
  "${CMAKE_CURRENT_BINARY_DIR}/Install/include"
)

install(TARGETS vrpn_Serial
	ARCHIVE DESTINATION lib
	PUBLIC_HEADER DESTINATION include)

#-----------------------------------------------------------------------------
# Gimbal library

set (GIMBAL_Sources Gimbal.cpp)
set (GIMBAL_PUBLIC_Headers Gimbal.h)
add_library (Gimbal
  ${GIMBAL_Sources} ${GIMBAL_PUBLIC_Headers}
)
set_property(TARGET Gimbal PROPERTY PUBLIC_HEADER ${GIMBAL_PUBLIC_Headers})
target_include_directories(Gimbal PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>
  "${CMAKE_CURRENT_BINARY_DIR}/Install/include"
)
target_link_libraries(Gimbal PUBLIC vrpn_Serial)
