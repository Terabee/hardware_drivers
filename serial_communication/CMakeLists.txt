cmake_minimum_required(VERSION 3.5.1)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -O -g -pthread")

project(serial_communication)

set(LIB_NAME serial_communication)

add_library(${LIB_NAME} SHARED
  $<$<BOOL:${WIN32}>:src/serial_windows.cpp>
  $<$<BOOL:${UNIX}>:src/serial.cpp>
)

target_include_directories(${LIB_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
  $<INSTALL_INTERFACE:include/>
)

set_target_properties(${LIB_NAME} PROPERTIES
  VERSION ${PACKAGE_VERSION}
)

install(DIRECTORY include/
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
  COMPONENT headers
)

install(TARGETS ${LIB_NAME}
  EXPORT ${LIB_NAME}
  DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  COMPONENT libraries
)

install(EXPORT ${LIB_NAME}
  DESTINATION "${export_dest_dir}"
  NAMESPACE hardware_drivers::
  FILE ${config_targets_file_serial_communication}
  COMPONENT configs
)
