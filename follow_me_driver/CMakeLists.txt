cmake_minimum_required(VERSION 3.5.1)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -O -g -pthread")

project(follow_me_driver)

set(LIB_NAME follow_me_driver)

add_library(${LIB_NAME} SHARED
  src/follow_me_driver.cpp
)

target_include_directories(${LIB_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
  $<INSTALL_INTERFACE:include/>
)

target_link_libraries(${LIB_NAME}
  serial_communication
)

install(DIRECTORY include/ DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")
install(TARGETS ${LIB_NAME} EXPORT ${LIB_NAME} DESTINATION "${CMAKE_INSTALL_LIBDIR}")
install(EXPORT ${LIB_NAME}
  DESTINATION "${export_dest_dir}"
  NAMESPACE hardware_drivers::
  FILE ${config_targets_file_follow_me_driver}
)
