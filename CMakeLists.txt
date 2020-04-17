cmake_minimum_required(VERSION 3.5.1)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -O -g -pthread")
set(PACKAGE_VERSION 1.0.0)
project(hardware_drivers VERSION ${PACKAGE_VERSION})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/bin)
option(BUILD_TESTS "Builds tests" ON)
option(BUILD_EXAMPLES "Build short example binaries" ON)

include(GNUInstallDirs)
set(project_config_in "${CMAKE_CURRENT_LIST_DIR}/cmake/hardware_driversConfig.cmake.in")
set(project_config_out "${CMAKE_CURRENT_BINARY_DIR}/hardware_driversConfig.cmake")
set(config_targets_file_serial_communication "serial_communication.cmake")
set(config_targets_file_follow_me_driver "follow_me_driver.cmake")
set(version_config_file "${CMAKE_CURRENT_BINARY_DIR}/hardware_driversConfigVersion.cmake")
set(export_dest_dir "${CMAKE_INSTALL_DATAROOTDIR}/hardware_drivers/cmake")

add_subdirectory(serial_communication)
add_subdirectory(follow_me_driver)
if(BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

if(BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

include(CMakePackageConfigHelpers)
configure_file("${project_config_in}" "${project_config_out}" @ONLY)
write_basic_package_version_file("${version_config_file}" COMPATIBILITY SameMajorVersion)
install(FILES
  "${project_config_out}"
  "${version_config_file}" DESTINATION "${export_dest_dir}"
)
