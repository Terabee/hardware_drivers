# Download and unpack googletest at configure time
configure_file(CMakeLists.txt.in ${CMAKE_BINARY_DIR}/googletest-download/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
    RESULT_VARIABLE result
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest-download )
if(result)
    message(FATAL_ERROR "CMake step for googletest failed: ${result}")
endif()
execute_process(COMMAND ${CMAKE_COMMAND} --build .
    RESULT_VARIABLE result
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest-download )
if(result)
    message(FATAL_ERROR "Build step for googletest failed: ${result}")
endif()

# Prevent overriding the parent project's compiler/linker
# settings on Windows - not sure if actually needed ...
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
# Add googletest directly to our build. This defines
# the gtest and gtest_main targets.

add_subdirectory(${CMAKE_BINARY_DIR}/googletest-src
  ${CMAKE_BINARY_DIR}/googletest-build
  EXCLUDE_FROM_ALL
)

# I have to add this include dir manually because of the bug in gmock build system
include_directories(SYSTEM ${CMAKE_BINARY_DIR}/googletest-src/googletest/include)

enable_testing()
add_executable(test_serial
  test_serial.cpp
)
target_link_libraries(test_serial
  serial_communication
  gmock_main
)
add_test(SerialTest ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/test_serial)

add_executable(test_follow_me_driver
  test_follow_me_driver.cpp
)
target_link_libraries(test_follow_me_driver
  serial_communication
  follow_me_driver
  gmock_main
)
add_test(FollowMeDriverTest ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/test_follow_me_driver)
