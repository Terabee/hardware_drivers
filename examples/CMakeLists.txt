cmake_minimum_required(VERSION 3.5.1)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -O -g -pthread")

project(hardware_drivers_examples)

add_executable(serial_example
  serial_example.cpp
)

target_link_libraries(serial_example
  serial_communication
)

add_executable(follow_me_driver_example
  follow_me_driver_example.cpp
)

target_link_libraries(follow_me_driver_example
  follow_me_driver
  serial_communication
)
