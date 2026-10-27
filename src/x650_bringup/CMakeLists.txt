cmake_minimum_required(VERSION 3.16)
project(x650_bringup)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
# TODO: find_package() for dependencies and add targets when implementation starts.

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
