cmake_minimum_required(VERSION 3.16)
project(x650_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/calibration_tf_publisher scripts/calibration_hash DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY urdf meshes config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_calibration test/test_calibration.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_package()
