cmake_minimum_required(VERSION 3.16)
project(x650_camera)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(x650_msgs REQUIRED)
find_package(x650_common REQUIRED)

# USB webcam (V4L2) capture. The ZED X wrapper is added when its driver is available (decision 0001).
add_library(usb_camera_component SHARED src/usb_camera_node.cpp)
target_include_directories(usb_camera_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(usb_camera_component rclcpp::rclcpp rclcpp_components::component
  ${sensor_msgs_TARGETS} ${diagnostic_msgs_TARGETS} ${x650_msgs_TARGETS} x650_common::x650_common)
rclcpp_components_register_node(usb_camera_component
  PLUGIN "x650_camera::UsbCameraNode" EXECUTABLE usb_camera_node)
install(TARGETS usb_camera_component LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_v4l2_timing test/test_v4l2_timing.cpp)
  target_include_directories(test_v4l2_timing PRIVATE include)
endif()

ament_package()
