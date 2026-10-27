cmake_minimum_required(VERSION 3.16)
project(x650_px4_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(x650_common REQUIRED)

# Header-only core: frame conversion + external-vision sample builder (no ROS runtime).
add_library(${PROJECT_NAME}_core INTERFACE)
target_include_directories(${PROJECT_NAME}_core INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(${PROJECT_NAME}_core INTERFACE Eigen3::Eigen x650_common::x650_common)

# uXRCE-DDS transport node (decision 0004).
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
add_library(px4_bridge_component SHARED src/px4_bridge_node.cpp)
target_link_libraries(px4_bridge_component ${PROJECT_NAME}_core rclcpp::rclcpp
  rclcpp_components::component ${nav_msgs_TARGETS} ${diagnostic_msgs_TARGETS} ${px4_msgs_TARGETS})
rclcpp_components_register_node(px4_bridge_component
  PLUGIN "x650_px4_bridge::Px4BridgeNode" EXECUTABLE px4_bridge_node)
install(TARGETS px4_bridge_component LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_core EXPORT export_${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(Eigen3 x650_common)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_frame_conversion test/test_frame_conversion.cpp)
  target_link_libraries(test_frame_conversion ${PROJECT_NAME}_core)
  ament_add_gtest(test_external_vision test/test_external_vision.cpp)
  target_link_libraries(test_external_vision ${PROJECT_NAME}_core)
endif()

ament_package()
