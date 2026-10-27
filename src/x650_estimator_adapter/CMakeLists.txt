cmake_minimum_required(VERSION 3.16)
project(x650_estimator_adapter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(x650_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# ROS-independent core (OdometrySplitter).
add_library(${PROJECT_NAME}_core INTERFACE)
target_include_directories(${PROJECT_NAME}_core INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(${PROJECT_NAME}_core INTERFACE Eigen3::Eigen x650_common::x650_common)

add_library(estimator_adapter_component SHARED src/estimator_adapter_node.cpp)
target_link_libraries(estimator_adapter_component
  ${PROJECT_NAME}_core
  rclcpp::rclcpp
  rclcpp_components::component
  tf2_ros::tf2_ros
  ${nav_msgs_TARGETS} ${geometry_msgs_TARGETS} ${std_msgs_TARGETS} ${diagnostic_msgs_TARGETS})
rclcpp_components_register_node(estimator_adapter_component
  PLUGIN "x650_estimator_adapter::EstimatorAdapterNode"
  EXECUTABLE estimator_adapter_node)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_core EXPORT export_${PROJECT_NAME})
install(TARGETS estimator_adapter_component
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(Eigen3 x650_common)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_odometry_splitter test/test_odometry_splitter.cpp)
  target_link_libraries(test_odometry_splitter ${PROJECT_NAME}_core)
endif()

ament_package()
