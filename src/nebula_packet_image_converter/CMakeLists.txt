cmake_minimum_required(VERSION 3.8)
project(nebula_packet_image_converter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nebula_msgs REQUIRED)
find_package(nebula_core_ros REQUIRED)

include_directories(include)

add_library(nebula_packets_to_image_component SHARED
  src/nebula_packets_to_image.cpp)
target_include_directories(nebula_packets_to_image_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(nebula_packets_to_image_component
  rclcpp
  rclcpp_components
  sensor_msgs
  nebula_msgs
  nebula_core_ros)
rclcpp_components_register_node(nebula_packets_to_image_component PLUGIN "nebula::packet_image_converter::NebulaPacketsToImage" EXECUTABLE nebula_packets_to_image)

add_library(image_to_nebula_packets_component SHARED
  src/image_to_nebula_packets.cpp)
target_include_directories(image_to_nebula_packets_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(image_to_nebula_packets_component
  rclcpp
  rclcpp_components
  sensor_msgs
  nebula_msgs
  nebula_core_ros)
rclcpp_components_register_node(image_to_nebula_packets_component PLUGIN "nebula::packet_image_converter::ImageToNebulaPackets" EXECUTABLE image_to_nebula_packets)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
