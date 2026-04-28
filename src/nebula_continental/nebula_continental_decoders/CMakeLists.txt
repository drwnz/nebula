cmake_minimum_required(VERSION 3.20)
project(nebula_continental_decoders)

find_package(autoware_cmake REQUIRED)
find_package(Boost REQUIRED)
find_package(nebula_core_common REQUIRED)
find_package(nebula_core_decoders REQUIRED)
find_package(nebula_continental_common REQUIRED)
find_package(continental_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(nebula_msgs REQUIRED)
find_package(rclcpp REQUIRED)
autoware_package()

# Continental Decoder
add_library(
  nebula_continental_decoders SHARED
  src/decoders/continental_ars548_decoder.cpp
  src/decoders/continental_srr520_decoder.cpp)

target_include_directories(
  nebula_continental_decoders
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

target_link_libraries(
  nebula_continental_decoders
  PUBLIC nebula_core_common::nebula_core_common
         nebula_continental_common::nebula_continental_common rclcpp::rclcpp
         Boost::headers)

ament_target_dependencies(nebula_continental_decoders PUBLIC continental_msgs
                          diagnostic_msgs nebula_msgs)

# Continental Plugin Library
add_library(nebula_continental_decoders_plugin SHARED src/continental_plugin.cpp src/continental_decoder_agnostic.cpp)
target_include_directories(nebula_continental_decoders_plugin PRIVATE include)
target_link_libraries(nebula_continental_decoders_plugin PUBLIC nebula_continental_decoders)
ament_target_dependencies(nebula_continental_decoders_plugin PUBLIC
  nebula_core_common
  nebula_core_decoders
  nebula_continental_common
)

install(TARGETS nebula_continental_decoders nebula_continental_decoders_plugin EXPORT export_nebula_continental_decoders)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_nebula_continental_decoders)

ament_export_dependencies(Boost continental_msgs diagnostic_msgs nebula_core_common
                          nebula_continental_common nebula_msgs rclcpp)

ament_package()
