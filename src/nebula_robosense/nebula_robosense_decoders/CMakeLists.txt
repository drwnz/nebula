cmake_minimum_required(VERSION 3.20)
project(nebula_robosense_decoders)

find_package(autoware_cmake REQUIRED)
find_package(Boost REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(PCAP REQUIRED libpcap)
autoware_package()

# Robosense Decoder
add_library(nebula_robosense_decoders SHARED src/robosense_driver.cpp)

target_include_directories(
  nebula_robosense_decoders
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

target_link_libraries(
  nebula_robosense_decoders
  PUBLIC nebula_core_common::nebula_core_common
         nebula_robosense_common::nebula_robosense_common
         nebula_core_decoders::nebula_core_decoders rclcpp::rclcpp
         Boost::headers)

add_executable(robosense_e1_pcap_dump src/robosense_e1_pcap_dump.cpp)
target_include_directories(
  robosense_e1_pcap_dump
  PRIVATE include ${PCAP_INCLUDE_DIRS})
target_link_libraries(
  robosense_e1_pcap_dump
  PRIVATE nebula_robosense_decoders ${PCAP_LIBRARIES})
target_compile_options(robosense_e1_pcap_dump PRIVATE ${PCAP_CFLAGS_OTHER})

# Robosense Info Decoder
add_library(nebula_robosense_decoders_info SHARED src/robosense_info_driver.cpp)

target_include_directories(
  nebula_robosense_decoders_info
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

target_link_libraries(
  nebula_robosense_decoders_info
  PUBLIC nebula_core_common::nebula_core_common
         nebula_robosense_common::nebula_robosense_common
         nebula_core_decoders::nebula_core_decoders rclcpp::rclcpp
         Boost::headers)

install(TARGETS nebula_robosense_decoders
        EXPORT export_nebula_robosense_decoders)
install(TARGETS nebula_robosense_decoders_info
        EXPORT export_nebula_robosense_decoders_info)
install(TARGETS robosense_e1_pcap_dump DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/plot_e1_csv.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_robosense_e1_decoder tests/test_robosense_e1_decoder.cpp)
  target_include_directories(test_robosense_e1_decoder PRIVATE include)
  target_link_libraries(test_robosense_e1_decoder nebula_robosense_decoders)
endif()


ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_nebula_robosense_decoders)
ament_export_targets(export_nebula_robosense_decoders_info)

ament_export_dependencies(Boost nebula_core_common nebula_robosense_common
                          nebula_core_decoders rclcpp)

ament_package()
