cmake_minimum_required(VERSION 3.14)
project(nebula_core_hw_interfaces)

if(DEFINED ENV{AMENT_PREFIX_PATH})
    find_package(autoware_cmake REQUIRED)
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(PCAP REQUIRED libpcap)
find_package(nebula_core_common REQUIRED)

add_library(nebula_core_hw_interfaces SHARED 
  src/pcap_packet_source.cpp
  src/udp_packet_source.cpp
  src/can_packet_source.cpp
)

target_include_directories(
  nebula_core_hw_interfaces
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>
  PRIVATE ${PCAP_INCLUDE_DIRS})

target_link_libraries(nebula_core_hw_interfaces
                      PUBLIC nebula_core_common
                      PRIVATE ${PCAP_LIBRARIES})

install(TARGETS nebula_core_hw_interfaces
        EXPORT ${PROJECT_NAME}Targets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(DEFINED ENV{AMENT_PREFIX_PATH})
    ament_export_include_directories(include)
    ament_export_targets(export_${PROJECT_NAME})
    ament_export_dependencies(nebula_core_common)
    autoware_package()
    ament_package()
endif()
