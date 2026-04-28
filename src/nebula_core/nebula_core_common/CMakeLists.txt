cmake_minimum_required(VERSION 3.20)
project(nebula_core_common)

if(DEFINED ENV{AMENT_PREFIX_PATH})
    find_package(autoware_cmake REQUIRED)
endif()

find_package(nlohmann_json REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)

add_library(nebula_core_common SHARED src/nebula_common.cpp)

target_include_directories(
  nebula_core_common
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

target_link_libraries(nebula_core_common PUBLIC
  nlohmann_json::nlohmann_json
  Boost::headers)

# Standard CMake install
install(TARGETS nebula_core_common 
        EXPORT ${PROJECT_NAME}Targets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include)

# ament-specific exports if building with ament
if(DEFINED ENV{AMENT_PREFIX_PATH})
    ament_export_include_directories(include)
    ament_export_targets(export_${PROJECT_NAME})
    ament_export_dependencies(Boost nlohmann_json)
    autoware_package()
    ament_package()
endif()
