cmake_minimum_required(VERSION 3.20)
project(nebula_core_runtime)

find_package(autoware_cmake REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(nebula_core_common REQUIRED)
find_package(nebula_core_decoders REQUIRED)
autoware_package()

add_library(nebula_core_runtime SHARED
  src/sensor_registry.cpp
)

target_include_directories(
  nebula_core_runtime
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

target_link_libraries(nebula_core_runtime
  nlohmann_json::nlohmann_json
  Boost::filesystem
  ${CMAKE_DL_LIBS}
)

ament_target_dependencies(nebula_core_runtime
  nebula_core_common
  nebula_core_decoders
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_sensor_registry test/test_sensor_registry.cpp)
  target_link_libraries(test_sensor_registry nebula_core_runtime)
  ament_target_dependencies(test_sensor_registry
    nebula_core_common
    nebula_core_decoders
  )
endif()

install(TARGETS nebula_core_runtime EXPORT export_nebula_core_runtime)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_nebula_core_runtime)

ament_export_dependencies(Boost nlohmann_json nebula_core_common nebula_core_decoders)

ament_package()
