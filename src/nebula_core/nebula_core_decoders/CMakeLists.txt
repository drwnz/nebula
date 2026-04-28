cmake_minimum_required(VERSION 3.20)
project(nebula_core_decoders)

if(DEFINED ENV{AMENT_PREFIX_PATH})
    find_package(autoware_cmake REQUIRED)
endif()

find_package(PNG REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED)
find_package(nebula_core_common REQUIRED)

add_library(nebula_core_decoders SHARED src/dummy.cpp)

target_include_directories(
  nebula_core_decoders
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
target_link_libraries(nebula_core_decoders PUBLIC PNG::PNG Eigen3::Eigen
                                                  Boost::headers
                                                  nebula_core_common)

install(TARGETS nebula_core_decoders
        EXPORT ${PROJECT_NAME}Targets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(DEFINED ENV{AMENT_PREFIX_PATH})
    ament_export_include_directories(include)
    ament_export_targets(export_${PROJECT_NAME})
    ament_export_dependencies(Boost nebula_core_common PNG Eigen3)
    autoware_package()
    ament_package()
endif()
