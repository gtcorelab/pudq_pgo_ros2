cmake_minimum_required(VERSION 3.8)
project(pudq_lib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(pudq_lib src/pudq_lib.cpp include/pudq_lib/pudq_lib.hpp)

ament_target_dependencies(
    pudq_lib
    rclcpp
    std_msgs
    geometry_msgs
)

target_compile_features(pudq_lib PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(pudq_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(pudq_lib PRIVATE "PUDQ_LIB_BUILDING_LIBRARY")

install(
    DIRECTORY include/
    DESTINATION include
)

install(
    TARGETS pudq_lib
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

ament_export_include_directories(
    include
)
ament_export_libraries(
    pudq_lib
)
ament_export_targets(
    export_${PROJECT_NAME}
)

ament_package()
