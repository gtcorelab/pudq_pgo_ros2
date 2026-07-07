cmake_minimum_required(VERSION 3.8)
project(pudq_keyframe_generator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pudq_lib REQUIRED)
find_package(pudq_msgs REQUIRED)

add_executable(
    ${PROJECT_NAME}_node
    src/${PROJECT_NAME}_node.cpp
    src/PUDQKeyframeGenerator.cpp
    include/PUDQKeyframeGenerator.hpp
)

ament_target_dependencies(
    ${PROJECT_NAME}_node
    rclcpp
    geometry_msgs
    std_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    pudq_lib
    pudq_msgs
)

target_include_directories(
    ${PROJECT_NAME}_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(
    TARGETS ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
