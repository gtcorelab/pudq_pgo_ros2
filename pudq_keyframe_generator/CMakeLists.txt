cmake_minimum_required(VERSION 3.8)
project(pudq_keyframe_generator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Enable automatic parallel computation
add_compile_options(-flto=auto)

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

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
    Eigen3
    rclcpp
    geometry_msgs
    std_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    pudq_lib
    pudq_msgs
)

# Include OpenMP support for parallelization
set_target_properties(${PROJECT_NAME}_node PROPERTIES COMPILE_FLAGS ${OpenMP_CXX_FLAGS})
set_target_properties(${PROJECT_NAME}_node PROPERTIES LINK_FLAGS "-fopenmp")

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
