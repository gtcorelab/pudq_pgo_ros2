cmake_minimum_required(VERSION 3.8)
project(pudq_graph_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Enable automatic parallel computation
add_compile_options(-flto=auto)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pudq_lib REQUIRED)
find_package(pudq_pgo_lib REQUIRED)
find_package(pudq_msgs REQUIRED)

find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(OsqpEigen REQUIRED)

add_executable(
    ${PROJECT_NAME}_node
    src/${PROJECT_NAME}_node.cpp
    src/PUDQGraphManager.cpp
    include/PUDQGraphManager.hpp
)

target_link_libraries(
    ${PROJECT_NAME}_node PUBLIC
    rclcpp::rclcpp
    ${std_msgs_TARGETS}
    ${std_srvs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
    ${tf2_TARGETS}
    ${tf2_ros_TARGETS}
    ${tf2_geometry_msgs_TARGETS}
    ${pudq_lib_TARGETS}
    ${pudq_pgo_lib_TARGETS}
    ${pudq_msgs_TARGETS}
    Eigen3::Eigen
    osqp::osqp
    OsqpEigen::OsqpEigen
)

set_target_properties(${PROJECT_NAME}_node PROPERTIES COMPILE_FLAGS ${OpenMP_CXX_FLAGS})
set_target_properties(${PROJECT_NAME}_node PROPERTIES LINK_FLAGS "-fopenmp")

target_include_directories(
    ${PROJECT_NAME}_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIR}
)

install(
    TARGETS ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/
)

install(
    DIRECTORY g2o
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()
