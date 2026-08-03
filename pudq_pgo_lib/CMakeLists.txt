cmake_minimum_required(VERSION 3.8)
project(pudq_pgo_lib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Enable automatic parallel computation
add_compile_options(-flto=auto)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pudq_lib REQUIRED)

find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)

add_library(
    ${PROJECT_NAME}
    src/pudq_pgo_lib.cpp
    src/PUDQGraph.cpp
    include/pudq_pgo_lib/pudq_pgo_lib.hpp
    include/pudq_pgo_lib/PUDQGraph.hpp
)

target_link_libraries(
    ${PROJECT_NAME} PUBLIC
    rclcpp::rclcpp
    ${geometry_msgs_TARGETS}
    ${pudq_lib_TARGETS}
    Eigen3::Eigen
    osqp::osqp
    OsqpEigen::OsqpEigen
)

# Include OpenMP support for parallelization
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS ${OpenMP_CXX_FLAGS})
set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "-fopenmp")

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIR}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "pudq_pgo_lib_BUILDING_LIBRARY")

install(
    DIRECTORY include/
    DESTINATION include
)

install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

ament_export_include_directories(
    include
)
ament_export_libraries(
    ${PROJECT_NAME}
)
ament_export_targets(
    export_${PROJECT_NAME}
)

ament_package()
