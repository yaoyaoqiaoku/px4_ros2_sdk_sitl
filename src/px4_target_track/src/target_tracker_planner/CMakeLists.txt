cmake_minimum_required(VERSION 3.8)
project(target_tracker_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(px4_ros2_cpp REQUIRED)

add_executable(target_tracker_planner_node
  src/target_tracker_planner_node.cpp
)

ament_target_dependencies(target_tracker_planner_node
  rclcpp
  geometry_msgs
  px4_msgs
  sensor_msgs
  px4_ros2_cpp
)

target_include_directories(target_tracker_planner_node
  PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS target_tracker_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch cfg
  DESTINATION share/${PROJECT_NAME}
)

ament_package()