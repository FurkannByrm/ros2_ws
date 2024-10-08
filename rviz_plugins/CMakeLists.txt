cmake_minimum_required(VERSION 3.5)
project(points_for_loop)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include)

add_executable(points_for_loop src/points_for_loop.cpp)

ament_target_dependencies(points_for_loop rclcpp visualization_msgs)

install(TARGETS
  points_for_loop
  DESTINATION lib/${PROJECT_NAME})

ament_package()
