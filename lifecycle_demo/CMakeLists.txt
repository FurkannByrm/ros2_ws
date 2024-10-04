cmake_minimum_required(VERSION 3.8)
project(lifecycle_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_executable(listener src/listener.cpp)
ament_target_dependencies(listener
lifecycle_msgs
std_msgs
rclcpp_lifecycle)

add_executable(talker src/lifecycle_talker.cpp)
ament_target_dependencies(talker
  lifecycle_msgs
  rclcpp_lifecycle
  std_msgs
)

include_directories(
  include
)


install(TARGETS
  listener
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)


ament_package()
