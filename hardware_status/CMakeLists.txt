cmake_minimum_required(VERSION 3.8)
project(hardware_status)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include)
add_executable(bms_status_node src/battery_status_publisher.cpp)

ament_target_dependencies(bms_status_node
  rclcpp
  std_msgs
  custom_interfaces
)

install(TARGETS
  bms_status_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
