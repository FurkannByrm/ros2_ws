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
find_package(twist_mux_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include)

# Create the executable for the bms_status_node
add_executable(bms_status_node src/battery_status_publisher.cpp)
ament_target_dependencies(bms_status_node 
  rclcpp
  std_msgs
  custom_interfaces
)

# Create the shared library for emg_stop_client
add_library(emg_stop_client SHARED src/emg_stop_client.cpp)
ament_target_dependencies(emg_stop_client
  rclcpp
  custom_interfaces
)

# Link the bms_status_node with the emg_stop_client library
target_link_libraries(bms_status_node
  emg_stop_client
)


add_executable(safety_stop src/safety_stop.cpp)
ament_target_dependencies(safety_stop rclcpp std_msgs sensor_msgs geometry_msgs twist_mux_msgs rclcpp_action visualization_msgs)



install(TARGETS
  bms_status_node
  emg_stop_client
  safety_stop
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
