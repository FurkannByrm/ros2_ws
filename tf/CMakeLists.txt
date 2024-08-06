cmake_minimum_required(VERSION 3.8)
project(tf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_typesupport_fastrtps_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(turtlesim REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

include_directories(include)

add_executable(tf_kinematics src/tf_kinematic.cpp)
ament_target_dependencies(tf_kinematics rclcpp custom_interfaces geometry_msgs tf2 tf2_ros)

ament_target_dependencies(tf_kinematics
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")

  install(
    DIRECTORY include
    DESTINATION include
  )
  install(TARGETS
  tf_kinematics
  DESTINATION lib/${PROJECT_NAME}
)
add_executable(static_tf2_broadcaster src/static_broadcaster.cpp)
ament_target_dependencies(
   static_tf2_broadcaster
   geometry_msgs
   rclcpp
   tf2
   tf2_ros
)
install(TARGETS
   static_tf2_broadcaster
   DESTINATION lib/${PROJECT_NAME})
ament_package()
