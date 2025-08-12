cmake_minimum_required(VERSION 3.8)
project(moveit2_samples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nlohmann_json REQUIRED)  
find_package(moveit_ros_planning_interface REQUIRED)

# Generate service interfaces first
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Position.srv"
  DEPENDENCIES geometry_msgs
)

include_directories(${PROJECT_SOURCE_DIR}/include)
add_executable(moveit_example src/moveit.cpp)
ament_target_dependencies(moveit_example
  rclcpp
  moveit_ros_planning_interface
)
add_executable(server src/position_server.cpp
                      src/position_parser.cpp)
ament_target_dependencies(server
  rclcpp) 

add_executable(client src/position_client.cpp
                      src/position_parser.cpp)
ament_target_dependencies(client
  rclcpp
  nlohmann_json)


# Link the generated interfaces to the executables
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(server "${cpp_typesupport_target}")
target_link_libraries(client "${cpp_typesupport_target}")

install(TARGETS
  server
  client
  DESTINATION lib/${PROJECT_NAME})

  install(TARGETS
  moveit_example
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
