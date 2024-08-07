cmake_minimum_required(VERSION 3.8)
project(midi_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
add_library(
  midi_bot
  SHARED
  src/diffbot_system.cpp
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)

target_compile_features(midi_bot PUBLIC cxx_std_17)
target_include_directories(midi_bot PUBLIC 
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR} include>
$<INSTALL_INTERFACE:include/midi_bot>
)

ament_target_dependencies(
  midi_bot PUBLIC 
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
target_link_libraries(midi_bot PUBLIC serial)

install(
  DIRECTORY include/
  DESTINATION include/midi_bot
)

install(
  DIRECTORY description/launch description/ros2_control description/urdf description/rviz
  DESTINATION share/midi_bot
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/midi_bot
)
install(TARGETS midi_bot
  EXPORT export_midi_bot
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


# pluginlib_export_plugin_description_file(hardware_interface diffdrive_arduino.xml)
ament_package()
