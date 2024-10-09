cmake_minimum_required(VERSION 3.8)
project(serial_comm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# Derlenecek kaynak dosyalar
add_executable(uart src/uart.cpp)

# rclcpp'yi linkle
ament_target_dependencies(uart rclcpp)

# Çıktı dosyasını kur
install(TARGETS
  uart
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
