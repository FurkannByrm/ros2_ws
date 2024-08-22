cmake_minimum_required(VERSION 3.8)
project(cpp_project_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)


include_directories(${PROJECT_SOURCE_DIR}/include)
add_executable(io src/_IO.cpp)
add_executable(ref src/reference.cpp)
add_executable(const src/const.cpp)
add_executable(do src/do_while.cpp)
add_executable(error_catch src/error_catch.cpp)
add_executable(string_ src/_string.cpp)
add_executable(struct_function src/struct_function.cpp)
add_executable(vector src/vector.cpp)
# add_executable(template src/template.cpp)
add_executable(copy src/copy_constructor.cpp)
ament_target_dependencies(copy
rclcpp
)

ament_target_dependencies(io
rclcpp
)

ament_target_dependencies(ref
rclcpp
)
# ament_target_dependencies(template
# rclcpp
# )
ament_target_dependencies(string_
rclcpp
)

ament_target_dependencies(do
rclcpp
)

ament_target_dependencies(const
rclcpp
)
ament_target_dependencies(error_catch
rclcpp
)
ament_target_dependencies(struct_function
rclcpp
)
ament_target_dependencies(vector
rclcpp
)

install(TARGETS
  ref
  copy
  string_
  error_catch
  io
  const
  do
  struct_function
  vector
  # template
  copy
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
