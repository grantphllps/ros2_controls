cmake_minimum_required(VERSION 3.8)
project(ros2_controls)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(pid src/pid.cpp)
ament_target_dependencies(pid rclcpp std_msgs geometry_msgs)

add_executable(rypt_xform src/rypt_xform.cpp)
ament_target_dependencies(rypt_xform rclcpp std_msgs geometry_msgs tf2 tf2_geometry_msgs)

install(TARGETS 
  rypt_xform
  pid
  DESTINATION lib/${PROJECT_NAME})
ament_package()
