# Copyright 2026 Gray Lin
# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.8)
project(motor_control_g4dual)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(motor_control_node
  src/motor_control_node.cpp
  src/main.cpp
)
target_compile_features(motor_control_node PUBLIC cxx_std_17)
target_include_directories(motor_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(motor_control_node
  geometry_msgs
  rclcpp
  std_msgs
)

install(TARGETS motor_control_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
