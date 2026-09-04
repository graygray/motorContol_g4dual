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
find_package(Threads REQUIRED)

add_library(motor_can_protocol STATIC
  src/motor_can_protocol.cpp
)
target_compile_features(motor_can_protocol PUBLIC cxx_std_17)
target_include_directories(motor_can_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(socket_can_transport STATIC
  src/socket_can_transport.cpp
)
target_compile_features(socket_can_transport PUBLIC cxx_std_17)
target_include_directories(socket_can_transport PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(socket_can_transport PUBLIC Threads::Threads)

add_executable(motor_control_node
  src/motor_control_node.cpp
  src/main.cpp
)
target_compile_features(motor_control_node PUBLIC cxx_std_17)
target_include_directories(motor_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(motor_control_node
  motor_can_protocol
  socket_can_transport
)
ament_target_dependencies(motor_control_node
  geometry_msgs
  rclcpp
  std_msgs
)

install(TARGETS motor_can_protocol socket_can_transport motor_control_node
  EXPORT export_motor_can_protocol
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(export_motor_can_protocol HAS_LIBRARY_TARGET)
ament_export_dependencies(Threads)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
