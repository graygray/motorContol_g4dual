# Copyright 2026 Gray Lin
# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.8)
project(motor_control_g4dual)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Threads REQUIRED)

add_library(differential_drive_kinematics STATIC
  src/differential_drive_kinematics.cpp
)
target_compile_features(differential_drive_kinematics PUBLIC cxx_std_17)
target_include_directories(differential_drive_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

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
  differential_drive_kinematics
  motor_can_protocol
  socket_can_transport
)
ament_target_dependencies(motor_control_node
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2_ros
)

install(TARGETS
  differential_drive_kinematics
  motor_can_protocol
  socket_can_transport
  motor_control_node
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
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_differential_drive_kinematics
    test/test_differential_drive_kinematics.cpp
  )
  target_link_libraries(test_differential_drive_kinematics
    differential_drive_kinematics
  )

  ament_add_gtest(test_motor_can_protocol
    test/test_motor_can_protocol.cpp
  )
  target_link_libraries(test_motor_can_protocol motor_can_protocol)

  if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    ament_add_gtest(test_socket_can_transport
      test/test_socket_can_transport.cpp
      TIMEOUT 10
    )
    target_link_libraries(test_socket_can_transport
      motor_can_protocol
      socket_can_transport
    )
  endif()
endif()

ament_package()
