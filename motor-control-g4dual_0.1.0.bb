# Copyright 2026 Gray Lin
# SPDX-License-Identifier: MIT

SUMMARY = "ROS 2 motor-control node for the G4 dual-motor controller"
DESCRIPTION = "Upper-layer ROS 2 motor control, SocketCAN transport, feedback, diagnostics, and odometry for the G4 dual-motor controller."
HOMEPAGE = "https://github.com/graygray/motorContol_g4dual"
AUTHOR = "Gray Lin <gray.lin@primax.com.tw>"
SECTION = "devel"

LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://LICENSE;md5=b3c44bdeb05b583b5e06d7e785df11a2"

ROS_CN = "motor_control_g4dual"
ROS_BPN = "motor_control_g4dual"

ROS_BUILD_DEPENDS = " \
    diagnostic-msgs \
    geometry-msgs \
    nav-msgs \
    rclcpp \
    sensor-msgs \
    std-msgs \
    std-srvs \
    tf2-ros \
"

ROS_BUILDTOOL_DEPENDS = " \
    ament-cmake-native \
"

ROS_EXPORT_DEPENDS = " \
    diagnostic-msgs \
    geometry-msgs \
    nav-msgs \
    rclcpp \
    sensor-msgs \
    std-msgs \
    std-srvs \
    tf2-ros \
"

ROS_BUILDTOOL_EXPORT_DEPENDS = ""

ROS_EXEC_DEPENDS = " \
    ament-index-python \
    diagnostic-msgs \
    geometry-msgs \
    launch \
    launch-ros \
    nav-msgs \
    rclcpp \
    sensor-msgs \
    std-msgs \
    std-srvs \
    tf2-ros \
"

# Tests are intentionally not built as part of the target image recipe.
ROS_TEST_DEPENDS = ""

DEPENDS = "${ROS_BUILD_DEPENDS} ${ROS_BUILDTOOL_DEPENDS}"
DEPENDS += "${ROS_EXPORT_DEPENDS} ${ROS_BUILDTOOL_EXPORT_DEPENDS}"
RDEPENDS:${PN} += "${ROS_EXEC_DEPENDS}"

ROS_BRANCH ?= "branch=main"
SRC_URI = "git://git@github.com/graygray/motorContol_g4dual.git;${ROS_BRANCH};protocol=ssh"
SRCREV = "${AUTOREV}"
S = "${WORKDIR}/git"

ROS_BUILD_TYPE = "ament_cmake"

inherit ros_${ROS_BUILD_TYPE}
