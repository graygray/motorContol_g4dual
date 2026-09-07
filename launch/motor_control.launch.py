# Copyright 2026 Gray Lin
# SPDX-License-Identifier: MIT

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("motor_control_g4dual"))
    parameters = package_share / "config" / "motor_control.yaml"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "log",
                default_value="false",
                description="Enable motor-control code-flow log messages",
            ),
            Node(
                package="motor_control_g4dual",
                executable="motor_control_node",
                name="motor_control",
                output="screen",
                parameters=[str(parameters)],
                arguments=["--log", LaunchConfiguration("log")],
            )
        ]
    )
