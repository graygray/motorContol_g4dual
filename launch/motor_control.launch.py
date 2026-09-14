# Copyright 2026 Gray Lin
# SPDX-License-Identifier: MIT

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
            DeclareLaunchArgument(
                "rpm_resolution",
                default_value="0.1",
                choices=["1", "1.0", "0.1"],
                description="RPM per CAN speed unit; must match controller firmware",
            ),
            Node(
                package="motor_control_g4dual",
                executable="motor_control_node",
                name="motor_control",
                output="screen",
                parameters=[
                    str(parameters),
                    {"rpm_resolution": ParameterValue(
                        LaunchConfiguration("rpm_resolution"), value_type=float
                    )},
                ],
                arguments=["--log", LaunchConfiguration("log")],
            )
        ]
    )
