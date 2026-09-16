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
                description="Legacy alias for the initial info setting",
            ),
            DeclareLaunchArgument(
                "info",
                default_value=LaunchConfiguration("log"),
                description="Enable informational events (runtime parameter: info)",
            ),
            DeclareLaunchArgument(
                "rpm_resolution",
                default_value="1.0",
                choices=["1", "1.0", "0.1"],
                description="RPM per CAN command unit",
            ),
            DeclareLaunchArgument(
                "feedback_rpm_resolution",
                default_value="0.1",
                choices=["1", "1.0", "0.1"],
                description="RPM per CAN speed-feedback unit",
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
                    {"feedback_rpm_resolution": ParameterValue(
                        LaunchConfiguration("feedback_rpm_resolution"), value_type=float
                    )},
                ],
                arguments=["--info", LaunchConfiguration("info")],
            )
        ]
    )
