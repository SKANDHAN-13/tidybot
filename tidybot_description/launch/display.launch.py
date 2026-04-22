"""
TidyBot — display.launch.py

Launches:
  1. xacro → robot_description (manual joint sliders for visualisation) in the RViz2

Usage:
  ros2 launch tidybot_description display.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue



# ────────────────────────────────────── 
def generate_launch_description():

    pkg_share = get_package_share_directory("tidybot_description")

    default_model = os.path.join(pkg_share, "urdf", "tidybot.urdf.xacro")
    default_rviz  = os.path.join(pkg_share, "rviz",  "tidybot.rviz")

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=default_model,
        description="Absolute path to the robot URDF/xacro file",
    )
    rviz_config_arg = DeclareLaunchArgument(
        name="rvizconfig",
        default_value=default_rviz,
        description="Absolute path to the RViz config file",
    )
# ────────────────────────────────────── 
    # Robot_description from xacro
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]), value_type=str
    )


    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )
# ──────────────────────────────────────
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )

    return LaunchDescription(
        [
            model_arg, #Model arguments
            rviz_config_arg, #RViz configuration
            robot_state_publisher_node, #Robot state
            joint_state_publisher_gui_node, #Joint state
            rviz_node, #RViz initiation
        ]
    )
