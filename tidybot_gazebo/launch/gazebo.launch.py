"""
TidyBot — gazebo.launch.py
===========================
Launches:
  Gazebo (with tidybot_home.world) and produces the required simulation


Usage:
  ros2 launch tidybot_gazebo gazebo.launch.py
  
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# ─────────────────────────────────────────────────────────
def generate_launch_description():

    pkg_gazebo   = get_package_share_directory("tidybot_gazebo")
    pkg_desc     = get_package_share_directory("tidybot_description")
    gazebo_ros   = get_package_share_directory("gazebo_ros")

    # ─────────────────────────────────────────────────────────
    world_file = os.path.join(pkg_gazebo, "worlds", "tidybot_home.world")
    xacro_file = os.path.join(pkg_desc,   "urdf",   "tidybot.urdf.xacro")

    gui_arg = DeclareLaunchArgument(
        name="gui",
        default_value="true",
        description="Launch Gazebo GUI (gzclient)",
    )
    world_arg = DeclareLaunchArgument(
        name="world",
        default_value=world_file,
        description="Gazebo world file",
    )

    x_arg = DeclareLaunchArgument(name="x",     default_value="1.0")
    y_arg = DeclareLaunchArgument(name="y",     default_value="2.0")
    z_arg = DeclareLaunchArgument(name="z",     default_value="0.22")
    yaw_arg = DeclareLaunchArgument(name="yaw", default_value="0.0")

    # ────────────────────────────────────────
    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", xacro_file]), value_type=str
        )
    }

    # ────────────────────────────────────────────────────────
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, "launch", "gzserver.launch.py")
        ),
        launch_arguments={"world": LaunchConfiguration("world")}.items(),
    )


    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, "launch", "gzclient.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("gui")),
    )

    # ───────────────────────────────────────────────
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

   
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                name="spawn_tidybot",
                output="screen",
                arguments=[
                    "-entity",    "tidybot",
                    "-topic",     "robot_description",
                    "-x",         LaunchConfiguration("x"),
                    "-y",         LaunchConfiguration("y"),
                    "-z",         LaunchConfiguration("z"),
                    "-Y",         LaunchConfiguration("yaw"),
                ],
            )
        ],
    )

    return LaunchDescription(
        [
            gui_arg,
            world_arg,
            x_arg, y_arg, z_arg, yaw_arg,  #Arguments for gui, world and spawn position
            gzserver,   
            gzclient,                      #Gazebo server + client
            rsp_node,                      #Robot state publisher (for TF frames)
            spawn_robot,                    #Spawn tidybot after 3 s 
        ]
    )
