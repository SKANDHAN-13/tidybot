"""
tidybot.launch.py
=================
SINGLE ENTRY POINT for the complete TidyBot simulation.

Launches all four components in the correct sequence:

  t=0   Component 1: robot_state_publisher with tidybot.urdf.xacro
  t=0   Component 2: Gazebo Classic  (gzserver + gzclient + spawn_entity)
  t=5   Component 3: Robot localisation (robot_localization EKF)
  t=8   Component 3: Nav2 full stack (AMCL, NavFn, DWB, behaviour server)
  t=30  Component 4: Behavior nodes  (object_detector, arm_controller,
                                       pick_and_place, task_manager)

Usage
-----
  # Full system with Gazebo window:
  ros2 launch tidybot_bringup tidybot.launch.py

  # Headless / CI mode:
  ros2 launch tidybot_bringup tidybot.launch.py gui:=false

Arguments
---------
  gui          (default true)   — show Gazebo client window
  use_sim_time (default true)   — all nodes use /clock
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # ── package directories ───────────────────────────────────────────────
    pkg_bringup  = get_package_share_directory("tidybot_bringup")
    pkg_desc     = get_package_share_directory("tidybot_description")
    pkg_gazebo   = get_package_share_directory("tidybot_gazebo")
    pkg_nav      = get_package_share_directory("tidybot_navigation")
    pkg_behavior = get_package_share_directory("tidybot_behavior")

    # ── arguments ─────────────────────────────────────────────────────────
    gui_arg = DeclareLaunchArgument(
        "gui", default_value="true",
        description="Show Gazebo client window",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Use simulation clock from /clock",
    )

    gui           = LaunchConfiguration("gui")
    use_sim_time  = LaunchConfiguration("use_sim_time")

    # ── Component 2+3: navigation launch (includes Gazebo + EKF + Nav2) ──
    #    This is the existing Component 3 entry point which already chains
    #    Gazebo → EKF → Nav2 internally.
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, "launch", "navigation.launch.py")
        ),
        launch_arguments={
            "gui":          gui,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # ── Component 4: behavior nodes (delayed to allow Nav2 to come up) ───
    behavior_launch = TimerAction(
        period=35.0,    # Nav2 lifecycle manager needs ~25-30 s to activate
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_behavior, "launch", "behavior.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                }.items(),
            )
        ],
    )

    return LaunchDescription([
        gui_arg,
        use_sim_time_arg,
        navigation_launch,
        behavior_launch,
    ])
