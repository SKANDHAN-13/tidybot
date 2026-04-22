"""
navigation.launch.py
=====================
Single-command entry point for TidyBot Component 3.

Launches (in order):
  1. Gazebo  (gzserver + gzclient)  via tidybot_gazebo/gazebo.launch.py
  2. robot_localization EKF          (wheel odom + IMU -> /odometry/filtered)
  3. Nav2 full stack                 (map_server, amcl, planner, controller,
                                      behavior_server, bt_navigator,
                                      waypoint_follower, lifecycle_manager)
  4. waypoint_navigator              (sends robot through both rooms)

Map choice: Pre-built occupancy grid (not live SLAM).
Rationale: SLAM requires extra compute and sensor warm-up time.  A pre-built
map lets the robot localize with AMCL immediately, allowing the full mission
to complete well within the 5-minute simulation limit.  The map was generated
by generate_map.py to exactly match tidybot_home.world geometry.

Usage:
  ros2 launch tidybot_navigation navigation.launch.py
  ros2 launch tidybot_navigation navigation.launch.py gui:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap


def generate_launch_description():

    pkg_nav    = get_package_share_directory("tidybot_navigation")
    pkg_gazebo = get_package_share_directory("tidybot_gazebo")
    pkg_nav2   = get_package_share_directory("nav2_bringup")
    pkg_desc   = get_package_share_directory("tidybot_description")

    # ── Paths ──────────────────────────────────────────────────────────────
    map_yaml     = os.path.join(pkg_nav, "maps",   "tidybot_home.yaml")
    nav2_params  = os.path.join(pkg_nav, "config", "nav2_params.yaml")
    ekf_config   = os.path.join(pkg_nav, "config", "ekf.yaml")

    # ── Arguments ──────────────────────────────────────────────────────────
    gui_arg = DeclareLaunchArgument(
        "gui", default_value="true",
        description="Open Gazebo client window",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
    )

    # ── 1. Gazebo world + robot spawn ──────────────────────────────────────
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "gui": LaunchConfiguration("gui"),
        }.items(),
    )

    # ── 2. EKF sensor fusion (robot_localization) ──────────────────────────
    # Fuses /tidybot/odom + /tidybot/imu -> /odometry/filtered
    # Starts after Gazebo has time to publish sensors (5 s)
    ekf_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_node",
                output="screen",
                parameters=[
                    ekf_config,
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
                remappings=[
                    ("/odometry/filtered", "/odometry/filtered"),
                ],
            )
        ],
    )

    # ── 3. Nav2 full stack ─────────────────────────────────────────────────
    # Starts 8 s after launch (after EKF + sensor warmup)
    nav2_launch = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav2, "launch", "bringup_launch.py")
                ),
                launch_arguments={
                    "use_sim_time":    LaunchConfiguration("use_sim_time"),
                    "autostart":       "true",
                    "map":             map_yaml,
                    "params_file":     nav2_params,
                    "use_composition": "False",
                }.items(),
            )
        ],
    )

    # ── 4. Waypoint navigator — disabled: task_manager runs its own survey
    #    Running both simultaneously causes competing nav goals that preempt
    #    each other and prevent the robot from moving.
    waypoint_node = TimerAction(
        period=25.0,
        actions=[
            Node(
                package="tidybot_navigation",
                executable="waypoint_navigator",
                name="waypoint_navigator",
                output="screen",
                parameters=[
                    {"use_sim_time": LaunchConfiguration("use_sim_time")}
                ],
            )
        ],
    )

    # ── RViz2: sensor visualisation (LaserScan, PointCloud2, Camera, Map) ──
    rviz_config = os.path.join(pkg_desc, "rviz", "tidybot_sensors.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        condition=IfCondition(LaunchConfiguration("gui")),
        output="screen",
    )

    return LaunchDescription([
        gui_arg,
        use_sim_time_arg,
        gazebo_launch,
        ekf_node,
        nav2_launch,
        rviz_node,
        # waypoint_node disabled — task_manager handles navigation
    ])
