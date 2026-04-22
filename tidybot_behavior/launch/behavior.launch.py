"""
behavior.launch.py
==================
Launches all TidyBot behavior nodes:
  1. object_detector   — perceives pickup objects via Gazebo + PointCloud2
  2. arm_controller    — custom joint trajectory controller for the arm
  3. pick_and_place    — reach-grasp-lift-carry-place state machine
  4. task_manager      — top-level mission orchestrator

Nodes are staggered to allow arm_controller to initialise and send the
arm to its home posture before pick_and_place starts waiting for detections.
"""

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():

    # Starts immediately — must be live before navigation begins
    collision_monitor_node = Node(
        package="tidybot_behavior",
        executable="collision_monitor",
        name="collision_monitor",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    object_detector_node = Node(
        package="tidybot_behavior",
        executable="object_detector",
        name="object_detector",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    arm_controller_node = Node(
        package="tidybot_behavior",
        executable="arm_controller",
        name="arm_controller",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    pick_and_place_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="tidybot_behavior",
                executable="pick_and_place",
                name="pick_and_place",
                output="screen",
                parameters=[{"use_sim_time": True}],
            )
        ],
    )

    task_manager_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="tidybot_behavior",
                executable="task_manager",
                name="task_manager",
                output="screen",
                parameters=[{"use_sim_time": True}],
            )
        ],
    )

    return LaunchDescription([
        collision_monitor_node,
        object_detector_node,
        arm_controller_node,
        pick_and_place_node,
        task_manager_node,
    ])
