"""
control.launch.py
=================
Placeholder launch file for TidyBot ros2_control stack.

TidyBot's simulation uses Gazebo Classic with
libgazebo_ros_joint_pose_trajectory.so for arm joint control, so
ros2_control is not required at runtime.  This file is provided for:
  - Reference and future migration to ros2_control hardware interface
  - Satisfying the CMakeLists install(DIRECTORY launch ...) directive

For the active control pipeline, see:
  tidybot_behavior/launch/behavior.launch.py  → arm_controller node
  tidybot_bringup/launch/tidybot.launch.py    → full system entry point
"""

from launch import LaunchDescription


def generate_launch_description():
    # All arm control is handled by libgazebo_ros_joint_pose_trajectory.so
    # (loaded by Gazebo through the URDF plugin block) and by the
    # tidybot_behavior arm_controller node.
    return LaunchDescription([])
