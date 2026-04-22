cmake_minimum_required(VERSION 3.8)
project(tidybot_navigation)

find_package(ament_cmake REQUIRED)

# Install shared directories (config, maps, launch files)
install(DIRECTORY config maps launch scripts
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts as executables so `ros2 run` and Node() can find them
install(PROGRAMS
  scripts/waypoint_navigator.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME waypoint_navigator
)

ament_package()
