# TidyBot — ROS 2 Autonomous Room-Tidying Robot

A full-stack ROS 2 Humble simulation of a skid-steer mobile manipulator that
autonomously localises itself, detects objects on the floor using RGB-D
perception, and picks them up one-by-one to deposit them in a collection box —
all inside a Gazebo Classic environment.

---

## Table of Contents

1. [Quick Start](#quick-start)
2. [Prerequisites — System](#prerequisites--system)
3. [Prerequisites — ROS 2 Packages](#prerequisites--ros-2-packages)
4. [Python Dependencies](#python-dependencies)
5. [Workspace Setup & Build](#workspace-setup--build)
6. [Launch Instructions](#launch-instructions)
7. [System Architecture](#system-architecture)
8. [Package Reference](#package-reference)
9. [Behaviour Nodes](#behaviour-nodes)
10. [Tooling Scripts](#tooling-scripts)
11. [Known Issues](#known-issues)

---

## Quick Start

```bash
# 1. Install ROS 2 Humble (see Prerequisites section below if not already done)

# 2. Install all apt dependencies
sudo apt update && sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-nav2-bringup \
  ros-humble-navigation2 \
  ros-humble-robot-localization \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-xacro \
  ros-humble-slam-toolbox \
  ros-humble-trajectory-msgs \
  ros-humble-rmw-cyclonedds-cpp \
  python3-colcon-common-extensions \
  python3-rosdep

# 3. Clone
mkdir -p ~/tidybot_ws/src && cd ~/tidybot_ws/src
git clone https://github.com/SKANDHAN-13/tidybot.git .

# 4. Python dependencies
pip install -r requirements.txt

# 5. rosdep
cd ~/tidybot_ws
rosdep init || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 6. Build
colcon build
source install/setup.bash

# 7. Launch
ros2 launch tidybot_bringup tidybot.launch.py
```

The robot starts its tidying mission automatically ~15 s after launch (time for
Nav2 + AMCL to initialise).

---

## Prerequisites — System

| Requirement | Version | Notes |
|---|---|---|
| Ubuntu | 22.04 LTS | 24.04 with ROS 2 Jazzy should also work |
| ROS 2 | Humble Hawksbill | https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html |
| Gazebo Classic | 11.x | Installed automatically with `ros-humble-gazebo-ros-pkgs` |
| Python | 3.10 | Ships with Ubuntu 22.04 |
| pip | ≥ 22 | `sudo apt install python3-pip` |

### Installing ROS 2 Humble from scratch

```bash
# Locale
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add ROS 2 apt repo
sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list

sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions python3-rosdep

# Source in every new shell (add to ~/.bashrc)
source /opt/ros/humble/setup.bash
```

---

## Prerequisites — ROS 2 Packages

Install all ROS 2 dependencies with a single command:

```bash
sudo apt update && sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros \
  ros-humble-gazebo-msgs \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-nav2-msgs \
  ros-humble-nav2-simple-commander \
  ros-humble-robot-localization \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-xacro \
  ros-humble-slam-toolbox \
  ros-humble-trajectory-msgs \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-rviz2 \
  ros-humble-controller-manager \
  ros-humble-joint-trajectory-controller \
  ros-humble-joint-state-broadcaster \
  ros-humble-rmw-cyclonedds-cpp \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-pip
```

> **Tip:** After installing `python3-rosdep`, run:
> ```bash
> sudo rosdep init && rosdep update
> ```
> then inside the workspace root:
> ```bash
> rosdep install --from-paths src --ignore-src -r -y
> ```
> to auto-install anything missed above.

---

## Python Dependencies

```bash
pip install -r requirements.txt
```

| Package | Purpose |
|---|---|
| `numpy` | Cluster maths in `object_detector.py` |
| `opencv-python` | Point-cloud cluster analysis |
| `scipy` | Spatial transforms / KDTree (optional cluster merging) |
| `open3d` | Richer point-cloud debugging (optional) |
| `matplotlib` | `tools/compute_inertia.py` geometry plots |

All ROS Python bindings (`rclpy`, `sensor_msgs`, `nav2_msgs`, etc.) are
installed as part of the ROS 2 desktop installation and are **not** in
`requirements.txt`.

---

## Workspace Setup & Build

```bash
mkdir -p ~/tidybot_ws/src
cd ~/tidybot_ws/src
git clone https://github.com/SKANDHAN-13/tidybot.git .

cd ~/tidybot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

> Add `source ~/tidybot_ws/install/setup.bash` to `~/.bashrc` so you do not
> have to source it in every terminal.

---

## Launch Instructions

### Full simulation (Gazebo + Nav2 + RViz2 + all behaviour nodes)

```bash
source ~/tidybot_ws/install/setup.bash
ros2 launch tidybot_bringup tidybot.launch.py
```

This single launch file starts:

| Component | What it runs |
|---|---|
| Gazebo Classic | `gzserver` + `gzclient`, loads `tidybot_home.world`, spawns robot |
| Robot State Publisher | Publishes TF tree from URDF |
| EKF (robot_localization) | Fuses wheel odometry + IMU → `/odometry/filtered` |
| Nav2 stack | AMCL, NavFn planner, DWB controller, costmaps, lifecycle managers |
| RViz2 | Visualisation with pre-configured `tidybot_sensors.rviz` |
| `task_manager` | Top-level sequencer |
| `object_detector` | RGB-D + Gazebo model-state fusion |
| `pick_and_place` | Navigation + arm orchestration |
| `arm_controller` | Joint trajectory executor |
| `collision_monitor` | 20 Hz ultrasonic + bumper safety loop |

### Individual sub-stacks

```bash
# Gazebo + robot description only
ros2 launch tidybot_gazebo gazebo.launch.py

# Nav2 only (requires Gazebo already running)
ros2 launch tidybot_navigation navigation.launch.py

# Behaviour nodes only (requires Gazebo + Nav2 already running)
ros2 launch tidybot_behavior behavior.launch.py
```

### Useful debug commands

```bash
# Live task state
ros2 topic echo /tidybot_task/status

# Confirmed object detections
ros2 topic echo /tidybot_detect/confirmed_objects

# Arm command / status
ros2 topic echo /tidybot_arm/command
ros2 topic echo /tidybot_arm/status

# Live joint positions
ros2 topic echo /tidybot/joint_states

# Manually query a Gazebo object pose
ros2 service call /gazebo/get_model_state \
  gazebo_msgs/srv/GetModelState \
  "{model_name: 'obj_red_block', relative_entity_name: 'world'}"

# Re-run map generation (outputs tidybot_home.pgm + tidybot_home.yaml)
cd src/tidybot_navigation/maps
python3 generate_map.py
```

---

## System Architecture

```
+--------------------------------------------------------------------------+
|                        tidybot.launch.py (bringup)                       |
+-----------------------------------+--------------------------------------+
                                    |
          +--------------------------+-------------------------+
          v                         v                         v
   [Gazebo Classic 11]        [Nav2 stack]             [tidybot_behavior]
   gzserver / gzclient         AMCL                     task_manager
   tidybot_home.world           NavFn planner            object_detector
   robot spawned                DWB controller           pick_and_place
   joint_state_pub              costmaps                 arm_controller
   skid-steer drive             EKF odometry             collision_monitor
   RGBD camera plugin           map_server
   ultrasonic sonar x4          lifecycle_manager x2
   bumper contact plugin
```

### Key topics

| Topic | Type | Publisher → Subscriber |
|---|---|---|
| `/gazebo/model_states` | `ModelStates` | Gazebo → `object_detector` |
| `/gazebo/get_model_state` | service | `object_detector` → Gazebo (2 s poll) |
| `/tidybot/rgbd_camera/points` | `PointCloud2` | Gazebo → `object_detector` |
| `/tidybot_detect/confirmed_objects` | `PoseArray` | `object_detector` → `task_manager`, `pick_and_place` |
| `/tidybot_task/status` | `String` | `task_manager` → `collision_monitor` |
| `/tidybot_task/start` | `String` | `task_manager` → `pick_and_place` |
| `/tidybot_nav/pnp_status` | `String` | `pick_and_place` → `task_manager` |
| `/tidybot_arm/command` | `String` | `pick_and_place` → `arm_controller` |
| `/tidybot_arm/status` | `String` | `arm_controller` → `pick_and_place` |
| `/tidybot/contact` | `ContactsState` | Gazebo → `collision_monitor` |
| `/tidybot/ultrasonic_{front,left,right,rear}` | `Range` | Gazebo → `collision_monitor` |
| `/navigate_to_pose` | action | `pick_and_place` → Nav2 |
| `/cmd_vel` | `Twist` | Nav2 / `collision_monitor` → robot |

---

## Package Reference

```
src/
+-- tidybot_behavior/          # Python — all high-level autonomy nodes
|   +-- tidybot_behavior/
|   |   +-- task_manager.py       # top-level state sequencer
|   |   +-- object_detector.py    # RGB-D + model_states perception
|   |   +-- pick_and_place.py     # navigation + arm orchestration
|   |   +-- arm_controller.py     # named-posture joint trajectory executor
|   |   +-- collision_monitor.py  # 20 Hz safety loop
|   +-- launch/behavior.launch.py
|
+-- tidybot_bringup/           # Top-level launch (glues everything)
|   +-- launch/tidybot.launch.py
|
+-- tidybot_control/           # ros2_control config (reference / future use)
|   +-- config/tidybot_controllers.yaml
|
+-- tidybot_description/       # URDF/Xacro robot model + RViz2 configs
|   +-- urdf/tidybot.urdf.xacro
|   +-- rviz/tidybot_sensors.rviz
|
+-- tidybot_gazebo/            # Gazebo world + spawn launch
|   +-- worlds/tidybot_home.world
|
+-- tidybot_navigation/        # Nav2 params, EKF config, pre-built map
|   +-- config/nav2_params.yaml
|   +-- config/ekf.yaml
|   +-- maps/tidybot_home.{pgm,yaml}
|   +-- maps/generate_map.py   # map generation script
|
+-- tools/
|   +-- compute_inertia.py     # inertia tensor calculator + URDF blocks
|
+-- requirements.txt
+-- README.md
```

---

## Behaviour Nodes

### task_manager.py

Top-level sequencer. Publishes task-state strings on `/tidybot_task/status`
that gate the collision monitor. State machine:

```
WAITING_FOR_NAV2
  -> WAITING_FOR_DETECTOR
  -> WAITING_FOR_AMCL_CONVERGENCE
  -> OBJECT_SEARCH_SPIN
  -> STARTING_PICK_AND_PLACE
  -> MISSION_SUCCESS | MISSION_TIMEOUT
```

### object_detector.py

Triple-redundant object localisation to work around Gazebo Classic
non-deterministic QoS behaviour:

1. BEST_EFFORT subscription to `/gazebo/model_states`
2. RELIABLE subscription to `/gazebo/model_states`
3. 2 s service poll of `/gazebo/get_model_state` for each known object name

Publishes `PoseArray` on `/tidybot_detect/confirmed_objects`.

Known objects: `obj_red_block`, `obj_blue_block`, `obj_yellow_can`,
`obj_green_box`, `obj_orange_block`, `obj_teal_bottle`, `obj_purple_shoe`

### pick_and_place.py

Full pick-and-place state machine. For each detected object:

1. Sends Nav2 goal to a stand-off pose (0.6 m offset).
2. Aligns the torso RGB-D camera by rotating in-place.
3. Executes posture sequence:
   `reach -> lower -> grasp -> lift -> carry -> lower_to_box -> release -> retract`
4. Publishes progress on `/tidybot_nav/pnp_status`.

### arm_controller.py

Named-posture joint trajectory executor. Sends `JointTrajectory` messages
to the Gazebo joint controller.

**Shoulder constraint:** dead zone (-0.89, +0.89) rad — all postures stay in
the positive hemisphere [+0.89, pi]:

| Posture | Shoulder (rad) | Purpose |
|---|---|---|
| home / carry / retract | +1.22 | Neutral / transport (URDF spawn angle) |
| reach | +1.00 | Reach toward object |
| lower / grasp | +1.00 | Lower to floor, close gripper |
| lower_to_box | +0.90 to +1.00 | Deposit into collection box |

### collision_monitor.py

20 Hz safety loop. Ultrasonic STOP suppressed during:

```
WAITING_FOR_NAV2 | WAITING_FOR_DETECTOR |
WAITING_FOR_AMCL_CONVERGENCE | OBJECT_SEARCH_SPIN
```

Contact sensor STOP is always active regardless of task state.

---

## Hardware / Sensor Model

| Component | Details |
|---|---|
| Drive | Skid-steer, 4 wheels, 0.55 m track width, 0.10 m diameter |
| Arm | 5-DOF right arm: shoulder, elbow, wrist + 2-finger parallel gripper |
| Localisation | AMCL on a 195 x 92 px / 0.05 m/cell pre-built map |
| Odometry | robot_localization EKF (wheel odometry + IMU) |
| Depth camera | RGBD (PointCloud2), mounted on torso, tilted 30 deg downward |
| Proximity | Front, left, right, rear ultrasonic sonars (Gazebo ray sensors) |
| Collision | Gazebo bumper plugin -> /tidybot/contact |
| Total mass | ~29 kg (see tools/compute_inertia.py) |

---

## Tooling Scripts

### tidybot_navigation/maps/generate_map.py

Generates the pre-built occupancy-grid map (`tidybot_home.pgm` +
`tidybot_home.yaml`) that exactly matches `tidybot_home.world`.

```bash
cd src/tidybot_navigation/maps
python3 generate_map.py
# Outputs: tidybot_home.pgm  (195 x 92 px binary PGM, 0.05 m/px)
#          tidybot_home.yaml (Nav2 map metadata)
```

The map is committed to the repo; only re-run this if you modify the world.

Map geometry:
- Resolution: 0.05 m/pixel, Origin: (-0.30, -0.30, 0.0) m
- Room 1 interior: x in [0.00, 5.00], y in [0.00, 4.00] m
- Room 2 interior: x in [5.15, 9.15], y in [0.00, 4.00] m
- Doorway: y in [1.50, 2.50] at x = 5.00 to 5.15 m

### tools/compute_inertia.py

Calculates analytical inertia tensors for every URDF link and prints
ready-to-paste `<inertial>` blocks. The output values were used directly in
the `box_inertia` and `cyl_inertia_y/z` xacro macros.

```bash
python3 tools/compute_inertia.py
```

Sample output:
```
TidyBot -- Inertia Tensor Summary
Link                                   mass (kg)      ixx           iyy           izz
base_link                                20.000   0.426667      0.426667      0.833333
wheel (each, x4)                          1.000   0.000729      0.001250      0.000729
torso_link                                5.000   0.090208      0.105208      0.026042
...
Total robot mass (approx): 29.20 kg
```

Formulas used:
- Solid cuboid (w x d x h):  ixx = m/12*(d^2+h^2),  iyy = m/12*(w^2+h^2),  izz = m/12*(w^2+d^2)
- Cylinder axis Y (r, l):    ixx = izz = m/12*(3r^2+l^2),  iyy = m/2*r^2
- Cylinder axis Z (r, l):    ixx = iyy = m/12*(3r^2+l^2),  izz = m/2*r^2

---

## Known Issues

| Issue | Status | Workaround |
|---|---|---|
| `object_detector` shows 0 confirmed objects | Active | `/gazebo/model_states` QoS is non-deterministic in Gazebo Classic; service-poll fallback added — verify with `ros2 service call /gazebo/get_model_state ...` |
| `arm_controller` startup crash (AssertionError in positions field) | Active | Add explicit `float()` casts to posture position lists |
| `rcl_shutdown already called` tracebacks on Ctrl-C | Cosmetic | ROS 2 Humble lifecycle manager races with SIGINT; no functional impact |
| Inflation radius warning (0.250 < inscribed 0.324) | Cosmetic | Increase `inflation_radius` in `nav2_params.yaml` to >= 0.35 |
