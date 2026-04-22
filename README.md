# TidyBot — ROS 2 Autonomous Room-Tidying Robot

A full-stack ROS 2 Humble simulation of a mobile manipulator that autonomously localises itself, detects objects on the floor using RGB-D perception, and picks them up one-by-one and deposits them into a collection box — all inside a Gazebo Classic environment.

---

## Table of Contents

1. [Overview](#overview)
2. [System Architecture](#system-architecture)
3. [Package Structure](#package-structure)
4. [Behaviour Scripts](#behaviour-scripts)
5. [Hardware / Sensor Model](#hardware--sensor-model)
6. [Prerequisites](#prerequisites)
7. [Build & Run](#build--run)
8. [Known Issues & Design Notes](#known-issues--design-notes)

---

## Overview

TidyBot spawns in a simulated home room alongside several small objects (blocks, cans, bottles, a shoe). It then:

1. Waits for Nav2 (AMCL + costmaps) and the object detector to come online.
2. Performs a brief in-place spin to help AMCL converge and collect a first point-cloud sweep.
3. Iterates over each detected object: navigates to a stand-off pose, aligns the on-torso RGB-D camera, then executes a pick-and-place arm sequence to lift the object and drop it in the collection box.
4. Reports `MISSION_SUCCESS` (or `MISSION_TIMEOUT`) when done.

---

## System Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│                          tidybot.launch.py                        │
│  (bringup — starts Gazebo, Nav2, EKF, RViz2, and all nodes below)│
└─────────────────────────────┬────────────────────────────────────┘
                              │
          ┌───────────────────┼──────────────────────┐
          ▼                   ▼                       ▼
   [Gazebo Classic]    [Nav2 stack]           [tidybot_behavior]
   gzserver / client   AMCL, NavFn            ├─ task_manager
   robot spawned       DWB, costmaps          ├─ object_detector
   joint state pub     EKF odometry           ├─ pick_and_place
   skid-steer drive    map_server             ├─ arm_controller
                                              └─ collision_monitor
```

**Topic / service summary**

| Topic / Service | Direction | Used by |
|---|---|---|
| `/gazebo/model_states` | Gazebo → ROS | `object_detector` (BEST_EFFORT + RELIABLE dual-subscribe) |
| `/gazebo/get_model_state` | service | `object_detector` (2 s poll fallback) |
| `/tidybot/rgbd_camera/points` | Gazebo → ROS | `object_detector` (PC2 cluster backup) |
| `/tidybot_detect/confirmed_objects` | `object_detector` → | `task_manager`, `pick_and_place` |
| `/tidybot_task/status` | `task_manager` → | `collision_monitor` |
| `/tidybot_task/start` | `task_manager` → | `pick_and_place` |
| `/tidybot_nav/pnp_status` | `pick_and_place` → | `task_manager` |
| `/tidybot_arm/command` | `pick_and_place` → | `arm_controller` |
| `/tidybot_arm/status` | `arm_controller` → | `pick_and_place` |
| `/tidybot/contact` | Gazebo bumper → ROS | `collision_monitor` |
| `/tidybot/ultrasonic_{front,left,right,rear}` | Gazebo sonar → ROS | `collision_monitor` |
| `/navigate_to_pose` (action) | `pick_and_place` → Nav2 | navigation |

---

## Package Structure

```
src/
├── tidybot_behavior/          # Python — all high-level autonomy nodes
│   ├── tidybot_behavior/
│   │   ├── task_manager.py
│   │   ├── object_detector.py
│   │   ├── pick_and_place.py
│   │   ├── arm_controller.py
│   │   └── collision_monitor.py
│   ├── launch/behavior.launch.py
│   └── setup.py
│
├── tidybot_bringup/           # Top-level launch gluing everything together
│   └── launch/tidybot.launch.py
│
├── tidybot_control/           # ros2_control YAML + launch (not used in Gazebo Classic path)
│   └── config/tidybot_controllers.yaml
│
├── tidybot_description/       # URDF/Xacro robot model + RViz2 configs
│   ├── urdf/tidybot.urdf.xacro
│   └── rviz/tidybot_sensors.rviz
│
├── tidybot_gazebo/            # Gazebo world + spawn launch
│   └── worlds/tidybot_home.world
│
└── tidybot_navigation/        # Nav2 params, EKF config, pre-built map
    ├── config/nav2_params.yaml
    ├── config/ekf.yaml
    └── maps/tidybot_home.{pgm,yaml}
```

---

## Behaviour Scripts

### `task_manager.py`
Top-level sequencer. Publishes task-state strings on `/tidybot_task/status` that gate the collision monitor. State machine:

```
WAITING_FOR_NAV2 → WAITING_FOR_DETECTOR → WAITING_FOR_AMCL_CONVERGENCE
 → OBJECT_SEARCH_SPIN → STARTING_PICK_AND_PLACE → MISSION_SUCCESS | MISSION_TIMEOUT
```

### `object_detector.py`
Triple-redundant object localisation:
- **BEST_EFFORT** subscription to `/gazebo/model_states`
- **RELIABLE** subscription to `/gazebo/model_states` (handles Gazebo Classic QoS variability)
- **2 s service poll** of `/gazebo/get_model_state` for each known object name (guaranteed fallback)

Publishes confirmed poses as `geometry_msgs/PoseArray` on `/tidybot_detect/confirmed_objects`.

### `pick_and_place.py`
Full pick-and-place state machine. For each object:
1. Sends Nav2 goal to a 0.6 m stand-off pose behind the object.
2. Aligns the torso camera by rotating in-place until the object centroid is centred in frame.
3. Calls `arm_controller` with the sequence: `reach → lower → grasp → lift → carry → lower_to_box → release → retract`.
4. Publishes progress strings on `/tidybot_nav/pnp_status`.

### `arm_controller.py`
Named-posture joint trajectory executor. Sends `JointTrajectory` messages directly to the Gazebo joint controller topic.

**Shoulder joint constraints** — the right arm has a dead zone `(-0.89, +0.89)` rad where the torque is insufficient with a fully extended elbow. All postures are constrained to the positive hemisphere `[+0.89, π]`:

| Posture | Shoulder (rad) | Purpose |
|---|---|---|
| `home` / `carry` / `retract` | +1.22 | Neutral / transport |
| `reach` | +1.00 | Extended reach toward object |
| `lower` / `grasp` | +1.00, elbow –1.55 | Lower to floor level |
| `lower_to_box` | +0.90 – +1.00 | Deposit into collection box |

### `collision_monitor.py`
20 Hz safety loop. Reads four ultrasonic sensors (front, left, right, rear) and a bumper contact sensor.

- **Ultrasonic STOP** is **suppressed** during task states in `SUPPRESS_ULTRASONIC_STATES` = `{WAITING_FOR_NAV2, WAITING_FOR_DETECTOR, WAITING_FOR_AMCL_CONVERGENCE, OBJECT_SEARCH_SPIN}` — prevents false stops from spawn-wall echoes during initialisation and search spin.
- **Contact sensor STOP** is always active regardless of task state.
- When suppression lifts (transition to `STARTING_PICK_AND_PLACE`), any latched emergency state is cleared.

---

## Hardware / Sensor Model

| Component | Details |
|---|---|
| Drive | Skid-steer, 4 wheels, 0.55 m track width, 0.1 m wheel diameter |
| Arm | 5-DOF right arm: shoulder, elbow, wrist + 2-finger gripper |
| Localisation | AMCL on a 195×92 px / 0.05 m/cell pre-built map |
| Odometry fusion | robot_localization EKF (wheel odometry + IMU) |
| Depth camera | RGBD (PointCloud2), mounted on torso, tilted downward |
| Proximity | Front, left, right, rear ultrasonic sonars (Gazebo ray plugins) |
| Collision | Gazebo bumper plugin → contact states |

---

## Prerequisites

- **ROS 2 Humble** (Ubuntu 22.04)
- **Gazebo Classic 11**
- ROS packages: `nav2_*`, `robot_localization`, `trajectory_msgs`, `gazebo_ros`, `gazebo_msgs`
- Python 3.10

Install ROS 2 Humble: https://docs.ros.org/en/humble/Installation.html

---

## Build & Run

```bash
# 1. Clone into your ROS 2 workspace
mkdir -p ~/tidybot_ws/src && cd ~/tidybot_ws/src
git clone https://github.com/SKANDHAN-13/tidybot.git .

# 2. Install dependencies
cd ~/tidybot_ws
rosdep install --from-paths src --ignore-src -r -y

# 3. Build
colcon build
source install/setup.bash

# 4. Launch everything (Gazebo + Nav2 + behaviour nodes + RViz2)
ros2 launch tidybot_bringup tidybot.launch.py
```

The robot will automatically begin its tidying mission after Nav2 comes up (typically ~15 s).

### Useful debug commands

```bash
# Check object detections
ros2 topic echo /tidybot_detect/confirmed_objects

# Watch task state
ros2 topic echo /tidybot_task/status

# Watch arm commands
ros2 topic echo /tidybot_arm/command

# Manual Gazebo model query (verify objects are spawned)
ros2 service call /gazebo/get_model_state \
  gazebo_msgs/srv/GetModelState \
  "{model_name: 'obj_red_block', relative_entity_name: 'world'}"

# Live joint positions
ros2 topic echo /tidybot/joint_states
```

---

## Known Issues & Design Notes

| Issue | Status | Notes |
|---|---|---|
| `object_detector` shows 0 confirmed despite 25+ PC2 clusters | Active | `/gazebo/model_states` QoS mismatch with Gazebo Classic ROS 2 bridge; service-poll fallback added as guaranteed path |
| Arm controller crash on launch (`AssertionError: positions must be float`) | Active | `positions` list type mismatch — posture values need explicit `float()` cast |
| Collision monitor fires at t=1s | Fixed | Rear ultrasonic sees spawn wall; suppressed during non-navigation states via task-state awareness |
| Arms crossed dead-zone (shoulder 0.0–0.8 rad) | Fixed | All postures moved to ≥+0.89 rad positive hemisphere |
| `rcl_shutdown already called` tracebacks on Ctrl-C | Cosmetic | ROS 2 Humble lifecycle manager races with SIGINT; no functional impact |
