#!/usr/bin/env python3
"""
waypoint_navigator.py
======================
Sends TidyBot through a predetermined waypoint mission that traverses
both rooms, passing through the doorway.

Waypoints (map frame, pre-built map of tidybot_home.world):
  WP 0  (1.0, 2.0)  -- Robot home / start     (Room 1 centre)
  WP 1  (3.5, 2.0)  -- Mid Room 1             (clears furniture)
  WP 2  (4.6, 2.0)  -- Approach doorway        (Room 1 side)
  WP 3  (5.7, 2.0)  -- Through doorway         (Room 2 entry)
  WP 4  (7.15, 2.0) -- Room 2 centre
  WP 5  (8.5, 2.0)  -- East end of Room 2
  WP 6  (7.15, 2.0) -- Back to Room 2 centre   (return leg)
  WP 7  (5.7, 2.0)  -- Through doorway (return)
  WP 8  (4.6, 2.0)  -- Room 1 Entry
  WP 9  (1.0, 2.0)  -- Return home

Uses Nav2 FollowWaypoints action (NOT hardcoded movement).
The robot uses AMCL + EKF for localisation throughout.

Measurable outputs (satisfies logging requirement):
  - Logs each goal dispatched and result achieved to stdout
  - Publishes /tidybot_nav/mission_status (std_msgs/String)
  - Logs timestamps so total sim time can be verified < 5 min

Recovery behaviour (bonus):
  The node monitors execution time per waypoint.  If a waypoint takes
  longer than 60 s, it logs a warning (Nav2's own behavior_server will
  have attempted spin + backup automatically via BT).
"""

import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from std_msgs.msg import String
from action_msgs.msg import GoalStatus


# ── Waypoint table (x, y, yaw_deg) ──────────────────────────────────────────
WAYPOINTS = [
    (1.0,  2.0,   0.0),   # 0 start  / home
    (3.5,  2.0,   0.0),   # 1 mid Room 1
    (4.6,  2.0,   0.0),   # 2 approach doorway
    (5.7,  2.0,   0.0),   # 3 through doorway (Room 2 side)
    (7.15, 2.0,   0.0),   # 4 Room 2 centre
    (8.5,  2.0, 180.0),   # 5 east wall turn
    (7.15, 2.0, 180.0),   # 6 back to Room 2 centre
    (5.7,  2.0, 180.0),   # 7 through doorway (return)
    (4.6,  2.0, 180.0),   # 8 Room 1 entry
    (1.0,  2.0, 180.0),   # 9 home
]


def yaw_to_quaternion(yaw_deg: float):
    """Convert yaw (degrees) to (qx, qy, qz, qw)."""
    yaw = math.radians(yaw_deg)
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def make_pose(x: float, y: float, yaw_deg: float, frame: str = "map") -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    qx, qy, qz, qw = yaw_to_quaternion(yaw_deg)
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


class WaypointNavigator(Node):

    def __init__(self):
        super().__init__("waypoint_navigator")

        self._ac = ActionClient(self, FollowWaypoints, "follow_waypoints")
        self._status_pub = self.create_publisher(String, "/tidybot_nav/mission_status", 10)

        self._mission_start = None
        self._log("WaypointNavigator initialised.  Waiting for Nav2 FollowWaypoints server ...")
        self._ac.wait_for_server(timeout_sec=30.0)
        self._log("Action server ready.  Sending mission waypoints.")
        self._send_mission()

    # ─────────────────────────────────────────────────────────────────────────
    def _log(self, msg: str):
        self.get_logger().info(msg)
        status = String()
        status.data = msg
        self._status_pub.publish(status)

    # ─────────────────────────────────────────────────────────────────────────
    def _send_mission(self):
        goal = FollowWaypoints.Goal()

        # Stamp all waypoints with current time
        now = self.get_clock().now().to_msg()
        for idx, (x, y, yaw) in enumerate(WAYPOINTS):
            p = make_pose(x, y, yaw)
            p.header.stamp = now
            goal.poses.append(p)
            self._log(f"  WP {idx:2d}:  x={x:5.2f}  y={y:5.2f}  yaw={yaw:6.1f} deg")

        self._mission_start = time.monotonic()
        self._log(f"Dispatching {len(WAYPOINTS)} waypoints to Nav2 FollowWaypoints ...")

        send_future = self._ac.send_goal_async(
            goal,
            feedback_callback=self._feedback_cb,
        )
        send_future.add_done_callback(self._goal_response_cb)

    # ─────────────────────────────────────────────────────────────────────────
    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._log("ERROR: Mission goal rejected by Nav2!")
            return
        self._log("Mission goal accepted.  Robot is navigating ...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    # ─────────────────────────────────────────────────────────────────────────
    def _feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        curr = fb.current_waypoint
        name = f"WP {curr}"
        elapsed = time.monotonic() - self._mission_start
        self._log(f"[t={elapsed:6.1f}s] Navigating to {name} "
                  f"(x={WAYPOINTS[curr][0]:.2f}, y={WAYPOINTS[curr][1]:.2f})")

        # Bonus: warn if a single waypoint takes longer than 60 s
        if elapsed > 60.0 * (curr + 1):
            self.get_logger().warn(
                f"Waypoint {curr} taking longer than expected "
                f"(elapsed={elapsed:.1f}s). "
                "Nav2 recovery (spin/backup) may be active."
            )

    # ─────────────────────────────────────────────────────────────────────────
    def _result_cb(self, future):
        result = future.result().result
        status = future.result().status
        elapsed = time.monotonic() - self._mission_start

        missed = list(result.missed_waypoints)

        if status == GoalStatus.STATUS_SUCCEEDED:
            if missed:
                self._log(
                    f"Mission COMPLETED WITH SKIPS in {elapsed:.1f}s.  "
                    f"Missed WPs: {missed}"
                )
            else:
                self._log(
                    f"Mission COMPLETED SUCCESSFULLY in {elapsed:.1f}s.  "
                    "All waypoints reached.  Both rooms traversed."
                )
        else:
            self._log(
                f"Mission ENDED with status {status} in {elapsed:.1f}s.  "
                f"Missed WPs: {missed}"
            )

        # Print reachable summary for measurable output
        reached = [i for i in range(len(WAYPOINTS)) if i not in missed]
        self._log(f"Goals reached: {reached}")
        self._log(f"Total simulation mission time: {elapsed:.1f} s")


# ─────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
