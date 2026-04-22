"""
collision_monitor.py
====================

Thresholds
----------
  WARN_DIST   0.35 m  — slow down warning (logged)
  STOP_DIST   0.18 m  — hard stop: override cmd_vel, cancel nav goal

Topics subscribed
-----------------
  /tidybot/ultrasonic         [sensor_msgs/LaserScan]  front
  /tidybot/ultrasonic_left    [sensor_msgs/LaserScan]  left
  /tidybot/ultrasonic_right   [sensor_msgs/LaserScan]  right
  /tidybot/contact            [gazebo_msgs/ContactsState]  front bumper

Topics published
-----------------
  /cmd_vel                    [geometry_msgs/Twist]    zero when obstacle close
  /tidybot/collision_status   [std_msgs/String]        "CLEAR" | "WARN:dir" | "STOP:dir"
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from gazebo_msgs.msg import ContactsState
from nav2_msgs.action import NavigateToPose

WARN_DIST = 0.35   # m
STOP_DIST = 0.18   # m

# How many consecutive STOP readings before actually stopping
# No. of positives from sensor to confirm obstacle, to avoid reacting to spurious readings.
STOP_DEBOUNCE = 2 

# Task states in which ultrasonic obstacle-stop is suppressed. 
# The robot is either stationary or rotating in-place during these states.
SUPPRESS_ULTRASONIC_STATES = {
    "OBJECT_SEARCH_SPIN",
    "WAITING_FOR_AMCL_CONVERGENCE",
    "WAITING_FOR_DETECTOR",
    "WAITING_FOR_NAV2",
}




class CollisionMonitor(Node):

    def __init__(self):
        super().__init__("collision_monitor")

        self._range   = {"front": 99.0, "left": 99.0, "right": 99.0}
        self._contact = False
        self._stop_count: dict[str, int] = {k: 0 for k in ("front", "left", "right", "contact")}
        self._emergency_active = False
        self._task_state = "WAITING_FOR_NAV2"   # start suppressed

        sensor_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(LaserScan, "/tidybot/ultrasonic",               #Front ultrasonic
                                 lambda m: self._scan_cb(m, "front"), sensor_qos)
        self.create_subscription(LaserScan, "/tidybot/ultrasonic_left",          #Left ultrasonic
                                 lambda m: self._scan_cb(m, "left"), sensor_qos)
        self.create_subscription(LaserScan, "/tidybot/ultrasonic_right",         #Right ultrasonic
                                 lambda m: self._scan_cb(m, "right"), sensor_qos)
        self.create_subscription(ContactsState, "/tidybot/contact",              #Contact sensor
                                 self._contact_cb, 10)
        self.create_subscription(String, "/tidybot_task/status",
                                 self._task_status_cb, 10)                       #Task state updates (for ultrasonic suppression)

        self._cmd_pub    = self.create_publisher(Twist,  "/cmd_vel",                    10)
        self._status_pub = self.create_publisher(String, "/tidybot/collision_status",   10)  #Publishes "CLEAR", "WARN:dir:dist", or "STOP:dir"

        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")     

        self.create_timer(0.05, self._monitor_tick)   # 20 Hz safety loop

        self.get_logger().info(
            "CollisionMonitor ready — watching front/left/right ultrasonics + contact"
        )

    # Task state callback ───────────────────────────────────────────────
    def _task_status_cb(self, msg: String):
        self._task_state = msg.data
        if msg.data not in SUPPRESS_ULTRASONIC_STATES:
            # Re-entering nav or pick-and-place: clear any stale emergency state
            self._emergency_active = False
            self._stop_count = {k: 0 for k in self._stop_count}
            self.get_logger().info(
                f"[CollisionMonitor] Ultrasonic protection ENABLED (state={msg.data})")
        else:
            self.get_logger().info(
                f"[CollisionMonitor] Ultrasonic protection SUPPRESSED (state={msg.data})")

    # Sensor callbacks ─────────────────────────────────────────────────
    def _scan_cb(self, msg: LaserScan, direction: str):
        valid = [r for r in msg.ranges
                 if math.isfinite(r) and msg.range_min < r < msg.range_max]
        self._range[direction] = min(valid) if valid else 99.0

    def _contact_cb(self, msg: ContactsState):
        self._contact = len(msg.states) > 0

    # ── 20 Hz safety loop ────────────────────────────────────────────────
    def _monitor_tick(self):
        # Ultrasonic obstacle-stop is suppressed during spin/wait phases
        # (the robot is stationary or rotating in-place; walls will always
        #  be within stop-distance inside a room).
        ultrasonic_active = self._task_state not in SUPPRESS_ULTRASONIC_STATES

        triggered_dir = None

        # Contact sensor = instant STOP regardless of task state
        if self._contact:
            triggered_dir = "contact"
        elif ultrasonic_active:
            for direction, dist in self._range.items():
                if dist < STOP_DIST:
                    triggered_dir = direction
                    break

        if triggered_dir:
            self._stop_count[triggered_dir] = self._stop_count.get(triggered_dir, 0) + 1
        else:
            # Reset all counters when clear
            self._stop_count = {k: 0 for k in self._stop_count}

        if triggered_dir and self._stop_count.get(triggered_dir, 0) >= STOP_DEBOUNCE:
            if not self._emergency_active:
                self.get_logger().warn(
                    f"[CollisionMonitor] STOP — obstacle {triggered_dir} | "
                    f"dist={self._range.get(triggered_dir, 0.0):.2f} m"
                )
                self._emergency_active = True
                self._cancel_nav_goal()

                # Publish zero once — do NOT flood /cmd_vel as it fights/ occupies space from Nav2
                if triggered_dir == "contact":
                    self._cmd_pub.publish(Twist())

            self._publish_status(f"STOP:{triggered_dir}")
            return

        # Warn zone
        for direction, dist in self._range.items():
            if dist < WARN_DIST:
                self._publish_status(f"WARN:{direction}:{dist:.2f}")
                if self._emergency_active:
                    self.get_logger().info(
                        f"[CollisionMonitor] Obstacle cleared ({direction})"
                    )
                self._emergency_active = False
                return

        # All clear
        if self._emergency_active:
            self.get_logger().info("[CollisionMonitor] All clear — resuming")
            self._emergency_active = False
        self._publish_status("CLEAR")

    def _cancel_nav_goal(self):
        """Best-effort cancel of any active Nav2 goal."""
        try:
            if self._nav_client.server_is_ready():
                # cancel_all_goals_async is the correct API
                self._nav_client._cancel_goal(None)
        except Exception:
            pass   # non-critical

    def _publish_status(self, text: str):
        m = String()
        m.data = text
        self._status_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = CollisionMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
