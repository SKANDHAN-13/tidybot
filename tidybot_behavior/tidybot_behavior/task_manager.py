"""
task_manager.py
===============
Top-level task manager – wait for Nav2 + detections, do a fast
in-place AMCL convergence spin, then immediately send GO to
pick_and_place.  No multi-room survey sweep.
"""

import math
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose


class TaskManager(Node):

    def __init__(self):
        super().__init__("task_manager")

        self._objects_seen = False
        self._mission_done = False
        self._mission_log: list[tuple[float, str]] = []

        self._start_pub  = self.create_publisher(String, "/tidybot_task/start",  10)
        self._status_pub = self.create_publisher(String, "/tidybot_task/status", 10)
        self._cmd_pub    = self.create_publisher(Twist,  "/cmd_vel",             10)

        self.create_subscription(String,    "/tidybot_nav/pnp_status", self._pnp_cb, 10)
        self.create_subscription(PoseArray, "/tidybot_detect/confirmed_objects",
                                 self._objects_cb, 10)

        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # Track AMCL convergence via pose covariance
        self._amcl_converged = False
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose",
            self._amcl_cb, 10)

        threading.Thread(target=self._main, daemon=True).start()
        self.get_logger().info("TaskManager started")

    def _pnp_cb(self, msg: String):
        ts = time.time()
        self._mission_log.append((ts, msg.data))
        self.get_logger().info(f"[PnP] {msg.data}")
        if msg.data.startswith("MISSION_COMPLETE"):
            self._mission_done = True

    def _objects_cb(self, msg: PoseArray):
        if msg.poses:
            self._objects_seen = True

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        """AMCL is converged when x-y position covariance drops below threshold."""
        cov_xx = msg.pose.covariance[0]   # x variance
        cov_yy = msg.pose.covariance[7]   # y variance
        if cov_xx < 0.05 and cov_yy < 0.05:
            if not self._amcl_converged:
                self.get_logger().info(
                    f"[TM] AMCL converged (cov_x={cov_xx:.4f}, cov_y={cov_yy:.4f})")
            self._amcl_converged = True

    def _pub(self, text: str):
        m = String(); m.data = text
        self._status_pub.publish(m)
        self.get_logger().info(f"[TM] {text}")

    def _wait_for_localization(self):
        """Wait for AMCL to converge on the pre-built map (max 15 s)."""
        self._pub("WAITING_FOR_AMCL_CONVERGENCE")
        deadline = time.time() + 15.0
        while not self._amcl_converged and time.time() < deadline:
            time.sleep(0.2)
        if self._amcl_converged:
            self.get_logger().info("[TM] AMCL converged")
        else:
            self.get_logger().warn("[TM] AMCL convergence timeout — proceeding anyway")

    def _search_spin(self):
        """Rotate slowly until camera detects at least one object, then stop.
        Gives up after one full 360° rotation if nothing found."""
        if self._objects_seen:
            return   # already have detections, skip
        self._pub("OBJECT_SEARCH_SPIN")
        self.get_logger().info("[TM] No objects visible yet — rotating to search...")
        cmd = Twist()
        cmd.angular.z = 0.4   # rad/s  (slower than AMCL spin for better detection)
        # one full 360° at 0.4 rad/s ≈ 15.7 s
        deadline = time.time() + 16.0
        while time.time() < deadline:
            if self._objects_seen:
                self.get_logger().info("[TM] Object detected — stopping search spin")
                break
            self._cmd_pub.publish(cmd)
            time.sleep(0.1)
        self._cmd_pub.publish(Twist())   # stop
        if not self._objects_seen:
            self.get_logger().warn("[TM] Full 360° search complete, no objects found")

    def _main(self):
        time.sleep(5.0)

        self._pub("WAITING_FOR_NAV2")
        while not self._nav_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().info("Waiting for Nav2…")

        self._pub("WAITING_FOR_DETECTOR")
        deadline = time.time() + 20.0
        while not self._objects_seen and time.time() < deadline:
            time.sleep(0.5)
        if not self._objects_seen:
            self.get_logger().warn("No detections yet — continuing anyway")

        self._wait_for_localization()
        self._search_spin()   # rotates until objects seen, no-op if already seen

        self._pub("STARTING_PICK_AND_PLACE")
        m = String(); m.data = "GO"
        self._start_pub.publish(m)

        deadline = time.time() + 600.0
        while not self._mission_done and time.time() < deadline:
            time.sleep(1.0)

        self._pub("MISSION_SUCCESS" if self._mission_done else "MISSION_TIMEOUT")
        t0 = self._mission_log[0][0] if self._mission_log else time.time()
        for ts, ev in self._mission_log:
            self.get_logger().info(f"  +{ts-t0:6.1f}s  {ev}")


def main(args=None):
    rclpy.init(args=args)
    node = TaskManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
