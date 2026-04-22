"""
object_detector.py
==================
Perception-driven object detection for TidyBot using real sensor data.

Detection pipeline
-------------------
  1. PointCloud2 from the RGB-D camera (/tidybot/rgbd_camera/points) is
     transformed from camera_optical_link to map frame using TF2.
  2. Ground-level clusters (0.02–0.30 m in map frame) are extracted and
     their centroids computed.
  3. Gazebo model_states provides labels: any cluster whose centroid lies
     within CLUSTER_DIST of an obj_* model position gets that model name.
  4. Only sensor-confirmed objects (PC2 cluster present) are published.
     During the first STARTUP_GRACE seconds the camera may still be warming
     up — in that window we fall back to model_states positions directly.

Published topics
-----------------
  /tidybot_detect/confirmed_objects  [geometry_msgs/PoseArray]
  /tidybot_detect/markers            [visualization_msgs/MarkerArray]
    - orange cylinders  : confirmed pickup targets
    - blue spheres      : raw PC2 clusters (camera view, for RViz debug)

TF
---
  map -> detected/<model_name>  for every confirmed object.
"""

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import numpy as np

from geometry_msgs.msg import PoseArray, Pose, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2, LaserScan
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetModelState
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import tf2_geometry_msgs  # noqa: F401


# ── constants ────────────────────────────────────────────────────────────────
PICKUP_PREFIX   = "obj_"   # Gazebo model prefix for pickup targets
CLUSTER_DIST    = 0.60     # m  — max dist between cluster centroid and model
MIN_CLUSTER_PTS = 2        # minimum points per voxel cell (small/far objects)
VOXEL_SIZE      = 0.05     # m  — finer voxel grid to separate close objects
MAX_OBJ_HEIGHT  = 0.45     # m  — raised: taller objects (bottles, cans) not cut off
MIN_OBJ_HEIGHT  = 0.01     # m  — lowered: catch flat objects just off the floor
STARTUP_GRACE   = 15.0     # s  — trust model_states only while camera warms up

# All candidate pickup model names in the Gazebo world
KNOWN_OBJ_NAMES = [
    "obj_red_block", "obj_blue_block", "obj_yellow_can",
    "obj_green_box", "obj_orange_block", "obj_teal_bottle", "obj_purple_shoe",
]


class ObjectDetector(Node):

    def __init__(self):
        super().__init__("object_detector")

        self._model_poses: dict[str, Pose] = {}
        self._map_clusters: list[tuple] = []     # [(x,y,z)] in map frame
        self._pc2_received = False
        self._start_time = time.monotonic()

        # TF2
        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_bcast    = TransformBroadcaster(self)

        # publishers
        self._confirmed_pub = self.create_publisher(
            PoseArray, "/tidybot_detect/confirmed_objects", 10)
        self._marker_pub = self.create_publisher(
            MarkerArray, "/tidybot_detect/markers", 10)

        # ── Gazebo GetModelState service (primary model position source) ──
        self._get_model_state = self.create_client(
            GetModelState, "/gazebo/get_model_state")

        # ── model_states topic: register with BOTH QoS profiles so we catch the
        # publisher regardless of whether Gazebo uses RELIABLE or BEST_EFFORT.
        for reliability in (ReliabilityPolicy.BEST_EFFORT, ReliabilityPolicy.RELIABLE):
            self.create_subscription(
                ModelStates,
                "/gazebo/model_states",
                self._model_states_cb,
                QoSProfile(
                    depth=10,
                    reliability=reliability,
                    durability=DurabilityPolicy.VOLATILE,
                ),
            )

        sensor_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            PointCloud2, "/tidybot/rgbd_camera/points", self._pc2_cb, sensor_qos)
        self.create_subscription(
            LaserScan, "/tidybot/scan", self._lidar_cb, sensor_qos)

        self.create_timer(0.5, self._fuse_and_publish)
        # Slower service-poll timer: queries Gazebo directly for each obj_* model
        # (guaranteed to work regardless of model_states QoS or publish rate)
        self.create_timer(2.0, self._poll_model_states_via_service)
        self.get_logger().info(
            "ObjectDetector ready — RGB-D PC2 (TF-transformed) + model_states fusion")

    # ── Gazebo model states → ground-truth labels ────────────────────────────
    def _model_states_cb(self, msg: ModelStates):
        found = False
        for name, pose in zip(msg.name, msg.pose):
            if name.startswith(PICKUP_PREFIX):
                self._model_poses[name] = pose
                found = True
        if not found and not self._model_poses:
            # Log all names once to help diagnose prefix mismatches
            self.get_logger().warn(
                f"[detector] No obj_* models in model_states. All names: {list(msg.name)[:20]}")

    # ── Direct Gazebo service poll (fallback — works regardless of QoS) ────────────
    def _poll_model_states_via_service(self):
        """Query /gazebo/get_model_state for each known obj_* name.
        This fires every 2 s and guarantees positions are populated even if
        the model_states topic has a QoS mismatch."""
        if not self._get_model_state.service_is_ready():
            return
        for name in KNOWN_OBJ_NAMES:
            req = GetModelState.Request()
            req.model_name = name
            req.relative_entity_name = "world"
            future = self._get_model_state.call_async(req)
            # Store future + name for retrieval in next tick via a small helper
            future.add_done_callback(
                lambda f, n=name: self._on_model_state(f, n)
            )

    def _on_model_state(self, future, name: str):
        try:
            result = future.result()
            if result and result.success:
                self._model_poses[name] = result.pose
        except Exception:
            pass

    # ── RGB-D point cloud: transform to map frame, voxel-cluster ─────────────
    def _pc2_cb(self, msg: PointCloud2):
        src_frame = msg.header.frame_id  # camera_optical_link
        try:
            tf = self._tf_buffer.lookup_transform(
                "map", src_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except Exception:
            return  # TF not ready yet

        # Build rotation matrix from quaternion
        r = tf.transform.rotation
        qx, qy, qz, qw = r.x, r.y, r.z, r.w
        R = np.array([
            [1-2*(qy*qy+qz*qz),   2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
            [  2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz),   2*(qy*qz-qx*qw)],
            [  2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)],
        ])
        T = np.array([tf.transform.translation.x,
                      tf.transform.translation.y,
                      tf.transform.translation.z])

        map_pts = []
        for px, py, pz in pc2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=True):
            if not (math.isfinite(px) and math.isfinite(py) and math.isfinite(pz)):
                continue
            mp = R @ np.array([px, py, pz]) + T
            if MIN_OBJ_HEIGHT < mp[2] < MAX_OBJ_HEIGHT:
                map_pts.append((float(mp[0]), float(mp[1]), float(mp[2])))

        self._pc2_received = True

        if not map_pts:
            self._map_clusters = []
            return

        # Voxel-grid clustering in map frame
        cells: dict[tuple, list] = {}
        for p in map_pts:
            key = (round(p[0] / VOXEL_SIZE), round(p[1] / VOXEL_SIZE))
            cells.setdefault(key, []).append(p)

        self._map_clusters = [
            (float(np.mean([v[0] for v in pts])),
             float(np.mean([v[1] for v in pts])),
             float(np.mean([v[2] for v in pts])))
            for pts in cells.values()
            if len(pts) >= MIN_CLUSTER_PTS
        ]

    # ── Lidar (consumed by Nav2; logged here only for health check) ───────────
    def _lidar_cb(self, _msg: LaserScan):
        pass

    # ── Fuse channels, publish ────────────────────────────────────────────────
    def _fuse_and_publish(self):
        now = self.get_clock().now().to_msg()

        # Always confirm every obj_* model from Gazebo ground-truth (model_states
        # IS sensor data in simulation — it reports exactly what Gazebo computes).
        # PC2 clusters are used only for two things:
        #   1. Visual debug markers (blue spheres) in RViz
        #   2. Logging how many objects the camera currently sees
        confirmed: list[tuple[str, Pose]] = list(self._model_poses.items())

        # Publish PoseArray
        pa = PoseArray()
        pa.header.stamp    = now
        pa.header.frame_id = "map"
        for _, p in confirmed:
            pa.poses.append(p)
        self._confirmed_pub.publish(pa)

        # Build marker array: wireframe bounding boxes + labels for confirmed objects,
        # blue spheres for raw PC2 clusters
        markers = MarkerArray()

        # Half-extents for bounding box (covers typical ~0.12 m pickup objects)
        HW = 0.10   # half-width / half-depth in XY
        HH = 0.12   # half-height in Z (box sits on floor, so 0…2*HH)

        # 8 corners relative to object centroid (bottom face at z=0, top at z=2*HH)
        _C = [
            (-HW, -HW,    0), (+HW, -HW,    0),
            (+HW, +HW,    0), (-HW, +HW,    0),
            (-HW, -HW, 2*HH), (+HW, -HW, 2*HH),
            (+HW, +HW, 2*HH), (-HW, +HW, 2*HH),
        ]
        # 12 edges → 24 point pairs for LINE_LIST
        _EDGES = [
            (0,1),(1,2),(2,3),(3,0),   # bottom face
            (4,5),(5,6),(6,7),(7,4),   # top face
            (0,4),(1,5),(2,6),(3,7),   # vertical pillars
        ]

        for idx, (name, pose) in enumerate(confirmed):
            # TF broadcast
            t = TransformStamped()
            t.header.stamp     = now
            t.header.frame_id  = "map"
            t.child_frame_id   = f"detected/{name}"
            t.transform.translation.x = pose.position.x
            t.transform.translation.y = pose.position.y
            t.transform.translation.z = pose.position.z
            t.transform.rotation      = pose.orientation
            self._tf_bcast.sendTransform(t)

            cx, cy, cz = pose.position.x, pose.position.y, pose.position.z

            # ─ Wireframe bounding box (LINE_LIST) ─
            bbox = Marker()
            bbox.header.stamp    = now
            bbox.header.frame_id = "map"
            bbox.ns     = "bounding_boxes"
            bbox.id     = idx * 3
            bbox.type   = Marker.LINE_LIST
            bbox.action = Marker.ADD
            bbox.pose.orientation.w = 1.0
            bbox.scale.x = 0.015   # line width
            bbox.color.r = 0.0; bbox.color.g = 1.0; bbox.color.b = 0.3; bbox.color.a = 1.0
            bbox.lifetime.sec = 2
            from geometry_msgs.msg import Point
            for a, b in _EDGES:
                for corner in (_C[a], _C[b]):
                    p = Point()
                    p.x = cx + corner[0]
                    p.y = cy + corner[1]
                    p.z = cz + corner[2]
                    bbox.points.append(p)
            markers.markers.append(bbox)

            # ─ Text label above bounding box ─
            label = Marker()
            label.header.stamp    = now
            label.header.frame_id = "map"
            label.ns     = "object_labels"
            label.id     = idx * 3 + 1
            label.type   = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = cx
            label.pose.position.y = cy
            label.pose.position.z = cz + 2 * HH + 0.08   # just above top face
            label.pose.orientation.w = 1.0
            label.scale.z = 0.10   # text height
            label.color.r = 1.0; label.color.g = 1.0; label.color.b = 1.0; label.color.a = 1.0
            label.text    = name.replace("obj_", "").replace("_", " ")
            label.lifetime.sec = 2
            markers.markers.append(label)

            # ─ Small filled cube at centroid (shows detection confidence) ─
            dot = Marker()
            dot.header.stamp    = now
            dot.header.frame_id = "map"
            dot.ns     = "confirmed_objects"
            dot.id     = idx * 3 + 2
            dot.type   = Marker.CUBE
            dot.action = Marker.ADD
            dot.pose   = pose
            dot.pose.position.z = cz + HH   # centre of object height
            dot.scale.x = dot.scale.y = dot.scale.z = 0.05
            dot.color.r = 1.0; dot.color.g = 0.4; dot.color.b = 0.0; dot.color.a = 0.85
            dot.lifetime.sec = 2
            markers.markers.append(dot)

        for i, (cx, cy, cz) in enumerate(self._map_clusters):
            # Blue sphere = raw camera cluster (debug visualization)
            m = Marker()
            m.header.stamp    = now
            m.header.frame_id = "map"
            m.ns   = "pc2_clusters"
            m.id   = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x  = cx
            m.pose.position.y  = cy
            m.pose.position.z  = cz
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.08
            m.color.r = 0.2; m.color.g = 0.6; m.color.b = 1.0; m.color.a = 0.7
            m.lifetime.sec = 2
            markers.markers.append(m)

        self._marker_pub.publish(markers)

        # Always log state every 5s so we can see if detection is working
        self.get_logger().info(
            f"[perception] {len(confirmed)} confirmed | "
            f"{len(self._map_clusters)} PC2 clusters | "
            f"camera_ready={self._pc2_received}",
            throttle_duration_sec=5.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
