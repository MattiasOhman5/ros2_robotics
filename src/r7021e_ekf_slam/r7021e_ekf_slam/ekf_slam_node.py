# ------------------------------------------------------------------------
# R7021E Lab 4: EKF-SLAM with 2D LiDAR and odometry
# ------------------------------------------------------------------------
# ekf_slam_impl_stub.py
# EKF-SLAM implementation with point landmarks (range-bearing sensing).
# See ekf_slam_node.py for the ROS2 node wrapper.
# ------------------------------------------------------------------------

import numpy as np
from .utils import*
import tf2_ros, math, rclpy
from typing import Optional, List, Tuple
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import TransformStamped, PoseStamped
from .landmark_extraction import extract_landmarks_from_scan_raster
from .ekf_slam_impl import EKFSLAM  # choose solution vs student 
import time
import csv
import os
import glob


class EKFSLAMNode(Node):
    """
    ROS2 node wrapper for EKF-SLAM with 2D LiDAR and odometry.
    See ekf_slam_impl.py for the EKF-SLAM implementation.
    """

    def __init__(self):
        super().__init__('ekf_slam_node', automatically_declare_parameters_from_overrides=True)
        # --- Topic Parameters ---
        self.scan_topic = self.get_parameter('scan_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.laser_frame = self.get_parameter('laser_frame').value
        # --- Raster feature extraction parameters ---
        self.raster_res = self.get_parameter('raster_res').value
        self.raster_extent = self.get_parameter('raster_extent').value
        self.raster_min_px = self.get_parameter('raster_min_px').value
        self.raster_max_px = self.get_parameter('raster_max_px').value
        self.raster_max_aspect = self.get_parameter('raster_max_aspect').value
        # --- EKF-SLAM parameters ---
        self.Q_diag = self.get_parameter('Q_diag').value
        self.R_diag = self.get_parameter('R_diag').value
        self.publish_static_tf = self.get_parameter('publish_static_laser_tf').value
        # --- Publishers ---
        self.odom_pub = self.create_publisher(Odometry, 'ekf_slam/odom', 10)
        self.map_pub = self.create_publisher(PointCloud2, 'ekf_slam/landmarks', 10)
        self.path_pub = self.create_publisher(Path, 'ekf_slam/path', 10)
        # --- Path message and max size ---
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.map_frame
        self.path_max_size = int(self.get_parameter('path_max_size').value)
        # --- TF ---
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # Optionally publish a static base->laser transform (only if your robot doesn't publish one)
        if bool(self.publish_static_tf):
            self._publish_static_base_to_laser()
        # --- Subscriptions ---
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_cb,
                                                qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_cb,
                                                qos_profile_sensor_data)
        # --- Filter and book-keeping ---
        self.ekf = EKFSLAM(Q_diag=self.Q_diag, R_diag=self.R_diag)
        # Odometry states
        self._last_odom_stamp: Optional[Time] = None
        self._last_odom_pose = None         # (x, y, yaw) from the *latest* odom msg
        self._odom_ref_pose = None          # (x, y, yaw) at the *last* filter step (predict/update boundary)
        self._ever_predicted = False
        # Prediction-only timer to keep pose moving if scans are sparse
        rate = float(self.get_parameter('prediction_rate').value)
        self.pred_timer = self.create_timer(1.0 / max(rate, 1.0), self._prediction_timer_cb)
        # --- Info ---
        self.get_logger().info(
            f"EKF-SLAM node started. Subscribed to {self.scan_topic} and {self.odom_topic}. "
            f"Frames: map='{self.map_frame}', base='{self.base_frame}', laser='{self.laser_frame}'"
        )
        # --- Info parameters ---
        self.get_logger().info(f"Raster feature extraction: res={self.raster_res}m, extent={self.raster_extent}m, "
                                f"min_px={self.raster_min_px}, max_px={self.raster_max_px}, "
                                f"max_aspect={self.raster_max_aspect}")
        self.get_logger().info(f"EKF-SLAM: Q_diag={self.Q_diag}, R_diag={self.R_diag}")
        self.get_logger().info(f"EKF-SLAM: path_max_size={self.path_max_size}")
        self.get_logger().info(f"EKF-SLAM: prediction_rate={rate} Hz")

        # log stuff:

        # --- Logging setup (timing and state size) ---
        log_dir = "/home/mattias/ros2_robotics/src/r7021e_ekf_slam/datalogs"
        os.makedirs(log_dir, exist_ok=True)

        # Determine next available log number
        existing_logs = sorted(glob.glob(os.path.join(log_dir, "log_file_*.csv")))
        next_index = 1
        if existing_logs:
            last = os.path.basename(existing_logs[-1])
            try:
                last_num = int(last.replace("log_file_", "").replace(".csv", ""))
                next_index = last_num + 1
            except ValueError:
                pass

        self.log_path = os.path.join(log_dir, f"log_file_{next_index}.csv")

        # Open CSV and write header
        self.log_file = open(self.log_path, "w", newline="")
        self.csv_writer = csv.writer(self.log_file)

        # Retrieve current filter parameters
        Q_vals = self.get_parameter("Q_diag").value
        R_vals = self.get_parameter("R_diag").value

        # Write header (with parameters)
        self.csv_writer.writerow([
            f"# Q_diag={Q_vals}, R_diag={R_vals}"
        ])
        self.csv_writer.writerow(["timestamp", "state_size", "elapsed"])
        self.get_logger().info(f"Logging EKF timing to {self.log_path}")
        
    # --- Odometry: just store the latest odom, apply on scan or timer ---
    def odom_cb(self, msg: Odometry):
        stamp = Time.from_msg(msg.header.stamp)
        x, y, yaw = pose_msg_to_xyyaw(msg)

        self._last_odom_pose = (x, y, yaw)
        self._last_odom_stamp = stamp

        # Initialize reference on first odom
        if self._odom_ref_pose is None:
            self._odom_ref_pose = (x, y, yaw)
        return
    
    # --- Scan: extract features, predict from odom, update ---
    def scan_cb(self, msg: LaserScan):
        if self._last_odom_pose is None:
            # We need odom to interpret motion before first update.
            self.get_logger().warn(
                "Scan received before any odom. Skipping this scan.",
                throttle_duration_sec=5.0
            )
            return

        start_time = time.perf_counter()

        # Flush all pending odometry into a single predict step
        self._apply_pending_odom_predict()

        # Feature extraction in the laser frame
        z_list = extract_landmarks_from_scan_raster(
            msg,
            res=float(self.get_parameter('raster_res').value),
            extent_m=float(self.get_parameter('raster_extent').value),
            min_px=int(self.get_parameter('raster_min_px').value),
            max_px=int(self.get_parameter('raster_max_px').value),
            max_aspect=float(self.get_parameter('raster_max_aspect').value),
        )

        # Transform features to base_frame (range-bearing expressed at base)
        z_list_base = self._features_to_base_frame(z_list, msg)

        # Measurement update (if we have features)
        if z_list_base:
            self.ekf.update_with_scan_features(z_list_base)

        # Logging
        end_time = time.perf_counter()
        elapsed = (end_time - start_time)
        state_size = self.ekf.mu.shape[0] // 2
        

        self.csv_writer.writerow([time.time(), state_size, elapsed])
        self.log_file.flush()


        # Publish
        stamp = Time.from_msg(msg.header.stamp)
        self.publish_odom(stamp)
        self.publish_tf(stamp)
        self.publish_landmark_cloud(stamp)
        self.append_and_publish_path(stamp)
        return

    # --- Prediction-only timer callback ---
    def _prediction_timer_cb(self):
        """
        If scans are sparse, keep applying prediction-only steps from pending odometry.
        """
        if self._last_odom_pose is None or self._odom_ref_pose is None:
            return

        changed = self._apply_pending_odom_predict()
        if changed:
            # Use last odom stamp if available; otherwise now()
            stamp = self._last_odom_stamp if self._last_odom_stamp is not None else self.get_clock().now()
            self.publish_odom(stamp)
            self.publish_tf(stamp)
            self.append_and_publish_path(stamp)
        return

    # --- Helpers ---
    def _apply_pending_odom_predict(self) -> bool:
        """
        Compare last odom pose with the odom pose at the last filter boundary (_odom_ref_pose).
        If there is motion, convert to (drot1, dtrans, drot2) and call ekf.predict_odometry().
        Update _odom_ref_pose. Returns True iff a predict was applied.
        """
        if self._last_odom_pose is None or self._odom_ref_pose is None:
            return False

        x0, y0, th0 = self._odom_ref_pose
        x, y, th = self._last_odom_pose

        dx = x - x0
        dy = y - y0
        dtheta = angle_normalize(th - th0)

        dtrans = float(math.hypot(dx, dy))
        if dtrans < 1e-9 and abs(dtheta) < 1e-9:
            return False

        drot1 = angle_normalize(math.atan2(dy, dx) - th0) if dtrans > 1e-9 else 0.0
        drot2 = angle_normalize(dtheta - drot1)

        self.ekf.predict_odometry(drot1, dtrans, drot2)
        self._odom_ref_pose = (x, y, th)
        self._ever_predicted = True
        return True

    def _features_to_base_frame(self, z_list: List, scan_msg: LaserScan) -> List[Tuple[float, float]]:
        """
        Convert features returned by the extractor (in laser frame) to (range, bearing) in base_frame.
        We:
            - interpret tuples as (range, bearing) in laser frame,
            - convert to XY in laser,
            - transform with TF (base <- laser),
            - convert back to (range, bearing) in base frame.
        If TF is unavailable, we assume laser == base (warn once).
        """
        if not z_list:
            return []

        # Determine transform base <- laser at scan time
        need_tf = (scan_msg.header.frame_id != self.base_frame)
        T_bl = None
        if need_tf:
            try:
                tf = self.tf_buffer.lookup_transform(
                    target_frame=self.base_frame,
                    source_frame=scan_msg.header.frame_id,
                    time=scan_msg.header.stamp,
                    timeout=rclpy.duration.Duration(seconds=0.05)
                )
                tx = tf.transform.translation.x
                ty = tf.transform.translation.y
                # roll/pitch are ignored in 2D; use yaw
                q = tf.transform.rotation
                yaw = yaw_from_quaternion(q)
                T_bl = (tx, ty, yaw)
            except Exception as e:
                self.get_logger().warn(
                    5.0,
                    f"TF lookup {self.base_frame}<-{scan_msg.header.frame_id} failed ({e}). "
                    f"Assuming identical frames for update."
                )

        out: List[Tuple[float, float]] = []

        for z in z_list:
            # Accept (range, bearing) tuples; ignore anything else
            if isinstance(z, (tuple, list)) and len(z) >= 2:
                r, a = float(z[0]), float(z[1])
                # laser XY
                lx = r * math.cos(a)
                ly = r * math.sin(a)

                if T_bl is not None:
                    tx, ty, yaw = T_bl
                    cb = math.cos(yaw)
                    sb = math.sin(yaw)
                    bx = cb * lx - sb * ly + tx
                    by = sb * lx + cb * ly + ty
                else:
                    bx, by = lx, ly

                rb = math.hypot(bx, by)
                ab = math.atan2(by, bx)
                out.append((rb, angle_normalize(ab)))
            else:
                # If extractor returns something else (e.g. XY), silently skip unsupported entries.
                continue

        return out

    # -- Publishing ---
    def publish_odom(self, stamp: Time):
        msg = Odometry()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.map_frame
        msg.child_frame_id = self.base_frame

        x, y, th = float(self.ekf.mu[0]), float(self.ekf.mu[1]), float(self.ekf.mu[2])
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = yaw_to_quaternion(th)

        # 6x6 covariance (fill planar parts)
        cov = np.zeros((6, 6), dtype=float)
        cov[0:2, 0:2] = self.ekf.Sigma[0:2, 0:2]
        cov[5, 5] = self.ekf.Sigma[2, 2]
        msg.pose.covariance = cov.flatten().tolist()

        self.odom_pub.publish(msg)

    def publish_tf(self, stamp: Time):
        x, y, th = float(self.ekf.mu[0]), float(self.ekf.mu[1]), float(self.ekf.mu[2])
        t = TransformStamped()
        t.header.stamp = stamp.to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation = yaw_to_quaternion(th)
        self.tf_broadcaster.sendTransform(t)

    def publish_landmark_cloud(self, stamp: Time):
        pts = self.get_landmarks_xy()  # (N,2) array in map frame
        cloud = points_xy_to_pointcloud2(pts, frame_id=self.map_frame, stamp=stamp)
        self.map_pub.publish(cloud)

    def append_and_publish_path(self, stamp: Time):
        ps = PoseStamped()
        ps.header.frame_id = self.map_frame
        ps.header.stamp = stamp.to_msg()
        x, y, th = float(self.ekf.mu[0]), float(self.ekf.mu[1]), float(self.ekf.mu[2])
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = 0.0
        ps.pose.orientation = yaw_to_quaternion(th)

        # Append and enforce max size if requested
        self.path_msg.header.stamp = ps.header.stamp
        self.path_msg.poses.append(ps)
        if self.path_max_size > 0 and len(self.path_msg.poses) > self.path_max_size:
            # drop oldest
            self.path_msg.poses = self.path_msg.poses[-self.path_max_size:]

        self.path_pub.publish(self.path_msg)

    def get_landmarks_xy(self) -> np.ndarray:
        nL = (self.ekf.mu.shape[0] - 3) // 2
        if nL <= 0:
            return np.zeros((0, 2), dtype=float)
        return self.ekf.mu[3:].reshape(-1, 2).astype(float)

    # --- Static TF ---
    def _publish_static_base_to_laser(self):
        """
        Optional static TF publisher base_frame -> laser_frame.
        Prefer publishing this via robot_state_publisher, but this helps for quick testing.
        """
        tx = float(self.get_parameter('laser_tx').value)
        ty = float(self.get_parameter('laser_ty').value)
        tz = float(self.get_parameter('laser_tz').value)
        rr = float(self.get_parameter('laser_roll').value)
        rp = float(self.get_parameter('laser_pitch').value)
        ry = float(self.get_parameter('laser_yaw').value)

        # Compose quaternion from roll/pitch/yaw (XYZ order)
        cr = math.cos(rr * 0.5); sr = math.sin(rr * 0.5)
        cp = math.cos(rp * 0.5); sp = math.sin(rp * 0.5)
        cy = math.cos(ry * 0.5); sy = math.sin(ry * 0.5)

        # Rz * Ry * Rx (ROS standard)
        qw = cy*cp*cr + sy*sp*sr
        qx = cy*cp*sr - sy*sp*cr
        qy = cy*sp*cr + sy*cp*sr
        qz = sy*cp*cr - cy*sp*sr

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame
        t.child_frame_id = self.laser_frame
        t.transform.translation.x = tx
        t.transform.translation.y = ty
        t.transform.translation.z = tz
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.static_broadcaster.sendTransform(t)
        self.get_logger().info(
            f"Published static TF {self.base_frame} -> {self.laser_frame} "
            f"at (xyz)={(tx,ty,tz)}, rpy={(rr,rp,ry)}"
        )


def main():
    rclpy.init()
    node = EKFSLAMNode()
    rclpy.spin(node)
    if hasattr(node, "log_file") and not node.log_file.closed:
            node.log_file.close()
    node.destroy_node()
    rclpy.shutdown()
