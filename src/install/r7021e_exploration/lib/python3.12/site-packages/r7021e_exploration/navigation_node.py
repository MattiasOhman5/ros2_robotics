#!/usr/bin/env python3

import math
from typing import Optional, Tuple, List

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path, OccupancyGrid
from builtin_interfaces.msg import Time as TimeMsg
from rclpy.time import Time

import tf2_ros

import numpy as np
from r7021e_exploration.rrt import RRT, Node as RRTNode
from scipy.ndimage import binary_dilation
from sklearn.cluster import DBSCAN
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


def quat_to_yaw(q: Quaternion) -> float:
    """Extract planar yaw (rad) from a Quaternion."""
    x, y, z, w = q.x, q.y, q.z, q.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convert planar yaw (rad) to Quaternion."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class PathPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__('path_planner_node')

        # Parameters
        self.declare_parameter('map_topic', 'map')
        #self.declare_parameter('frontier_topic', 'frontier')
        self.declare_parameter('frontier_topic', 'frontiers')
        self.declare_parameter('path_topic', 'path')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('tf_timeout_sec', 0.5)

        map_topic: str = self.get_parameter('map_topic').get_parameter_value().string_value
        frontier_topic: str = self.get_parameter('frontier_topic').get_parameter_value().string_value
        path_topic: str = self.get_parameter('path_topic').get_parameter_value().string_value
        self.global_frame: str = self.get_parameter('global_frame').get_parameter_value().string_value
        self.base_frame: str = self.get_parameter('base_frame').get_parameter_value().string_value
        self.tf_timeout = Duration(seconds=self.get_parameter('tf_timeout_sec').get_parameter_value().double_value)

        default_qos = QoSProfile(depth=10)

        # Publishers (add path publisher here)
        self.path_pub =  self.create_publisher(Path, path_topic, default_qos)

        self.marker_pub = self.create_publisher(Marker, 'frontier_markers', 10)

        # Subscribers
        self.map_sub = self.create_subscription(OccupancyGrid, map_topic, self._on_map, default_qos)
        self.frontier_sub = self.create_subscription(OccupancyGrid, frontier_topic, self._on_frontier, default_qos)

        # TF
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # State
        self._latest_map: Optional[OccupancyGrid] = None
        self._latest_frontier: Optional[OccupancyGrid] = None

        # my changes (task 1)

        self.current_idx = -1
        self.current_path = None
        self.arrival_tolerance = 0.05

        self.paths = [
            [(0.0, 0.0), (0.5, -1.0), (3.5, -1.0), (3.5, 2.0), (0.5, 2.0)],
            [(0.5, 1.0), (3.5, 1.0), (3.5, 0.0), (0.0, 0.0), (0.5, -1.0)],
            [(1.5, -1.0), (1.5, 2.0), (2.5, 2.0), (2.5, 0.0)],
        ]

        #self._timer = self.create_timer(0.1, self._tick)
        self._timer = self.create_timer(2.0, self._on_timer)

        self.get_logger().info('PathPlannerNode initialized.')

    # ----------------- task 1 stuff -----------------

    def _make_path(self, setpoints):
        path = Path()
        path.header.frame_id = self.global_frame
        path.header.stamp = self.get_clock().now().to_msg()

        for x, y in setpoints:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation = yaw_to_quaternion(0.0)
            path.poses.append(ps)

        return path
    
    def _publish_next_path(self) -> None:
        self.current_idx = (self.current_idx + 1) % len(self.paths)
        self.current_path = self._make_path(self.paths[self.current_idx])
        self.path_pub.publish(self.current_path)
        self.get_logger().info(f'Published path {self.current_idx + 1}/{len(self.paths)}')


    def _tick(self) -> None:
        if self.current_path is None:
            pose = self.get_robot_pose()
            if pose is None:
                return
            self._publish_next_path()
            return

        pose = self.get_robot_pose()
        if pose is None:
            return

        x, y, _ = pose
        goal = self.current_path.poses[-1].pose.position
        dist = math.hypot(goal.x - x, goal.y - y)

        if dist < self.arrival_tolerance:
            self.get_logger().info('Reached goal, publishing next path')
            self._publish_next_path()


    # ----------------- TF Helper -----------------

    def get_robot_pose(self, target_frame: Optional[str] = None, source_frame: Optional[str] = None
                       ) -> Optional[Tuple[float, float, float]]:
        """
        Lookup TF to get robot pose (x, y, yaw) in target_frame.
        Returns None if not available within timeout.
        """
        tgt = target_frame or self.global_frame
        src = source_frame or self.base_frame

        try:
            transform = self.tf_buffer.lookup_transform(
                tgt, src, Time(), timeout=self.tf_timeout
            )
        except Exception as e:
            self.get_logger().warn(f'TF lookup {tgt} <- {src} failed: {e}')
            return None

        t = transform.transform.translation
        r = transform.transform.rotation
        yaw = quat_to_yaw(r)

        return (t.x, t.y, yaw)

    # ----------------- Callbacks -----------------

    def _on_frontier(self, msg: OccupancyGrid) -> None:
        """Store the latest frontier map (could be used by your planner)."""
        self._latest_frontier = msg

    def _on_map(self, msg: OccupancyGrid) -> None:
        """Trigger planning when a new map arrives."""
        self._latest_map = msg

        print(msg)

        pose = self.get_robot_pose()
        if pose is None:
            return

        x, y, yaw = pose
        # path = self.plan_path((x, y, yaw),
        #                       msg,
        #                       self._latest_frontier)

        # if path is None:
        #     return
        
    def _on_timer(self):
        if self._latest_map is None or self._latest_frontier is None:
            self.get_logger().warn("Map or frontier not yet received, skipping planning.")
            return

        pose = self.get_robot_pose()
        if pose is None:
            return

        # # Optional: skip if robot already near goal
        # if self.last_goal is not None:
        #     gx, gy = self.last_goal
        #     rx, ry, _ = pose
        #     if math.hypot(gx - rx, gy - ry) < 0.2:
        #         return  # already close enough to goal

        # Plan new path
        path = self.plan_path(pose, self._latest_map, self._latest_frontier)
        #if path is not None:
        #    self.last_goal = path.poses[-1].pose.position


    # task 3: anti collision

    def inflate_grid(self, map_msg: OccupancyGrid, iterations):
        width = map_msg.info.width
        height = map_msg.info.height

        grid = np.array(map_msg.data).reshape((height, width))

        occupied = grid == 100

        structure = np.ones((3, 3), dtype=bool)
        inflated = binary_dilation(occupied, structure=structure, iterations=iterations)

        grid[inflated] = 100

        map_msg.data = grid.flatten().tolist()

        # inflated = grid.copy()
        # for _ in range(iterations):
        #     temp = inflated.copy()
        #     for y in range(height):
        #         for x in range(width):
        #             if inflated[y, x] == 100:
        #                 for dy in [-1, 0, 1]:
        #                     for dx in [-1, 0, 1]:
        #                         ny = y + dy
        #                         nx = x + dx
        #                         if 0 <= ny < height and 0 <= nx < width:
        #                             temp[ny, nx] = 100
        #     inflated = temp.copy()

        # map_msg.data = inflated.flatten().tolist()
        return map_msg
    
    # task 4: frontier

    def convert_frontier(self, frontier_msg):
        width = frontier_msg.info.width
        height = frontier_msg.info.height
        res = frontier_msg.info.resolution
        origin = frontier_msg.info.origin

        data = np.array(frontier_msg.data).reshape((height, width))
        ys, xs = np.where(data > 0)

        x_world = origin.position.x + xs * res
        y_world = origin.position.y + ys * res

        return np.column_stack((x_world, y_world))
    
    def cluster_frontier(self, frontier_points, eps, min_samples):
        if len(frontier_points) == 0:
            return []
        
        clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(frontier_points)
        labels = clustering.labels_

        clusters = []
        for label in set(labels):
            if label == -1:
                continue
            cluster = frontier_points[labels == label]
            clusters.append(cluster)

        return clusters
    
    def select_frontier(self, frontier_msg, map_msg, window_size, max_candidates):
        map_h = map_msg.info.height
        map_w = map_msg.info.width
        map_res = map_msg.info.resolution
        map_origin = map_msg.info.origin.position

        f_h = frontier_msg.info.height
        f_w = frontier_msg.info.width

        map_grid = np.array(map_msg.data).reshape((map_h, map_w))
        frontier_grid = np.array(frontier_msg.data).reshape((f_h, f_w))

        half_win = window_size // 2
        candidates = []

        ys, xs = np.where(frontier_grid == 100)
        for y, x in zip(ys, xs):
            # Skip if cell is occupied
            if map_grid[y, x] == 100:
                continue

            y_min = max(0, y - half_win)
            y_max = min(map_h, y + half_win + 1)
            x_min = max(0, x - half_win)
            x_max = min(map_w, x + half_win + 1)

            window = map_grid[y_min:y_max, x_min:x_max]
            unknown_count = np.sum(window == -1)

            if unknown_count > 0:
                x_world = map_origin.x + x * map_res
                y_world = map_origin.y + y * map_res
                candidates.append((unknown_count, (x_world, y_world)))

        if not candidates:
            self.get_logger().warn("No valid frontier candidates found.")
            return []

        # Sort candidates by unknown_count (descending) without lambda
        for i in range(len(candidates)):
            for j in range(i + 1, len(candidates)):
                if candidates[j][0] > candidates[i][0]:
                    candidates[i], candidates[j] = candidates[j], candidates[i]

        top_candidates = []
        count = min(max_candidates, len(candidates))
        for i in range(count):
            top_candidates.append(candidates[i][1])

        self.get_logger().info(
            f"Found {len(top_candidates)} promising frontier goals."
        )

        return top_candidates
    
    def select_frontier_goal(self, clusters, robot_pose):
        if not clusters:
            return None

        rx, ry, _ = robot_pose

        max_dist = -1.0
        best_cluster = None

        for cluster in clusters:
            cx, cy = np.mean(cluster, axis=0)
            dist = math.hypot(cx - rx, cy - ry)
            if dist > max_dist:
                max_dist = dist
                best_cluster = cluster

        if best_cluster is None:
            return None

        centroid = np.mean(best_cluster, axis=0)
        return (float(centroid[0]), float(centroid[1]))
    
    # visualization

    def publish_frontier_marker(self, goal):
        if goal is None:
            return

        marker = Marker()
        marker.header.frame_id = self.global_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "goal_marker"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.2  # point size
        marker.scale.y = 0.2
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        p = Point()
        p.x = goal[0]
        p.y = goal[1]
        p.z = 0.0
        marker.points.append(p)

        marker.lifetime.sec = 0  # persistent
        self.marker_pub.publish(marker)
        self.get_logger().info(f"Published frontier marker at ({goal[0]:.2f}, {goal[1]:.2f})")
    # ----------------- Planning -----------------

    def plan_path(self,
                  start: Tuple[float, float, float],
                  map_msg: OccupancyGrid,
                  frontier_msg: Optional[OccupancyGrid]):
        """
        Use this function to plan the path
        """
        start_x, start_y, _ = start

        pose = self.get_robot_pose()

        if frontier_msg is None:
            self.get_logger().warn("Frontier message not yet received, skipping planning.")
            return None

        #frontier_points = self.convert_frontier(frontier_msg)
        #clusters = self.cluster_frontier(frontier_points, eps=0.3, min_samples=3)
        #goal = self.select_frontier_goal(clusters, pose)

        goal_candidates = self.select_frontier(frontier_msg, map_msg, 5, 20)

        #self.publish_frontier_marker(goal)

        inflated = self.inflate_grid(map_msg, 3)

        root_node = RRTNode(start_x, start_y)
        #planner = RRT(node=root_node, occupancy_grid=inflated, num_iterations=8000, goal=goal)

        path = []
        for goal in goal_candidates:
            self.get_logger().warn(f"Starting RRT with {goal}")
            planner = RRT(node=root_node, occupancy_grid=inflated, num_iterations=2000, goal=goal)
            planner.run_RRT()
            path = planner.extract_best_path()
            self.get_logger().warn(f"Finishing RRT")
            if path != []:
                break



        path_msg = self._make_path(path)
        self.path_pub.publish(path_msg)
        self.get_logger().info(f"Published RRT* path with {len(path)} points.")

        return path_msg


def main() -> None:
    rclpy.init()
    node = PathPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
