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

        self.status = "starting"

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
        self.arrival_tolerance = 0.1

        # skriv koordinaterna som (x, y)
        # self.paths = [
        #     [(0.0, 0.0), (0.5, -1.0), (3.5, -1.0), (3.5, 2.0), (0.5, 2.0)],
        #     [(0.5, 1.0), (3.5, 1.0), (3.5, 0.0), (0.0, 0.0), (0.5, -1.0)],
        #     [(1.5, -1.0), (1.5, 2.0), (2.5, 2.0), (2.5, 0.0)],
        # ]

        # bara en liten 0.5x0.5 kvadrat
        self.paths = [
            [(0.0, 0.0), (0.5, 0.0), (0.5, 0.5)],
            [(0.5, 0.5), (0.0, 0.5), (0.0, 0.0)]]

        #self._timer = self.create_timer(2.0, self._tick1)
        self._timer = self.create_timer(2.0, self._tick4)

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
    
    def _publish_next_path(self):
        self.current_idx = (self.current_idx + 1) % len(self.paths) # blir en loop återgår till första pathen tack vare modulo
        self.current_path = self._make_path(self.paths[self.current_idx])
        self.path_pub.publish(self.current_path)
        self.get_logger().info(f'Published path {self.current_idx + 1}/{len(self.paths)}')

    # visar att vi kan skicka path_msgs till controller
    def _tick1(self):
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

    # denna funktion kör exploration i task 4. Avståndscheck är tidsbaserad och path-planner körs när robot är tillräckligt nära slutmålet
    # av nuvarande path
    # rätt mkt felhantering här, hade nog kunnat snyggas till
    def _tick4(self):
        if self._latest_map is None or self._latest_frontier is None:
            self.get_logger().warn("Waiting for map and frontier...")
            return

        if self.status == "starting":
            pose = self.get_robot_pose()
            if pose is None:
                return
            
            self.status = "running"
            current_path = self.plan_path(pose, self._latest_map, self._latest_frontier)
            if current_path:
                self.current_path = current_path
            return

        pose = self.get_robot_pose()
        if pose is None:
            return

        x, y, _ = pose
        goal = self.current_path.poses[-1].pose.position
        dist = math.hypot(goal.x - x, goal.y - y)

        if dist < self.arrival_tolerance:
            self.get_logger().info('Reached goal, publishing next path')
            current_path = self.plan_path(pose, self._latest_map, self._latest_frontier)
            if current_path:
                self.current_path = current_path


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


    # task 3: anti collision

    def inflate_grid(self, map_msg: OccupancyGrid, iterations):
        
        # en ros-grid är bara en lista med grid-värden -> gör om till 2D-array istället
        grid = self.convert2grid(map_msg)
        # skapar en boolean mask -> alltså platser där grid-värdet = 100 kommer ha True
        occupied = grid == 100

        # ett fönster som är 3x3 eftersom 8 celler runt den aktuella ska markeras som occupied
        structure = np.ones((3, 3), dtype=bool)
        # från schipy.ndimage: https://docs.scipy.org/doc/scipy/reference/generated/scipy.ndimage.binary_dilation.html
        inflated = binary_dilation(occupied, structure=structure, iterations=iterations)

        grid[inflated] = 100

        map_msg.data = grid.flatten().tolist()

        return map_msg
    
    # task 4: frontier

    # ---- testade ny grej
    
    def convert2grid(self, occupancy_grid_msg):
        width = occupancy_grid_msg.info.width
        height = occupancy_grid_msg.info.height

        map_grid = np.array(occupancy_grid_msg.data).reshape((height, width))

        return map_grid

    def goal_generation(self, frontier_msg, n_goals):
        # målet med denna är att generera en lista med möjliga mål som vi kör RRT på
        frontier_grid = self.convert2grid(frontier_msg)
        # hitta all koordinater där vi har frontier points
        ys, xs = np.where(frontier_grid == 100)

        if len(xs) == 0:
            self.get_logger().warn("No frontier points available.")
            return []

        indices = np.arange(len(xs))
        # blanda koordinaterna för att sampla random frontier points https://arxiv.org/abs/2104.03724?
        np.random.shuffle(indices)
        selected = indices[:min(n_goals, len(xs))]

        goals = []
        for i in selected:
            x, y = xs[i], ys[i]

            xw = frontier_msg.info.origin.position.x + x * frontier_msg.info.resolution
            yw = frontier_msg.info.origin.position.y + y * frontier_msg.info.resolution
            goals.append((xw, yw))
        
        self.get_logger().info(f"Generated {len(goals)} goals.")

        return goals
    
    def compute_information_gain(self, path, map_msg, sensor_range):
        # vi räknar hur många unknown celler är runt varje delpunkt i en path
        # information gain är hög om man åker till ett ställe där man upptäcker många unknown celler
        grid = self.convert2grid(map_msg)
        origin = map_msg.info.origin
        resolution = map_msg.info.resolution
        width = map_msg.info.width
        height = map_msg.info.height
        radius_cells = int(sensor_range / map_msg.info.resolution)
        discovered = set()

        if path == []:
            return float('inf')

        for xw, yw in path:
            gx = int((xw - origin.position.x) / resolution)
            gy = int((yw - origin.position.y) / resolution)
            
            if gx < 0 or gy < 0 or gx >= width or gy >= height:
                continue

            x_min = max(0, gx - radius_cells)
            x_max = min(width, gx + radius_cells)
            y_min = max(0, gy - radius_cells)
            y_max = min(height, gy + radius_cells)

            window = np.where(grid[y_min:y_max, x_min:x_max] == -1)

            for y, x in zip(window[0], window[1]):
                discovered.add((x + x_min, y + y_min))

        return len(discovered)
    
    def total_cost(self, path_cost, info_gain, path_weight, info_weight):
        # kombinera kostnaden för path och information gain för att välja den bästa path
        # notera att jag har dock inte tune:at dessa vikter men den verkar fungera iaf
        return path_cost * path_weight - info_gain * info_weight

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

        # börja med att inflate all obstacles så vi inte planerar för tajt path (task 3)
        inflated = self.inflate_grid(map_msg, 4)

        # generera en lista med n_goals vi kan planera path till
        goals = self.goal_generation(frontier_msg, n_goals=50)

        if not goals:
            self.get_logger().warn("No goals generated from frontiers.")
            return None

        self.get_logger().info(f"Trying {len(goals)} frontier goals...")

        # håll koll på den bästa pathen
        best_path = []
        best_cost = float('inf')
        fail_count = 0
        
        # kör rrt på all potentiella mål
        for goal in goals:
            root_node = RRTNode(start_x, start_y)
            self.get_logger().warn(f"Starting RRT with goal {goal}")

            planner = RRT(
                node=root_node,
                occupancy_grid=inflated,
                num_iterations=500,
                goal=goal
            )
            planner.run_RRT()
            path, path_cost = planner.extract_best_path()

            info_gain = self.compute_information_gain(path, map_msg, sensor_range=2.0)
            total_cost = self.total_cost(path_cost, info_gain, path_weight=3.0, info_weight=2.0)

            if path == []:
                fail_count += 1

            if total_cost < best_cost:
                best_cost = total_cost
                best_path = path

        self.get_logger().info(f"failed {fail_count}/{len(goals)}")

        if best_path == []:
            return None

        # publicera den bästa path vi hittade
        path_msg = self._make_path(best_path)
        self.path_pub.publish(path_msg)
        self.get_logger().info(f"Published RRT* path with {len(best_path)} points.")

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
