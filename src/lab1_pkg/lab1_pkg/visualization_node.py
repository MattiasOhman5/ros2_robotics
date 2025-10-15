import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration as DurationD

class TrajectoryVisualizer(Node):
    def __init__(self):
        super().__init__('visual_node')

        self.marker_pub = self.create_publisher(Marker, '/trajectory_markers', 10)
        self.global_frame = 'odom'

        self.trajectory = [
            (0.5, -1.0),
            (0.0, -0.5),
            (0.5, 0.0),
            (1.0, 0.5),
            (0.5, 1.0),
            (0.0, 0.5),
            (0.5, 0.0),
            (1.0, -0.5),
            (0.5, -1.0)

        ]

        # Call periodically so we can see logging
        self.timer = self.create_timer(1.0, self.plot_trajectory)
        self.get_logger().info('Trajectory Visualizer started and running.')

    def plot_trajectory(self):
        self.get_logger().info('Publishing trajectory markers...')
        now = self.get_clock().now().to_msg()

        line_marker = Marker()
        line_marker.header.frame_id = self.global_frame
        line_marker.header.stamp = now
        line_marker.ns = "trajectory_lines"
        line_marker.id = 0
        line_marker.type = Marker.LINE_LIST
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.05
        line_marker.color.r = 0.0
        line_marker.color.g = 0.8
        line_marker.color.b = 1.0
        line_marker.color.a = 1.0
        line_marker.lifetime = DurationD(sec=0)

        line_points = []
        for i in range(len(self.trajectory) - 1):
            p1 = Point(x=self.trajectory[i][0], y=self.trajectory[i][1], z=0.0)
            p2 = Point(x=self.trajectory[i + 1][0], y=self.trajectory[i + 1][1], z=0.0)
            line_points.extend([p1, p2])
        line_marker.points = line_points

        sphere_marker = Marker()
        sphere_marker.header.frame_id = self.global_frame
        sphere_marker.header.stamp = now
        sphere_marker.ns = "trajectory_points"
        sphere_marker.id = 1
        sphere_marker.type = Marker.SPHERE_LIST
        sphere_marker.action = Marker.ADD
        sphere_marker.scale.x = 0.1
        sphere_marker.scale.y = 0.1
        sphere_marker.scale.z = 0.1
        sphere_marker.color.r = 1.0
        sphere_marker.color.g = 0.2
        sphere_marker.color.b = 0.2
        sphere_marker.color.a = 1.0
        sphere_marker.lifetime = DurationD(sec=0)

        sphere_points = [Point(x=x, y=y, z=0.0) for x, y in self.trajectory]
        sphere_marker.points = sphere_points

        self.marker_pub.publish(line_marker)
        self.marker_pub.publish(sphere_marker)
        self.get_logger().info('Published trajectory markers successfully.')

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
