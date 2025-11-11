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

        # Call periodically so we can see logging
        self.timer = self.create_timer(1.0, self.plot)
        self.get_logger().info('Trajectory Visualizer started and running.')

    def plot(self):
        self.get_logger().info('Publishing trajectory markers...')
        now = self.get_clock().now().to_msg()

        # --- Obstacle 1 ---
        obs1 = Marker()
        obs1.header.frame_id = self.global_frame
        obs1.header.stamp = now
        obs1.ns = "obstacles"
        obs1.id = 2
        obs1.type = Marker.CYLINDER
        obs1.action = Marker.ADD
        obs1.pose.position.x = 0.0
        obs1.pose.position.y = 0.5
        obs1.pose.position.z = 0.0
        obs1.scale.x = 2 * 0.15  
        obs1.scale.y = 2 * 0.15
        obs1.scale.z = 0.5      # height (flat cylinder)
        obs1.color.r = 1.0
        obs1.color.g = 0.0
        obs1.color.b = 0.0
        obs1.color.a = 0.6
        obs1.lifetime = DurationD(sec=0)

        # # --- Obstacle 2 ---
        obs2 = Marker()
        obs2.header.frame_id = self.global_frame
        obs2.header.stamp = now
        obs2.ns = "obstacles"
        obs2.id = 3
        obs2.type = Marker.CYLINDER
        obs2.action = Marker.ADD
        obs2.pose.position.x = 0.0
        obs2.pose.position.y = -0.5
        obs2.pose.position.z = 0.0
        obs2.scale.x = 2 * 0.15
        obs2.scale.y = 2 * 0.15
        obs2.scale.z = 0.5
        obs2.color.r = 1.0
        obs2.color.g = 0.0
        obs2.color.b = 0.0
        obs2.color.a = 0.6
        obs2.lifetime = DurationD(sec=0)

        self.marker_pub.publish(obs1)
        self.marker_pub.publish(obs2)
        self.get_logger().info('Published trajectory markers successfully.')

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
