import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose


class Trajectory(Node):
    def __init__(self):
        super().__init__('trajectory_node')

        self.setpoint_pub = self.create_publisher(Pose, '/next_setpoint', 10)

        # Declare mode parameter
        self.declare_parameter('mode', 1)
        mode = self.get_parameter('mode').get_parameter_value().integer_value

        # Choose setpoint based on mode
        if mode == 1:
            gx, gy = 0.5, 0.5
            self.get_logger().info("Mode 1: Setpoint (0.5, 0.5)")
        elif mode == 2:
            gx, gy = 2.0, 2.0
            self.get_logger().info("Mode 2: Setpoint (2.0, 2.0)")
        else:
            gx, gy = 0.0, 0.0
            self.get_logger().warn(f"Unknown mode {mode}, defaulting to (0.0, 0.0)")

        # Build Pose
        self.setpoint = Pose()
        self.setpoint.position.x = gx
        self.setpoint.position.y = gy
        self.setpoint.position.z = 0.0
        self.setpoint.orientation.w = 1.0

        # Publish periodically
        self.timer = self.create_timer(0.5, self.publish_setpoint)

    def publish_setpoint(self):
        self.setpoint_pub.publish(self.setpoint)


def main(args=None):
    rclpy.init(args=args)
    node = Trajectory()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()