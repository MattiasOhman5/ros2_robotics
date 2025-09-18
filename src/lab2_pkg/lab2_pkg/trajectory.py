import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import math
import numpy as np

class Trajectory(Node):
    def __init__(self):
        super().__init__('trajectory_node')

        self.setpoint_pub = self.create_publisher(Pose, '/next_setpoint', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.trajectory1 = [(0.01, 1.01),
                            (-1.0, 1.0),
                            (-1.0, 0.0),
                            (0.0, 0.0)]
        
        self.trajectory2 = self.generate_8()

        self.setpoints = self.trajectory1
        self.current_index = 0
        self.goal_tolerance = 0.01

        gx, gy = self.setpoints[self.current_index]
        self.current_setpoint = Pose()
        self.current_setpoint.position.x = gx
        self.current_setpoint.position.y = gy
        self.current_setpoint.position.z = 0.0
        self.current_setpoint.orientation.w = 1.0
        self.setpoint_pub.publish(self.current_setpoint)
        self.get_logger().info(f'Publishing initial setpoint: {gx:.2f}, {gy:.2f}')

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        gx, gy = self.setpoints[self.current_index]
        distance = math.sqrt((gx - x)**2 + (gy - y)**2)

        if distance < self.goal_tolerance:
            self.update_setpoint()

        self.setpoint_pub.publish(self.current_setpoint)

    def update_setpoint(self):
        self.current_index = (self.current_index + 1) % len(self.setpoints)
        gx, gy = self.setpoints[self.current_index]

        next_setpoint = Pose()
        next_setpoint.position.x = gx
        next_setpoint.position.y = gy
        next_setpoint.position.z = 0.0
        next_setpoint.orientation.w = 1.0

        self.current_setpoint = next_setpoint
        self.get_logger().info(f'Reached goal, new setpoint: {gx:.2f}, {gy:.2f}')

    def generate_8(self):
        theta1 = np.linspace(1.5*np.pi, 3.5*np.pi, 50, endpoint=False)
        theta2 = np.linspace(0.5*np.pi, -2.5*np.pi, 50, endpoint=False)
        r = 1.0

        x1 = r * np.cos(theta1)
        y1 = r * np.sin(theta1) + 1

        x2 = r * np.cos(theta2)
        y2 = r * np.sin(theta2) - 1 

        x = np.hstack((x1, x2))
        y = np.hstack((y1, y2))

        return list(zip(x, y))


def main(args=None):
    rclpy.init(args=args)
    node = Trajectory()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
