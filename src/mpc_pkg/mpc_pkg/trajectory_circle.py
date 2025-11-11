import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import math


class Trajectory(Node):
    def __init__(self):
        super().__init__('trajectory_node')

        self.setpoint_pub = self.create_publisher(Pose, '/next_setpoint', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Circle trajectory parameters
        self.radius = 0.55         # circle radius [m]
        self.num_points = 12      # number of waypoints around the circle
        self.goal_tolerance = 0.1
        self.timeout_sec = 12.0   # max time to stay on one waypoint [s]

        # Generate evenly spaced waypoints on the circle
        self.trajectory = []
        for i in range(self.num_points):
            theta = 2 * math.pi * i / self.num_points
            x = self.radius * math.cos(theta)
            y = self.radius * math.sin(theta)
            self.trajectory.append((x, y))

        self.current_index = 0
        self.time_elapsed = 0.0

        # Timer for publishing + timeout checking
        self.timer = self.create_timer(0.5, self.publish_and_check)

        # Initialize first setpoint
        gx, gy = self.trajectory[self.current_index]
        self.set_current_setpoint(gx, gy)

    def set_current_setpoint(self, gx, gy):
        self.current_setpoint = Pose()
        self.current_setpoint.position.x = gx
        self.current_setpoint.position.y = gy
        self.current_setpoint.position.z = 0.0
        self.current_setpoint.orientation.w = 1.0
        self.time_elapsed = 0.0  # reset timer
        self.get_logger().info(f"New setpoint: ({gx:.2f}, {gy:.2f})")

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        gx, gy = self.trajectory[self.current_index]

        distance = math.sqrt((gx - x)**2 + (gy - y)**2)
        if distance < self.goal_tolerance:
            self.get_logger().info(f"Reached setpoint ({gx:.2f}, {gy:.2f})")
            self.advance_setpoint(reached=True)

    def publish_and_check(self):
        # Publish the current setpoint periodically
        self.setpoint_pub.publish(self.current_setpoint)

        # Increase time spent on this waypoint
        self.time_elapsed += 0.5
        if self.time_elapsed > self.timeout_sec:
            gx, gy = self.trajectory[self.current_index]
            self.get_logger().warn(f"Timeout: skipping setpoint ({gx:.2f}, {gy:.2f})")
            self.advance_setpoint(reached=False)

    def advance_setpoint(self, reached: bool):
        self.current_index = (self.current_index + 1) % len(self.trajectory)
        gx, gy = self.trajectory[self.current_index]
        self.set_current_setpoint(gx, gy)

    def destroy_node(self):
        self.get_logger().info("Trajectory node shutting down.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Trajectory()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
