import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, TwistStamped
from nav_msgs.msg import Odometry
import math

class Simple_Controller(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(Pose, '/next_setpoint', self.goal_callback, 10)
        self.goal = None
        self.kp_lin = 0.5
        self.kp_ang = 1.0
        self.goal_tolerance = 0.005
        self.angle_tolerance = math.pi / 6

    def goal_callback(self, msg: Pose):
        self.goal = msg

    def odom_callback(self, msg: Odometry):
        if self.goal is None:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)

        gx = self.goal.position.x
        gy = self.goal.position.y

        dx = gx - x
        dy = gy - y
        distance = math.sqrt(dx**2 + dy**2)

        angle_to_goal = math.atan2(dy, dx)
        yaw_error = self.normalize_angle(angle_to_goal - yaw)

        cmd = TwistStamped()
        if distance > self.goal_tolerance:
            cmd.twist.angular.z = self.kp_ang * yaw_error
            if yaw_error > self.angle_tolerance:
                cmd.twist.linear.x = 0.0
            else:
                cmd.twist.linear.x = self.kp_lin * distance

        else:
            self.get_logger().info("Goal reached, stopping")
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.0

        self.cmd_pub.publish(cmd)

    def quaternion_to_yaw(self, q):
        A = 2.0 * (q.w * q.z + q.x * q.y)
        B = 1.0 - 2.0 * (q.y**2 + q.z**2)
        return math.atan2(A, B)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
def main(args=None):
    rclpy.init(args=args)
    node = Simple_Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
