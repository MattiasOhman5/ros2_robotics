import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, TwistStamped
from nav_msgs.msg import Odometry
import math
import casadi
import os
import numpy as np
import opengen as og

class MPC_Controller(Node):
    def __init__(self):
        super().__init__('mpc_controller')
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(Pose, '/next_setpoint', self.goal_callback, 10)

        package_dir = os.path.dirname(os.path.realpath(__file__))
        optimizer_path = os.path.join(package_dir, "bd", "turtle_mpc")
        self.mng = og.tcp.OptimizerTcpManager(optimizer_path)
        self.mng.start()

        self.goal = None
        self.current_state = None
        self.N = 15
        self.u_prev  = [0.0] * (2 * self.N)

    def goal_callback(self, msg):
        gx, gy = msg.position.x, msg.position.y
        self.goal = (gx, gy)
        #self.get_logger().info(f"New goal: ({gx:.2f}, {gy:.2f})")

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)

        # Run MPC only if we have a goal
        if self.goal is None:
            return
        
        gx, gy = self.goal
        p_val = np.array([x, y, yaw, gx, gy])

        sol = self.mng.call(p_val, initial_guess = self.u_prev, buffer_len=8*4096)

        if not sol.is_ok():
            self.get_logger().warn("MPC solver failed")
            return
        
        U = np.array(sol['solution'])
        v_seq = U[0:self.N]
        w_seq = U[self.N:]
        v0, w0 = v_seq[0], w_seq[0]

        self.u_prev = np.hstack((U[2:], [0.0, 0.0]))

        cmd = TwistStamped()
        cmd.twist.linear.x = float(v0)
        cmd.twist.angular.z = float(w0)
        self.cmd_pub.publish(cmd)

        #self.get_logger().info(
        #    f"MPC cmd: v={v0:.2f}, w={w0:.2f} | pos=({x:.2f}, {y:.2f}) → goal=({gx:.2f}, {gy:.2f})"
        #)

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
    node = MPC_Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()        

