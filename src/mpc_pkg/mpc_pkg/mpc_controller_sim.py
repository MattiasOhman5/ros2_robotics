import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, TwistStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import math
import casadi as ca
import os
import numpy as np
import do_mpc

class MPC_Controller(Node):
    def __init__(self):
        super().__init__('mpc_controller')
        self.goal = None
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(Pose, '/next_setpoint', self.goal_callback, 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)

        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_tolerance = 0.05
        self.turtlebot_radius = 0.11
        self.safety_distance = 0.05
        self.hitch_length = 0.212
        self.trailer_length = 0.25
        
        self.hitch_angle = 0.0

        self.model = self.defineTBotModel()
        self.mpc = self.defineTBotMPC(model=self.model, ts=0.05, N=20)

    def defineTBotModel(self):
        model_type = 'discrete'
        model = do_mpc.model.Model(model_type)

        x = model.set_variable(var_type='_x', var_name='x', shape=(1,1))
        y = model.set_variable(var_type='_x', var_name='y', shape=(1,1))
        th = model.set_variable(var_type='_x', var_name='th', shape=(1,1))
        phi = model.set_variable(var_type='_x', var_name='phi', shape=(1,1))

        vx = model.set_variable(var_type='_u', var_name='vx')
        vt = model.set_variable(var_type='_u', var_name='vt')

        xdes = model.set_variable(var_type='_tvp', var_name='xdes')
        ydes = model.set_variable(var_type='_tvp', var_name='ydes')

        L = self.hitch_length
        dt = 0.05

        # RK4 integration
        k1_x = vx*ca.cos(th)
        k1_y = vx*ca.sin(th)
        k1_th = vt
        k1_phi = vx*ca.sin(phi)/L - vt

        k2_x = vx*ca.cos(th + 0.5*dt*k1_th)
        k2_y = vx*ca.sin(th + 0.5*dt*k1_th)
        k2_th = vt
        k2_phi = vx*ca.sin(phi + 0.5*dt*k1_phi)/L - vt

        k3_x = vx*ca.cos(th + 0.5*dt*k2_th)
        k3_y = vx*ca.sin(th + 0.5*dt*k2_th)
        k3_th = vt
        k3_phi = vx*ca.sin(phi + 0.5*dt*k2_phi)/L - vt

        k4_x = vx*ca.cos(th + dt*k3_th)
        k4_y = vx*ca.sin(th + dt*k3_th)
        k4_th = vt
        k4_phi = vx*ca.sin(phi + dt*k3_phi)/L - vt

        model.set_rhs('x', x + (dt/6.0)*(k1_x + 2*k2_x + 2*k3_x + k4_x))
        model.set_rhs('y', y + (dt/6.0)*(k1_y + 2*k2_y + 2*k3_y + k4_y))
        model.set_rhs('th', th + (dt/6.0)*(k1_th + 2*k2_th + 2*k3_th + k4_th))
        model.set_rhs('phi', phi + (dt/6.0)*(k1_phi + 2*k2_phi + 2*k3_phi + k4_phi))

        model.setup()
        return model
    
    def defineTBotMPC(self, model, ts, N):
        mpc = do_mpc.controller.MPC(model)
        setup_mpc = {
            'n_horizon': N,
            't_step': ts,
            'n_robust': 0,
            'store_full_solution': True
        }
        mpc.set_param(**setup_mpc)

        # Objective with heavy penalty on hitch angle
        lterm = (model.x['x'] - model.tvp['xdes'])**2 + \
                (model.x['y'] - model.tvp['ydes'])**2 + \
                150*(model.x['phi'])**2
        mpc.set_objective(mterm=100*lterm, lterm=lterm)
        mpc.set_rterm(vx=0.1, vt=0.2)

        # State bounds
        mpc.bounds['lower','_x','x'] = -5.0
        mpc.bounds['upper','_x','x'] =  5.0
        mpc.bounds['lower','_x','y'] = -5.0
        mpc.bounds['upper','_x','y'] =  5.0
        mpc.bounds['lower','_x','phi'] = -0.5
        mpc.bounds['upper','_x','phi'] =  0.5

        # Input bounds
        mpc.bounds['lower','_u','vx'] = -0.15
        mpc.bounds['upper','_u','vx'] =  0.15
        mpc.bounds['lower','_u','vt'] = -2.0
        mpc.bounds['upper','_u','vt'] =  2.0

        # Nonlinear constraint on phi_dot
        L = self.hitch_length
        phi_dot = model.u['vx']*ca.sin(model.x['phi'])/L - model.u['vt']
        mpc.set_nl_cons('phi_dot_limit', phi_dot, ub=1.5, soft_constraint=False)
        mpc.set_nl_cons('phi_dot_limit_neg', -phi_dot, ub=1.5, soft_constraint=False)

        def tvp_fun(t_now):
            tvp = mpc.get_tvp_template()
            for k in range(N + 1):
                tvp['_tvp', k, 'xdes'] = self.goal_x
                tvp['_tvp', k, 'ydes'] = self.goal_y
            return tvp
        
        mpc.set_tvp_fun(tvp_fun)
        mpc.setup()
        
        return mpc
    
    def joint_callback(self, msg):
        try:
            idx = msg.name.index('hitch_joint')
            self.hitch_angle = msg.position[idx]
        except (ValueError, IndexError):
            pass
        
    def goal_callback(self, msg):
        self.goal_x = msg.position.x
        self.goal_y = msg.position.y
        self.goal = (self.goal_x, self.goal_y)

    def odom_callback(self, msg):
        if self.goal is None:
            return
        
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        
        x0 = np.array([[x], [y], [yaw], [self.hitch_angle]])
        
        cmd = TwistStamped()
        gx, gy = self.goal
        distance = math.sqrt((gx - x)**2 + (gy - y)**2)
        
        if distance < self.goal_tolerance:
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.0
        else:
            if abs(self.hitch_angle) > 0.6:
                self.get_logger().warn(f'Hitch angle critical: {self.hitch_angle:.2f} rad')
                cmd.twist.linear.x = 0.0
                cmd.twist.angular.z = 0.0
            else:
                self.mpc.x0 = x0
                u0 = self.mpc.make_step(x0)
                cmd.twist.linear.x = float(u0[0])
                cmd.twist.angular.z = float(u0[1])
                
                phi_dot = float(u0[0])*np.sin(self.hitch_angle)/self.hitch_length - float(u0[1])
                self.get_logger().info(
                    f'phi={self.hitch_angle:.3f}, phi_dot={phi_dot:.3f}, vx={float(u0[0]):.3f}, vt={float(u0[1]):.3f}',
                    throttle_duration_sec=1.0
                )
        
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