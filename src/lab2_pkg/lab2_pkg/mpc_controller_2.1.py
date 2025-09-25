import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, TwistStamped, Twist
from nav_msgs.msg import Odometry
import math
import casadi as ca
import os
import numpy as np
import do_mpc

class MPC_Controller(Node):
    def __init__(self):
        super().__init__('mpc_controller')
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.turtlebot_radius = 0.11
        self.safety_distance = 0.09
        self.cx, self.cy = 0.0, 0.0
        self.radius = 0.6
        T = 15
        self.omega = 2 * math.pi / T
        self.latest_odom = None

        self.model = self.defineTBotModel()
        self.mpc = self.defineTBotMPC(model=self.model, ts=0.2, N=15)

        self.control_timer = self.create_timer(0.2, self.control_loop)

    # ----------------- Define Model -----------------
    def defineTBotModel(self):

        model_type = 'continuous'
        model = do_mpc.model.Model(model_type)

        # States
        x = model.set_variable(var_type='_x', var_name='x', shape=(1,1))
        y = model.set_variable(var_type='_x', var_name='y', shape=(1,1))
        th = model.set_variable(var_type='_x', var_name='th', shape=(1,1))

        # Inputs
        vx = model.set_variable(var_type='_u', var_name='vx')
        vt = model.set_variable(var_type='_u', var_name='vt')

        # Time-varying parameters (setpoints)

        xdes = model.set_variable(var_type='_tvp', var_name='xdes')
        ydes = model.set_variable(var_type='_tvp', var_name='ydes')

        # Dynamics
        model.set_rhs('x', vx*ca.cos(th))
        model.set_rhs('y', vx*ca.sin(th))
        model.set_rhs('th', vt)

        model.setup()
        return model
    
    # ----------------- Setup MPC -----------------
    def defineTBotMPC(self, model, ts, N):
        mpc = do_mpc.controller.MPC(model)
        setup_mpc = {
            'n_horizon': N,
            't_step': ts,
            'n_robust': 1,
            'store_full_solution': True
        }
        mpc.set_param(**setup_mpc)

        # Objective
        lterm = (model.x['x'] - model.tvp['xdes'])**2 + (model.x['y'] - model.tvp['ydes'])**2
        mterm = lterm
        mpc.set_objective(mterm=mterm, lterm=lterm)
        mpc.set_rterm(vx=1e-2, vt=1e-2)

        # State Bounds
        mpc.bounds['lower','_x','x'] = -1.0
        mpc.bounds['upper','_x','x'] =  1.0
        mpc.bounds['lower','_x','y'] = -1.0
        mpc.bounds['upper','_x','y'] =  1.0

        # Input Constraints
        mpc.bounds['lower','_u','vx'] = 0.0
        mpc.bounds['upper','_u','vx'] =  0.1
        mpc.bounds['lower','_u','vt'] = -0.5
        mpc.bounds['upper','_u','vt'] =  0.5

        # Nonlinear Constraint for Obstacle 1
        x_obs1 = 0.5
        y_obs1 = 0.0
        r_obs1 = 0.15
        R1 = r_obs1 + self.turtlebot_radius + self.safety_distance
        mpc.set_nl_cons(
            'obs1',
            R1**2 - ((model.x['x']-x_obs1)**2 + (model.x['y']-y_obs1)**2),
            ub=0.0,
            soft_constraint=False   # <-- no penalty, enforced strictly
        )

        # Nonlinear Constraint for Obstacle 2
        x_obs2 = 0.0
        y_obs2 = 0.5
        r_obs2 = 0.15
        R2 = r_obs2 + self.turtlebot_radius + self.safety_distance
        mpc.set_nl_cons(
            'obs2',
            R2**2 - ((model.x['x']-x_obs2)**2 + (model.x['y']-y_obs2)**2),
            ub=0.0,
            soft_constraint=False   # <-- no penalty, enforced strictly
        )

        template = mpc.get_tvp_template()
        def tvp_fun(t_now):
            for k in range(mpc.settings.n_horizon+1):
                t = t_now + k * mpc.settings.t_step
                template['_tvp', k, 'xdes'] = self.cx + self.radius * math.cos(self.omega * t)
                template['_tvp', k, 'ydes'] = self.cy + self.radius * math.sin(self.omega * t)
            return template
        mpc.set_tvp_fun(tvp_fun)

        mpc.setup()
        mpc.x0 = np.array([0.0, 0.0, 0.0]).reshape(-1, 1)
        mpc.set_initial_guess()

        return mpc
        

    def odom_callback(self, msg):
        self.latest_odom = msg

    def control_loop(self):
        if self.latest_odom is None:
            return

        x = self.latest_odom.pose.pose.position.x
        y = self.latest_odom.pose.pose.position.y
        yaw = self.quaternion_to_yaw(self.latest_odom.pose.pose.orientation)

        x0 = np.array([x, y, yaw]).reshape(-1, 1)
        self.mpc.x0 = x0
        u0 = self.mpc.make_step(x0)

        cmd = TwistStamped()
        cmd.twist.linear.x = float(u0[0])
        cmd.twist.angular.z = float(u0[1])
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

