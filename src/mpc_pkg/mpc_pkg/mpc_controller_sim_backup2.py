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
        self.goal_sub = self.create_subscription(Pose, '/next_setpoint', self.goal_callback, 10)

        self.goal = None
        self.goal_tolerance = 0.05
        self.turtlebot_radius = 0.11
        self.safety_distance = 0.05
        
        # Cap geometry from SDF
        self.cap_offset = 0.35  # Cap center: 0.05 (joint) + 0.25 (inertial offset)
        self.cap_half_width = 0.1
        self.cap_half_length = 0.18

        self.model = self.defineTBotModel()
        self.mpc = self.defineTBotMPC(model=self.model, ts=0.1, N=20)


    def defineTBotModel(self):
        model_type = 'continuous'
        model = do_mpc.model.Model(model_type)

        x = model.set_variable(var_type='_x', var_name='x', shape=(1,1))
        y = model.set_variable(var_type='_x', var_name='y', shape=(1,1))
        th = model.set_variable(var_type='_x', var_name='th', shape=(1,1))

        vx = model.set_variable(var_type='_u', var_name='vx')
        vt = model.set_variable(var_type='_u', var_name='vt')

        xdes = model.set_variable(var_type='_tvp', var_name='xdes')
        ydes = model.set_variable(var_type='_tvp', var_name='ydes')

        model.set_rhs('x', vx*ca.cos(th))
        model.set_rhs('y', vx*ca.sin(th))
        model.set_rhs('th', vt)

        model.setup()
        return model
    

    def defineTBotMPC(self, model, ts, N):
        mpc = do_mpc.controller.MPC(model)
        setup_mpc = {
            'n_horizon': 40,
            't_step': ts,
            'n_robust': 0,
            'store_full_solution': True,
            'nlpsol_opts': {
                'ipopt.max_iter': 250,
                'ipopt.tol': 1e-4,
                'ipopt.acceptable_tol': 1e-3,
            }
        }
        mpc.set_param(**setup_mpc)

        # Base tracking objective
        tracking_error = (model.x['x'] - model.tvp['xdes'])**2 + (model.x['y'] - model.tvp['ydes'])**2
        
        # Obstacle parameters
        obstacles = [
            (0.0, 0.5, 0.15, 'obs1'),
            (0.0, -0.5, 0.15, 'obs2')
        ]
        
        # Initialize repulsive cost
        repulsive_cost = 0
        
        for x_obs, y_obs, r_obs, obs_name in obstacles:
            # --- BASE FOOTPRINT ---
            dist_base = ca.sqrt((model.x['x'] - x_obs)**2 + (model.x['y'] - y_obs)**2)
            clearance_base = self.turtlebot_radius + r_obs + self.safety_distance
            
            # Hard constraint for base
            mpc.set_nl_cons(f'{obs_name}_base',
                clearance_base - dist_base,
                ub=0.0,
                soft_constraint=False)
            
            # --- CAP---
            dx = (x_obs - model.x['x'])*ca.cos(model.x['th']) + (y_obs - model.x['y'])*ca.sin(model.x['th'])
            dy = -(x_obs - model.x['x'])*ca.sin(model.x['th']) + (y_obs - model.x['y'])*ca.cos(model.x['th'])

            # Distance to cap rectangle (accounts for width and length)
            dx_to_cap = ca.fmax(0, ca.fabs(dx - self.cap_offset) - self.cap_half_length)
            dy_to_cap = ca.fmax(0, ca.fabs(dy) - self.cap_half_width)
            dist_to_cap = ca.sqrt(dx_to_cap**2 + dy_to_cap**2)

            clearance_cap = r_obs + self.safety_distance

            # Hard constraint for cap
            mpc.set_nl_cons(f'{obs_name}_cap',
                clearance_cap - dist_to_cap,
                ub=0.0,
                soft_constraint=False)
            
           
        # Combine tracking and repulsion in objective
        lterm = 10.0 * tracking_error + 3.0 * repulsive_cost
        mterm = 20.0 * tracking_error + 5.0 * repulsive_cost
        
        mpc.set_objective(mterm=mterm, lterm=lterm)
        mpc.set_rterm(vx=0.1, vt=0.5)

        # State and control bounds
        mpc.bounds['lower','_x','x'] = -1.0
        mpc.bounds['upper','_x','x'] =  1.0
        mpc.bounds['lower','_x','y'] = -1.0
        mpc.bounds['upper','_x','y'] =  1.0
        mpc.bounds['lower','_u','vx'] = 0.0
        mpc.bounds['upper','_u','vx'] =  0.5
        mpc.bounds['lower','_u','vt'] = -0.8
        mpc.bounds['upper','_u','vt'] =  0.8

        template = mpc.get_tvp_template()
        def tvp_fun(t_now):
            return template  
        mpc.set_tvp_fun(tvp_fun)

        mpc.setup()
        mpc.x0 = np.array([0.0, 0.0, 0.0]).reshape(-1, 1)
        mpc.set_initial_guess()
        return mpc

    def goal_callback(self, msg):
        gx, gy = msg.position.x, msg.position.y
        self.goal = (gx, gy)
        
        tvp = self.mpc.get_tvp_template()
        for k in range(self.mpc.settings.n_horizon+1):
            tvp['_tvp', k, 'xdes'] = gx
            tvp['_tvp', k, 'ydes'] = gy
        self.mpc.set_tvp_fun(lambda t_now: tvp)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)

        if self.goal is None:
            return

        cmd = TwistStamped()

        gx, gy = self.goal
        distance = math.sqrt((gx - x)**2 + (gy - y)**2)
        if distance < self.goal_tolerance:
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.0
        else:
            x0 = np.array([x, y, yaw]).reshape(-1, 1)
            u0 = self.mpc.make_step(x0)
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