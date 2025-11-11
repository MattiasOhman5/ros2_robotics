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
        self.cap_offset = 0.1  # Cap is 0.1m in front of base_footprint
        self.cap_half_width = 0.075  # Cap is 0.15m wide, so half is 0.075
        self.cap_half_length = 0.05  # Cap is 0.1m long, so half is 0.05
        # Effective radius for circular approximation: diagonal of half-box
        self.cap_effective_radius = math.sqrt(self.cap_half_length**2 + self.cap_half_width**2)
        self.safety_distance = 0.00
        
        self.model = self.defineTBotModel()
        self.mpc = self.defineTBotMPC(model=self.model, ts=0.1, N=20)


    # ----------------- Define Model -----------------
    def defineTBotModel(self):
        model_type = 'continuous'
        model = do_mpc.model.Model(model_type)

        # States (now representing cap position)
        x = model.set_variable(var_type='_x', var_name='x', shape=(1,1))
        y = model.set_variable(var_type='_x', var_name='y', shape=(1,1))
        th = model.set_variable(var_type='_x', var_name='th', shape=(1,1))

        # Inputs (still applied to base_footprint)
        vx = model.set_variable(var_type='_u', var_name='vx')
        vt = model.set_variable(var_type='_u', var_name='vt')

        # Time-varying parameters (setpoints)
        xdes = model.set_variable(var_type='_tvp', var_name='xdes')
        ydes = model.set_variable(var_type='_tvp', var_name='ydes')

        # Dynamics - cap position moves with robot velocity plus rotation effect
        # dx_cap/dt = dx_base/dt - cap_offset * sin(th) * dth/dt
        # dy_cap/dt = dy_base/dt + cap_offset * cos(th) * dth/dt
        model.set_rhs('x', vx*ca.cos(th) - self.cap_offset*vt*ca.sin(th))
        model.set_rhs('y', vx*ca.sin(th) + self.cap_offset*vt*ca.cos(th))
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
        mpc.bounds['upper','_u','vx'] =  0.5
        mpc.bounds['lower','_u','vt'] = -0.8
        mpc.bounds['upper','_u','vt'] =  0.8

        # Use cap effective radius for obstacle avoidance
        R_total = self.cap_effective_radius + self.safety_distance

        # Nonlinear Constraint for Obstacle 1
        x_obs1 = 0.5
        y_obs1 = 0.0
        r_obs1 = 0.15
        R1 = r_obs1 + R_total
        mpc.set_nl_cons(
            'obs1',
            R1**2 - ((model.x['x']-x_obs1)**2 + (model.x['y']-y_obs1)**2),
            ub=0.0,
            soft_constraint=False
        )

        # Nonlinear Constraint for Obstacle 2
        x_obs2 = 0.0
        y_obs2 = 0.5
        r_obs2 = 0.15
        R2 = r_obs2 + R_total
        mpc.set_nl_cons(
            'obs2',
            R2**2 - ((model.x['x']-x_obs2)**2 + (model.x['y']-y_obs2)**2),
            ub=0.0,
            soft_constraint=False
        )

        self.add_obstacles_to_mpc(mpc, model, R_total)
        
        template = mpc.get_tvp_template()
        def tvp_fun(t_now):
            return template  
        mpc.set_tvp_fun(tvp_fun)

        mpc.setup()
        mpc.x0 = np.array([0.0, 0.0, 0.0]).reshape(-1, 1)
        mpc.set_initial_guess()
        return mpc


    def add_obstacles_to_mpc(self, mpc, model, R_total):
        cylinders = [
            (-1.1, -1.1, 0.15),
            (-1.1,  0.0, 0.15),
            (-1.1,  1.1, 0.15),
            ( 0.0, -1.1, 0.15),
            ( 0.0,  0.0, 0.15),
            ( 0.0,  1.1, 0.15),
            ( 1.1, -1.1, 0.15),
            ( 1.1,  0.0, 0.15),
            ( 1.1,  1.1, 0.15)
        ]
        
        for i, (x_obs, y_obs, r_obs) in enumerate(cylinders):
            R = r_obs + R_total
            mpc.set_nl_cons(
                f'cyl_{i}',
                R**2 - ((model.x['x']-x_obs)**2 + (model.x['y']-y_obs)**2),
                ub=0.0,
                soft_constraint=False
            )
        
        clearance = R_total
        
        # Wall 1: vertical at x=-1.1, inner face at x=-1.0
        mpc.set_nl_cons(
            'wall_1',
            model.x['x'] - (-1.0 - clearance),
            ub=0.0,
            soft_constraint=False
        )
        
        # Wall 2: horizontal at y=1.1, inner face at y=1.0
        mpc.set_nl_cons(
            'wall_2',
            model.x['y'] - (1.0 + clearance),
            ub=0.0,
            soft_constraint=False
        )
        
        # Wall 3: vertical at x=0, spans y∈[-1.1, 0], thickness ±0.10
        mpc.set_nl_cons('wall_3_left', -model.x['x'] - (0.10 + clearance), ub=0.0, soft_constraint=False)
        mpc.set_nl_cons('wall_3_right', model.x['x'] - (0.10 + clearance), ub=0.0, soft_constraint=False)
        
        # Wall 4: horizontal at y=-1.1, inner face at y=-1.0
        mpc.set_nl_cons(
            'wall_4',
            -1.0 + clearance - model.x['y'],
            ub=0.0,
            soft_constraint=False
        )
        
        # Wall 5: vertical at x=1.1, inner face at x=1.0
        mpc.set_nl_cons(
            'wall_5',
            model.x['x'] - (1.0 + clearance),
            ub=0.0,
            soft_constraint=False
        )
        
    
    def goal_callback(self, msg):
        gx, gy = msg.position.x, msg.position.y
        self.goal = (gx, gy)
        
        tvp = self.mpc.get_tvp_template()
        for k in range(self.mpc.settings.n_horizon+1):
            tvp['_tvp', k, 'xdes'] = gx
            tvp['_tvp', k, 'ydes'] = gy
        self.mpc.set_tvp_fun(lambda t_now: tvp)

    def odom_callback(self, msg):
        # Get base_footprint position and orientation
        x_base = msg.pose.pose.position.x
        y_base = msg.pose.pose.position.y
        yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)

        # Transform to cap position (0.1m in front of base_footprint)
        x_cap = x_base + self.cap_offset * math.cos(yaw)
        y_cap = y_base + self.cap_offset * math.sin(yaw)

        # Run MPC only if we have a goal
        if self.goal is None:
            return

        cmd = TwistStamped()

        gx, gy = self.goal
        distance = math.sqrt((gx - x_cap)**2 + (gy - y_cap)**2)
        if distance < self.goal_tolerance:
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.0

        else:
            # MPC plans in cap frame
            x0 = np.array([x_cap, y_cap, yaw]).reshape(-1, 1)
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