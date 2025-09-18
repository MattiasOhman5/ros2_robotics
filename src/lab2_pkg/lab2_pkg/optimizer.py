import os
import numpy as np
import casadi as ca
import opengen as og
import matplotlib.pyplot as plt


def setup_problem():
    N = 15  # horizon steps
    dt = 0.1  # sample time

    # weights
    q_pos = 10.0  # weight for x,y tracking error
    q_yaw = 0.1  # weight for heading error (not used here)
    r_u_lin = 0.05
    r_u_ang = 0.02
    s_du = 0.2

    # input bounds
    v_min, v_max = 0.0, 15.0
    w_min, w_max = -2.5, 2.5

    # casadi symbols for parameters
    p = ca.MX.sym('p', 5)
    x_current, y_current, theta_current, x_goal, y_goal = p[0], p[1], p[2], p[3], p[4]

    # decision variables (control inputs) -> optimizer must choose these
    U = ca.MX.sym('U', 2 * N)
    v = U[0:N]
    w = U[N:2 * N]

    return dict(
        N=N, dt=dt,
        q_pos=q_pos, q_yaw=q_yaw, r_u_lin=r_u_lin, r_u_ang=r_u_ang, s_du=s_du,
        v_min=v_min, v_max=v_max, w_min=w_min, w_max=w_max,
        p=p, x_current=x_current, y_current=y_current, theta_current=theta_current,
        x_goal=x_goal, y_goal=y_goal,
        U=U, v=v, w=w
    )


def dynamics(params):
    N, dt = params['N'], params['dt']
    v, w = params['v'], params['w']
    x_current, y_current, theta_current = params["x_current"], params["y_current"], params["theta_current"]

    # use lists for all states -> will be length N since that is our horizon
    X, Y, THETA = [], [], []

    x, y, theta = x_current, y_current, theta_current

    for k in range(N):
        x = x + v[k] * ca.cos(theta) * dt
        y = y + v[k] * ca.sin(theta) * dt
        theta = theta + w[k] * dt

        X.append(x)
        Y.append(y)
        THETA.append(theta)

    return ca.vertcat(*X), ca.vertcat(*Y), ca.vertcat(*THETA)


def cost_function(params, X, Y, THETA):
    x_goal, y_goal = params['x_goal'], params['y_goal']
    v, w = params["v"], params["w"]
    N = params["N"]

    # position tracking
    cost_position = params['q_pos'] * ca.sumsqr(X - x_goal) + params['q_pos'] * ca.sumsqr(Y - y_goal)

    # input effort
    cost_input = params['r_u_lin'] * ca.sumsqr(v) + params['r_u_ang'] * ca.sumsqr(w)

    # smoothness (differences between inputs)
    cost_smoothness = 0
    for k in range(1, N):
        cost_smoothness += params['s_du'] * ((v[k] - v[k - 1]) ** 2 + (w[k] - w[k - 1]) ** 2)

    total_cost = cost_position + cost_input + cost_smoothness

    return total_cost


def setup_optimizer(params):
    X, Y, THETA = dynamics(params)
    cost = cost_function(params, X, Y, THETA)

    U = params["U"]
    p = params["p"]

    v_min, v_max = params["v_min"], params["v_max"]
    w_min, w_max = params["w_min"], params["w_max"]
    N = params["N"]

    u_min = [v_min] * N + [w_min] * N
    u_max = [v_max] * N + [w_max] * N

    bounds = og.constraints.Rectangle(u_min, u_max)

    problem = og.builder.Problem(U, p, cost).with_constraints(bounds)

    build_config = og.config.BuildConfiguration() \
        .with_build_directory("bd") \
        .with_build_mode("release") \
        .with_tcp_interface_config(og.config.TcpServerConfiguration(bind_port=3320))

    meta = og.config.OptimizerMeta().with_optimizer_name("turtle_mpc")

    solver_config = og.config.SolverConfiguration() \
        .with_tolerance(1e-5) \
        .with_max_outer_iterations(5) \
        .with_initial_penalty(100.0)

    builder = og.builder.OpEnOptimizerBuilder(problem, meta, build_config, solver_config)
    builder.build()


def mpc_test(params):
    """
    Test the optimizer by simulating a dot (unicycle model) moving toward a goal.
    """
    mng = og.tcp.OptimizerTcpManager('bd/turtle_mpc')
    mng.start()

    x, y, theta = 0.0, 0.0, 0.0
    goal_x, goal_y = 2.0, 5.0
    dt = params["dt"]

    positions = []

    for step in range(150):  # simulate 30 control steps
        p_val = np.array([x, y, theta, goal_x, goal_y])

        sol = mng.call(p_val, initial_guess=[0.0] * (2 * params["N"]), buffer_len=8 * 4096)

        if not sol.is_ok():
            print("Solver failed:", sol.diagnostics)
            break

        U = np.array(sol["solution"])
        v_seq = U[0:params["N"]]
        w_seq = U[params["N"]:]

        v0, w0 = v_seq[0], w_seq[0]

        # apply dynamics
        x = x + v0 * np.cos(theta) * dt
        y = y + v0 * np.sin(theta) * dt
        theta = theta + w0 * dt

        positions.append((x, y))

        print(f"Step {step:02d} | v={v0:.2f}, w={w0:.2f} | pos=({x:.2f}, {y:.2f}) | goal=({goal_x}, {goal_y})")

    mng.kill()

    # plot trajectory
    positions = np.array(positions)
    plt.plot(positions[:, 0], positions[:, 1], 'b.-', label="Trajectory")
    plt.plot([goal_x], [goal_y], 'rx', label="Goal")
    plt.axis("equal")
    plt.legend()
    plt.xlabel("x")
    plt.ylabel("y")
    plt.title("MPC simulated trajectory")
    plt.show()


if __name__ == "__main__":
    params = setup_problem()
    setup_optimizer(params)  # builds if not already built
    mpc_test(params)
