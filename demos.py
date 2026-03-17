import numpy as np
from dynamics import nonlinear_dynamics, simulate, linearised_dynamics, linearised_state_space, rk4_step
from kalman_filter import Sensor, KalmanFilter
from visualisation import animate_with_kalman, plot_kalman_results, plot_pid_performance, plot_lqr_performance
import matplotlib.pyplot as plt
from controllers import *


def run_kalman_demo(params):
    dt = 0.02
    tf = 5.0

    z0 = np.array([0, 0, np.deg2rad(15), 0])
    sensor = Sensor(pos_std=0.01, angle_std=0.05)
    kf = KalmanFilter(dt)
    kf.x = np.array([0.05, 0, np.deg2rad(10), 0])
    t = np.arange(0, tf, dt)

    true_states = np.zeros((len(t), 4))
    noisy_meas = np.zeros((len(t), 2))
    filtered_states = np.zeros((len(t), 4))

    z = z0.copy()

    for i, ti in enumerate(t):
        true_states[i] = z

        # Measurement
        y = sensor.add_noise(z[[0, 2]])
        noisy_meas[i] = y

        kf.update(y)
        filtered_states[i] = kf.x
        kf.predict(u=0)

        # Propagate true system
        dz = nonlinear_dynamics(ti, z, u=0, p=params)
        z = z + dz * dt

    noisy_states = true_states.copy()
    # noisy_states[:, 0] = noisy_meas[:, 0]  # noisy position
    # noisy_states[:, 2] = noisy_meas[:, 1]  # noisy angle

    plot_kalman_results(t, true_states, noisy_meas, filtered_states)
    animate_with_kalman(t, noisy_states, filtered_states)


def run_linear_vs_nonlinear_demo(params, z0, u=0.0, dt=0.001, tf=5.0, plot=True):
    # nonlinear
    t, z_nl = simulate(nonlinear_dynamics, z0, 0, tf, dt, u, params)

    # linearised
    A, B = linearised_state_space(params)
    t, z_lin = simulate(linearised_dynamics, z0, 0, tf, dt, u, A, B)

    if plot:
        import matplotlib.pyplot as plt
        plt.figure(figsize=(10,4))
        plt.subplot(1,2,1)
        plt.plot(t, np.rad2deg(z_nl[2,:]), label="Nonlinear", linewidth=2)
        plt.plot(t, np.rad2deg(z_lin[2,:]), '--', label="Linearised", linewidth=2)
        plt.xlabel("Time (s)")
        plt.ylabel("Pendulum angle θ (deg)")
        plt.title("Pendulum angle comparison")
        plt.ylim([0, 200])  
        plt.legend()
        plt.grid()

        plt.subplot(1,2,2)
        plt.plot(t, z_nl[0,:], label="Nonlinear", linewidth=2)
        plt.plot(t, z_lin[0,:], '--', label="Linearised", linewidth=2)
        plt.xlabel("Time (s)")
        plt.ylabel("Cart position x (m)")
        plt.title("Cart position comparison")
        plt.ylim([-0.5, 0.5]) 
        plt.legend()
        plt.grid()

        plt.tight_layout()
        plt.show()

    return t, z_nl, z_lin


def kalman_demo_PID(params, z0, disturbance_force=0.0):
    dt = 0.02
    tf = 10
    t = np.arange(0, tf, dt)
    z = np.array(z0, dtype=float)

    sensor = Sensor(pos_std=0.01, angle_std=0.05)
    kf = KalmanFilter(dt)
    kf.x = np.array([0.05, 0, np.deg2rad(10), 0])

    theta_error = np.zeros(len(t))
    true_states = np.zeros((len(t), 4))
    noisy_meas = np.zeros((len(t), 2))
    filtered_states = np.zeros((len(t), 4))
    controls = np.zeros(len(t))

    theta_ref = 0
    pid = PID_controller(Kp=55, Ki=0.001, Kd=1.5, N=10)

    for i, ti in enumerate(t):
        true_states[i] = z

        y = sensor.add_noise(z[[0, 2]])
        noisy_meas[i] = y
        kf.update(y)

        theta_hat = kf.x[2]
        error = theta_ref - theta_hat
        theta_error[i] = error
        u = -pid.step(theta_ref, theta_hat, dt)

        # Apply disturbance at first step only
        disturbance_steps = int(0.4 / dt) 
        if i < disturbance_steps:
            u += disturbance_force



        controls[i] = u
        kf.predict(u=u)
        filtered_states[i] = kf.x

        z = rk4_step(nonlinear_dynamics, ti, z, dt, u, params)

    noisy_states = true_states.copy()
    # noisy_states[:, 0] = noisy_meas[:, 0]
    # noisy_states[:, 2] = noisy_meas[:, 1]

    animate_with_kalman(
        t,
        noisy_states,
        filtered_states,
        controls=controls,
        disturbances=None,
        control_mode={"mode": "PID"},
        params=params,
        z0=z0
    )

def kalman_demo_PID_move2m(params, z0, disturbance_force=0.0, animate=True):
    dt = 0.02
    tf = 12.0
    t = np.arange(0, tf, dt)
    z = np.array(z0, dtype=float)

    sensor = Sensor(pos_std=0.01, angle_std=0.05)
    kf = KalmanFilter(dt)
    kf.x = np.array([0.05, 0, np.deg2rad(10), 0])

    true_states = np.zeros((len(t), 4))
    noisy_meas = np.zeros((len(t), 2))
    filtered_states = np.zeros((len(t), 4))
    controls = np.zeros(len(t))

    # "2 meter sprint" target + stability requirement
    x_target = 2.0
    hold_time_s = 2.0
    tol_x_m = 0.05
    tol_xdot_mps = 0.08
    tol_theta_deg = 6.0
    tol_thetadot_rps = 0.25

    def smoothstep(s):
        # s in [0,1]
        return 3 * s**2 - 2 * s**3

    def smoothstep_dot(s, T):
        return (6 * s * (1 - s)) / T

    # Trajectory (reduces overshoot vs "step to 2m")
    T_move = 4.5

    # Outer loop: position/velocity tracking -> tilt reference
    Kpx = 1.4
    Kdx = 2.2
    Kix = 0.18
    x_int = 0.0
    x_int_max = 0.6

    theta_ref_max = np.deg2rad(12)

    # Inner loop: tilt tracking -> force
    pid = PID_controller(Kp=70, Ki=0.02, Kd=3.0, N=10)
    u_max = 25.0

    stable_steps_needed = int(round(hold_time_s / dt))
    stable_run = 0
    stable_achieved_time = None

    for i, ti in enumerate(t):
        true_states[i] = z

        y = sensor.add_noise(z[[0, 2]])
        noisy_meas[i] = y
        kf.update(y)

        x_hat  = kf.x[0]
        xd_hat = kf.x[1]

        if ti <= T_move:
            s = ti / T_move
            x_des = x_target * smoothstep(s)
            xdot_des = x_target * smoothstep_dot(s, T_move)
        else:
            x_des = x_target
            xdot_des = 0.0

        # Anti-windup: only integrate when close-ish and not obviously saturated
        x_err = x_des - x_hat
        if abs(x_err) < 0.8:
            x_int = np.clip(x_int + x_err * dt, -x_int_max, x_int_max)

        theta_ref = np.clip(
            Kpx * x_err + Kix * x_int - Kdx * (xd_hat - xdot_des),
            -theta_ref_max, theta_ref_max
        )

        u = -pid.step(theta_ref, kf.x[2], dt)
        u = float(np.clip(u, -u_max, u_max))

        # Apply disturbance at first step only
        disturbance_steps = int(0.4 / dt) 
        if i < disturbance_steps:
            u += disturbance_force

        controls[i] = u
        kf.predict(u=u)
        filtered_states[i] = kf.x

        z = rk4_step(nonlinear_dynamics, ti, z, dt, u, params)

        x = z[0]
        xdot = z[1]
        theta = z[2]
        thetadot = z[3]
        is_stable = (
            abs(x - x_target) <= tol_x_m
            and abs(xdot) <= tol_xdot_mps
            and abs(np.rad2deg(theta)) <= tol_theta_deg
            and abs(thetadot) <= tol_thetadot_rps
        )
        if is_stable:
            stable_run += 1
            if stable_achieved_time is None and stable_run >= stable_steps_needed:
                stable_achieved_time = ti - hold_time_s + dt
        else:
            stable_run = 0

    if stable_achieved_time is None:
        print(
            f"[PID move2m] Did not meet stability window: "
            f"x={x_target:.2f}m ±{tol_x_m:.2f}m for {hold_time_s:.1f}s "
            f"(also |ẋ|≤{tol_xdot_mps:.2f}, |θ|≤{tol_theta_deg:.1f}°, |ω|≤{tol_thetadot_rps:.2f})."
        , flush=True)
    else:
        print(
            f"[PID move2m] Stable at x={x_target:.2f}m for {hold_time_s:.1f}s "
            f"starting ~t={stable_achieved_time:.2f}s."
        , flush=True)

    noisy_states = true_states.copy()
    # noisy_states[:, 0] = noisy_meas[:, 0]
    # noisy_states[:, 2] = noisy_meas[:, 1]

    if animate:
        animate_with_kalman(
            t,
            noisy_states,
            filtered_states,
            controls=controls,
            disturbances=None,
            control_mode={"mode": "PID"},
            params=params,
            z0=z0
        )
    return {
        "t": t,
        "true_states": true_states,
        "filtered_states": filtered_states,
        "controls": controls,
        "x_target": x_target,
        "stable_achieved_time": stable_achieved_time,
        "stability": {
            "hold_time_s": hold_time_s,
            "tol_x_m": tol_x_m,
            "tol_xdot_mps": tol_xdot_mps,
            "tol_theta_deg": tol_theta_deg,
            "tol_thetadot_rps": tol_thetadot_rps,
        },
    }

def kalman_demo_LQR(params, z0, disturbance_force=0.0):
    dt = 0.02
    tf = 10.0
    t = np.arange(0, tf, dt)
    z = np.array(z0, dtype=float)

    # Sensor + Kalman filter
    sensor = Sensor(pos_std=0.01, angle_std=0.05)
    kf = KalmanFilter(dt)
    kf.x = np.array([0.0, 0.0, np.deg2rad(10.0), 0.0])

    # LQR setup
    Q = np.diag([10.0, 1.0, 200.0, 5.0])
    R = np.array([[2.0]])
    u_max = 20.0
    lqr = LQRController(params, Q=Q, R=R, u_max=u_max)
    x_ref = np.zeros(4)

    true_states = np.zeros((len(t), 4))
    noisy_meas = np.zeros((len(t), 2))
    filtered_states = np.zeros((len(t), 4))
    controls = np.zeros(len(t))

    def wrap_angle(angle):
        return (angle + np.pi) % (2.0 * np.pi) - np.pi

    for i, ti in enumerate(t):
        true_states[i] = z

        # Measurement
        y = sensor.add_noise(z[[0, 2]])
        noisy_meas[i] = y
        kf.update(y)

        # Wrap angle for LQR
        x_for_lqr = kf.x.copy()
        x_for_lqr[2] = wrap_angle(x_for_lqr[2])

        # Compute control
        u = lqr.compute_control(x_for_lqr, x_ref)

        # Apply disturbance on first step only
        disturbance_steps = int(0.4 / dt) 
        if i < disturbance_steps:
            u += disturbance_force



        controls[i] = u

        # Kalman predict
        kf.predict(u=u)
        filtered_states[i] = kf.x

        # Propagate system
        z = rk4_step(nonlinear_dynamics, ti, z, dt, u, params)

    # Reconstruct noisy states for visualization
    noisy_states = true_states.copy()
    # noisy_states[:, 0] = noisy_meas[:, 0]
    # noisy_states[:, 2] = noisy_meas[:, 1]

    # Animate
    animate_with_kalman(
        t,
        noisy_states,
        filtered_states,
        controls=controls,
        disturbances=None,
        control_mode={"mode": "LQR"},
        params=params,
        z0=z0
    )

def kalman_demo_LQR_move2m(params, z0, disturbance_force=0.0, animate=True):
    dt = 0.02
    tf = 12.0
    t = np.arange(0, tf, dt)
    z = np.array(z0, dtype=float)

    # Sensor + Kalman filter
    sensor = Sensor(pos_std=0.01, angle_std=0.05)
    kf = KalmanFilter(dt)
    kf.x = np.array([0.0, 0.0, np.deg2rad(10.0), 0.0])

    # LQR setup
    # Use a 2-stage LQR: "move" (track trajectory) then "hold" (brake & stabilize).
    u_max = 30.0
    Q_move = np.diag([220.0, 1500.0, 1400.0, 400.0])
    R_move = np.array([[2.0]])
    lqr_move = LQRController(params, Q=Q_move, R=R_move, u_max=u_max)

    Q_hold = np.diag([800.0, 9000.0, 2600.0, 1400.0])
    R_hold = np.array([[2.0]])
    lqr_hold = LQRController(params, Q=Q_hold, R=R_hold, u_max=u_max)

    def smoothstep(s):
        # s in [0,1]
        return 3*s**2 - 2*s**3

    def smoothstep_dot(s, T):
        return (6*s*(1-s)) / T
    
    # "2 meter sprint" target + stability requirement
    x_target = 2.0
    T_move = 5.0
    hold_time_s = 2.0
    tol_x_m = 0.05
    tol_xdot_mps = 0.08
    tol_theta_deg = 6.0
    tol_thetadot_rps = 0.25

    stable_steps_needed = int(round(hold_time_s / dt))
    stable_run = 0
    stable_achieved_time = None

    true_states = np.zeros((len(t), 4))
    noisy_meas = np.zeros((len(t), 2))
    filtered_states = np.zeros((len(t), 4))
    controls = np.zeros(len(t))

    def wrap_angle(angle):
        return (angle + np.pi) % (2.0 * np.pi) - np.pi

    for i, ti in enumerate(t):
        true_states[i] = z

        # Measurement
        y = sensor.add_noise(z[[0, 2]])
        noisy_meas[i] = y
        kf.update(y)

        # Wrap angle for LQR
        x_for_lqr = kf.x.copy()
        x_for_lqr[2] = wrap_angle(x_for_lqr[2])

        # Reference
        if ti <= T_move:
            s = ti / T_move
            x_des = x_target * smoothstep(s)
            xdot_des = x_target * smoothstep_dot(s, T_move)
        else:
            x_des = x_target
            xdot_des = 0.0
        x_ref = np.array([x_des, xdot_des, 0.0, 0.0])

        # Compute control
        if ti <= T_move:
            u = lqr_move.compute_control(x_for_lqr, x_ref)
        else:
            u = lqr_hold.compute_control(x_for_lqr, x_ref)

        # Apply disturbance on first step only
        disturbance_steps = int(0.4 / dt) 
        if i < disturbance_steps:
            u += disturbance_force

        controls[i] = u

        # Kalman predict
        kf.predict(u=u)
        filtered_states[i] = kf.x

        # Propagate system
        z = rk4_step(nonlinear_dynamics, ti, z, dt, u, params)

        x = z[0]
        xdot = z[1]
        theta = z[2]
        thetadot = z[3]
        is_stable = (
            abs(x - x_target) <= tol_x_m
            and abs(xdot) <= tol_xdot_mps
            and abs(np.rad2deg(theta)) <= tol_theta_deg
            and abs(thetadot) <= tol_thetadot_rps
        )
        if is_stable:
            stable_run += 1
            if stable_achieved_time is None and stable_run >= stable_steps_needed:
                stable_achieved_time = ti - hold_time_s + dt
        else:
            stable_run = 0

    if stable_achieved_time is None:
        print(
            f"[LQR move2m] Did not meet stability window: "
            f"x={x_target:.2f}m ±{tol_x_m:.2f}m for {hold_time_s:.1f}s "
            f"(also |ẋ|≤{tol_xdot_mps:.2f}, |θ|≤{tol_theta_deg:.1f}°, |ω|≤{tol_thetadot_rps:.2f})."
        , flush=True)
    else:
        print(
            f"[LQR move2m] Stable at x={x_target:.2f}m for {hold_time_s:.1f}s "
            f"starting ~t={stable_achieved_time:.2f}s."
        , flush=True)

    # Reconstruct noisy states for visualization
    noisy_states = true_states.copy()
    # noisy_states[:, 0] = noisy_meas[:, 0]
    # noisy_states[:, 2] = noisy_meas[:, 1]

    # Animate
    if animate:
        animate_with_kalman(
            t,
            noisy_states,
            filtered_states,
            controls=controls,
            disturbances=None,
            control_mode={"mode": "LQR"},
            params=params,
            z0=z0
        )
    return {
        "t": t,
        "true_states": true_states,
        "filtered_states": filtered_states,
        "controls": controls,
        "x_target": x_target,
        "stable_achieved_time": stable_achieved_time,
        "stability": {
            "hold_time_s": hold_time_s,
            "tol_x_m": tol_x_m,
            "tol_xdot_mps": tol_xdot_mps,
            "tol_theta_deg": tol_theta_deg,
            "tol_thetadot_rps": tol_thetadot_rps,
        },
    }
