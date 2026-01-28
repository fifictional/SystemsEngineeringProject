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
