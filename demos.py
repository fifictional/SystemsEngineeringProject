import numpy as np
from dynamics import nonlinear_dynamics, simulate, linearised_dynamics, linearised_state_space, rk4_step
from kalman_filter import Sensor, KalmanFilter
from visualisation import animate_with_kalman, plot_kalman_results, plot_pid_performance
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

        # Kalman update + predict
        kf.update(y)
        filtered_states[i] = kf.x
        kf.predict(u=0)

        # Propagate true system
        dz = nonlinear_dynamics(ti, z, u=0, p=params)
        z = z + dz * dt

    plot_kalman_results(t, true_states, noisy_meas, filtered_states)
    animate_with_kalman(t, true_states, filtered_states)


def run_linear_vs_nonlinear_demo(params, z0, u=0.0, dt=0.001, tf=5.0, plot=True):
    #  nonlinear
    t, z_nl = simulate(nonlinear_dynamics, z0, 0, tf, dt, u, params)

    # linearised
    A, B = linearised_state_space(params)
    t, z_lin = simulate(linearised_dynamics, z0, 0, tf, dt, u, A, B)

    if plot:
        import matplotlib.pyplot as plt
        plt.figure(figsize=(10,4))
        plt.subplot(1,2,1)
        plt.plot(t, np.rad2deg(z_nl[2,:]), label="Nonlinear")
        plt.plot(t, np.rad2deg(z_lin[2,:]), '--', label="Linearised")
        plt.xlabel("Time (s)")
        plt.ylabel("Pendulum angle θ (deg)")
        plt.title("Pendulum angle comparison")
        plt.ylim([0, 200])  
        plt.legend()
        plt.grid()

        plt.subplot(1,2,2)
        plt.plot(t, z_nl[0,:], label="Nonlinear")
        plt.plot(t, z_lin[0,:], '--', label="Linearised")
        plt.xlabel("Time (s)")
        plt.ylabel("Cart position x (m)")
        plt.title("Cart position comparison")
        plt.ylim([-0.5, 0.5]) 
        plt.legend()
        plt.grid()

        plt.tight_layout()
        plt.show()

    return t, z_nl, z_lin

def kalman_demo_PID(params, z0):
    dt = 0.02
    tf = 10
    t = np.arange(0, tf, dt)
    z = np.array(z0, dtype=float)

    sensor = Sensor(pos_std=0.01, angle_std=0.05)
    kf = KalmanFilter(dt)
    kf.x = np.array([0.05, 0, np.deg2rad(10), 0])

    # Just for tuning
    theta_error = np.zeros(len(t))
    true_states = np.zeros((len(t), 4))
    noisy_meas = np.zeros((len(t), 2))
    filtered_states = np.zeros((len(t), 4))
    controls = np.zeros(len(t))

    theta_ref = 0
    pid = PID_controller(Kp=70, Ki=0, Kd=22, N=10)

    for i, ti in enumerate(t):
        true_states[i] = z

        y = sensor.add_noise(z[[0, 2]])
        noisy_meas[i] = y
        kf.update(y)

        theta_hat = kf.x[2]
        error = theta_ref - theta_hat
        theta_error[i] = error
        u = -pid.step(theta_ref, theta_hat, dt)
        controls[i] = u

        kf.predict(u=u)
        filtered_states[i] = kf.x

        z = rk4_step(nonlinear_dynamics, ti, z, dt, u, params)

    plot_pid_performance(t, theta_error)
    animate_with_kalman(t, true_states, filtered_states)
    return t, true_states, noisy_meas, filtered_states, controls, theta_error

def kalman_demo_LQR(params, z0):
    dt = 0.02
    tf = 10
    t = np.arange(0, tf, dt)
    z = np.array(z0, dtype=float)

    sensor = Sensor(pos_std=0.01, angle_std=0.05)
    kf = KalmanFilter(dt)

    kf.x = np.array([0.0, 0.0, np.deg2rad(10.0), 0.0])


    Q = np.diag([
        100.0,   # x       : cart position weight
        10.0,    # x_dot   : cart velocity weight
        1000.0,  # theta_tilde : pendulum angle deviation from 180 deg
        50.0     # theta_dot   : pendulum angular velocity weight
    ])

    R = np.array([[10.0]])

    u_max = 20.0

    lqr = LQRController(params, Q=Q, R=R, u_max=u_max)

    true_states = np.zeros((len(t), 4))
    noisy_meas = np.zeros((len(t), 2))
    filtered_states = np.zeros((len(t), 4))
    controls = np.zeros(len(t))

    x_ref = np.zeros(4)

    for i, ti in enumerate(t):
        true_states[i] = z


        y = sensor.add_noise(z[[0, 2]])
        noisy_meas[i] = y

        kf.update(y)

    
        x_for_lqr = kf.x.copy()
    
        x_for_lqr[2] = x_for_lqr[2] + np.pi

      
        u = -lqr.compute_control(x_for_lqr, x_ref)
        controls[i] = u

        kf.predict(u=u)
        filtered_states[i] = kf.x

        z = rk4_step(nonlinear_dynamics, ti, z, dt, u, params)


    theta_true = true_states[:, 2]
    theta_tilde_true = theta_true - np.pi   
    plot_pid_performance(t, theta_tilde_true)


    animate_with_kalman(t, true_states, filtered_states)

    return t, true_states, noisy_meas, filtered_states, controls
