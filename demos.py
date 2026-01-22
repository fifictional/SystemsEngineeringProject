import numpy as np
from dynamics import nonlinear_dynamics, simulate, linearised_dynamics, linearised_state_space
from kalman_filter import Sensor, KalmanFilter
from visualisation import animate_with_kalman, plot_kalman_results

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