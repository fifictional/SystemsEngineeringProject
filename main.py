from demos import run_kalman_demo, run_linear_vs_nonlinear_demo, kalman_demo_PID
import numpy as np

if __name__ == "__main__":
    params = {
        "M": 1.0,
        "m": 0.2,
        "l": 0.3,
        "I": 0.006,
        "g": 9.81,
        "b_x": 0.1,
        "b_theta": 0.05
    }

    # uncomment to run with and without noise demo
    run_kalman_demo(params)


    z0 = [0, 0, np.deg2rad(5), 0]
    kalman_demo_PID(params, z0)
    
    t, z_nl, z_lin = run_linear_vs_nonlinear_demo(params, z0)
    z0_large = [0, 0, np.deg2rad(20), 0]
    t2, z_nl2, z_lin2 = run_linear_vs_nonlinear_demo(params, z0_large)

    z0 = np.array([0.0, 0.0, np.deg2rad(2.0), 0.0])
    kalman_demo_LQR(params, z0)
