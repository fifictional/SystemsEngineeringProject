from demos import run_kalman_demo, run_linear_vs_nonlinear_demo
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
    # print("Running Kalman filter demo...")
    run_kalman_demo(params)

    # z0 = [0, 0, np.deg2rad(5), 0]
    # t, z_nl, z_lin = run_linear_vs_nonlinear_demo(params, z0)

    # z0_large = [0, 0, np.deg2rad(20), 0]
    # t2, z_nl2, z_lin2 = run_linear_vs_nonlinear_demo(params, z0_large)

