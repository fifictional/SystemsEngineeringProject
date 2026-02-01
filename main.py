from demos import (
    run_kalman_demo, 
    run_linear_vs_nonlinear_demo, 
    kalman_demo_PID, 
    kalman_demo_LQR
)
from visualisation import animate_realtime_control 
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

    z0 = [0, 0, np.deg2rad(15), 0]
    

    # Option 1: Pre-computed demos (original)
    # run_kalman_demo(params)
    # kalman_demo_PID(params, z0)
    # t, z_nl, z_lin = run_linear_vs_nonlinear_demo(params, z0)
    

    # Option 2: Real-time interactive control 
    animate_realtime_control(params, z0, max_time=30.0)
