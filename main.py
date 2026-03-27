from demos import (
    kalman_demo_LQR_move2m,
    kalman_demo_PID_move2m,
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

    z0 = [0, 0, np.deg2rad(40), 0]

    # Option 1: Pre-computed demos (original)
    # run_kalman_demo(params)
    # x_settle_lqr, final_error_lqr = kalman_demo_LQR_move2m(params, z0, disturbance_force=0)
    # lqr_settle_text = f"{x_settle_lqr:.2f} s" if x_settle_lqr is not None else "not settled"
    # print(f"LQR move 2m settling time = {lqr_settle_text}, final position error = {final_error_lqr:.2f} m")

    # x_settle_pid, final_error_pid = kalman_demo_PID_move2m(params, z0, disturbance_force=0)
    # pid_settle_text = f"{x_settle_pid:.2f} s" if x_settle_pid is not None else "not settled"
    # print(f"PID move 2m settling time = {pid_settle_text}, final position error = {final_error_pid:.2f} m")
    # t, z_nl, z_lin = run_linear_vs_nonlinear_demo(params, z0)
    
    # Option 2: Real-time interactive control 
    animate_realtime_control(params, z0, max_time=30.0)
