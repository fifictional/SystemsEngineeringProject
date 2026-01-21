import numpy as np
import matplotlib.pyplot as plt

# Import from your own modules
from dynamics import state_cal, step_rk, solve_ivp
from kalman_filter import Sensor, KalmanFilter
# from controllers import PIDController, LQRController
from visualisation import animate_with_kalman, plot_kalman_results

def simulate_with_noise_and_kalman():
    """Minimal working simulation with noise and Kalman filter"""
    dt = 0.02  # Slightly larger for faster simulation
    duration = 5  # Shorter duration for testing
    steps = int(duration/dt)
    
    # True state: start with 15° tilt from upright
    true_state = np.array([0, 0, np.pi + np.deg2rad(15), 0])
    
    # Initialize
    sensor = Sensor(angle_std=0.05, pos_std=0.01)  # More noise to see effect
    ekf = KalmanFilter(dt)
    
    # Initialize EKF with wrong estimate (to see correction)
    ekf.x = np.array([0.1, 0, np.pi + np.deg2rad(10), 0])  # 10° instead of 15°
    
    # Storage
    time_points = np.zeros(steps)
    true_states = np.zeros((steps, 4))
    noisy_measurements = np.zeros((steps, 2))
    filtered_states = np.zeros((steps, 4))
    
    current_time = 0
    
    for i in range(steps):
        time_points[i] = current_time
        true_states[i] = true_state.copy()
        
        # Get noisy measurement
        noisy_measurement = sensor.add_noise(true_state[[0, 2]])
        noisy_measurements[i] = noisy_measurement
        
        # Kalman filter update
        ekf.update(noisy_measurement)
        
        # Store filtered state
        filtered_states[i] = ekf.x.copy()
        
        # Kalman filter predict
        ekf.predict(u=0)
        
        # Simulate true dynamics forward (simplified Euler for now)
        dz = state_cal(current_time, true_state, F=0)
        true_state = true_state + dz * dt
        
        current_time += dt
    
    return time_points, true_states, noisy_measurements, filtered_states






if __name__ == '__main__':

    print("\nRunning simulation with sensor noise and Kalman filter...")
    t, true_states, noisy_measurements, filtered_states = simulate_with_noise_and_kalman()
    
    # plot results
    plot_kalman_results(t, true_states, noisy_measurements, filtered_states)
    
    # animate comparison
    print("\nAnimating true vs filtered states...")
    ani = animate_with_kalman(t, true_states, filtered_states)