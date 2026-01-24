import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle, Circle
import numpy as np

def animate_with_kalman(t, true_states, filtered_states):
    """Animate both true and filtered states"""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
    
    # Left: True state
    ax1.set_aspect('equal')
    ax1.set_xlim(-1, 1)
    ax1.set_ylim(-0.5, 1.5)
    ax1.grid(True)
    ax1.set_title('True Pendulum (with noise in simulation)')
    
    # Right: Filtered state
    ax2.set_aspect('equal')
    ax2.set_xlim(-1, 1)
    ax2.set_ylim(-0.5, 1.5)
    ax2.grid(True)
    ax2.set_title('Kalman Filter Estimate')
    
    # Initialize true pendulum
    cart_true = Rectangle((-0.15, 0), 0.3, 0.15, fill=False, lw=2, color='blue')
    ax1.add_patch(cart_true)
    rod_true, = ax1.plot([], [], 'k-', lw=3)
    bob_true = Circle((0, 0), 0.03, fill=True, color='red')
    ax1.add_patch(bob_true)
    time_text_true = ax1.text(0.02, 0.95, "", transform=ax1.transAxes)
    
    # Initialize filtered pendulum
    cart_filtered = Rectangle((-0.15, 0), 0.3, 0.15, fill=False, lw=2, color='green')
    ax2.add_patch(cart_filtered)
    rod_filtered, = ax2.plot([], [], 'k-', lw=3)
    bob_filtered = Circle((0, 0), 0.03, fill=True, color='orange')
    ax2.add_patch(bob_filtered)
    time_text_filtered = ax2.text(0.02, 0.95, "", transform=ax2.transAxes)
    
    def init():
        rod_true.set_data([], [])
        bob_true.center = (0, 0.15)
        rod_filtered.set_data([], [])
        bob_filtered.center = (0, 0.15)
        time_text_true.set_text("")
        time_text_filtered.set_text("")
        return rod_true, bob_true, cart_true, rod_filtered, bob_filtered, cart_filtered, time_text_true, time_text_filtered
    
    def update(i):
        L = 0.3
        pivot_y = 0.15
        
        # Update true pendulum
        x_true = true_states[i, 0]
        theta_true = true_states[i, 2]
        cart_true.set_xy((x_true - 0.15, 0))
        bx_true = x_true + L * np.sin(theta_true)
        by_true = pivot_y + L * np.cos(theta_true)
        rod_true.set_data([x_true, bx_true], [pivot_y, by_true])
        bob_true.center = (bx_true, by_true)
        time_text_true.set_text(f"True\nTime: {t[i]:.2f}s\nAngle: {np.rad2deg(theta_true-np.pi):.1f}°")
        
        # Update filtered pendulum
        x_filtered = filtered_states[i, 0]
        theta_filtered = filtered_states[i, 2]
        cart_filtered.set_xy((x_filtered - 0.15, 0))
        bx_filtered = x_filtered + L * np.sin(theta_filtered)
        by_filtered = pivot_y + L * np.cos(theta_filtered)
        rod_filtered.set_data([x_filtered, bx_filtered], [pivot_y, by_filtered])
        bob_filtered.center = (bx_filtered, by_filtered)
        time_text_filtered.set_text(f"Kalman Filter\nTime: {t[i]:.2f}s\nAngle: {np.rad2deg(theta_filtered-np.pi):.1f}°")
        
        return rod_true, bob_true, cart_true, rod_filtered, bob_filtered, cart_filtered, time_text_true, time_text_filtered
    
    # Animate every 5th frame for speed
    step = max(1, len(t)//200)
    ani = FuncAnimation(fig, update, frames=range(0, len(t), step),
                        init_func=init, blit=True, interval=50)
    
    plt.tight_layout()
    plt.show()
    return ani

def plot_kalman_results(t, true_states, noisy_measurements, filtered_states):
    """Simple plot of Kalman filter results"""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    
    # Plot 1: Angle
    true_angles = np.rad2deg(true_states[:, 2] - np.pi)
    noisy_angles = np.rad2deg(noisy_measurements[:, 1] - np.pi)
    filtered_angles = np.rad2deg(filtered_states[:, 2] - np.pi)
    
    ax1.plot(t, true_angles, 'b-', linewidth=2, label='True Angle')
    ax1.plot(t, noisy_angles, 'r.', markersize=3, alpha=0.5, label='Noisy Measurements')
    ax1.plot(t, filtered_angles, 'g-', linewidth=2, label='Kalman Filter')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Angle from Upright (deg)')
    ax1.set_title('Kalman Filter: Angle Estimation with Noise')
    ax1.legend()
    ax1.grid(True)
    
    # Plot 2: Position
    ax2.plot(t, true_states[:, 0], 'b-', linewidth=2, label='True Position')
    ax2.plot(t, noisy_measurements[:, 0], 'r.', markersize=3, alpha=0.5, label='Noisy Measurements')
    ax2.plot(t, filtered_states[:, 0], 'g-', linewidth=2, label='Kalman Filter')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Cart Position (m)')
    ax2.set_title('Kalman Filter: Position Estimation with Noise')
    ax2.legend()
    ax2.grid(True)
    
    plt.tight_layout()
    plt.show()
    
    # Print some stats
    print("\n=== Kalman Filter Performance ===")
    final_angle_error = abs(true_angles[-1] - filtered_angles[-1])
    final_pos_error = abs(true_states[-1, 0] - filtered_states[-1, 0])
    print(f"Final angle error: {final_angle_error:.2f}°")
    print(f"Final position error: {final_pos_error*100:.1f} cm")
    print(f"Noise reduction (angle): {np.std(noisy_angles)/np.std(true_angles-filtered_angles):.1f}x")

def plot_pid_performance(t, theta_error):
    plt.figure()
    plt.plot(t, np.rad2deg(theta_error))
    plt.xlabel("Time (s)")
    plt.ylabel("Angle error (deg)")
    plt.title("Pendulum angle tracking error")
    plt.grid()
    plt.show()

def plot_lqr_performance(t, true_states, filtered_states, controls, x_ref=None):
    """
    Plot LQR performance with proper angle wrapping to avoid 360° jumps
    """
    if x_ref is None:
        x_ref = np.zeros(4)

    def wrap_angle(angle):
        """Wrap angle to (-pi, pi]"""
        return (angle + np.pi) % (2.0 * np.pi) - np.pi
    
    def angle_diff(a, b):
        """Smallest signed difference from b to a"""
        return wrap_angle(a - b)

    # Wrap angles
    theta_true_wrapped = np.array([wrap_angle(th) for th in true_states[:, 2]])
    theta_filt_wrapped = np.array([wrap_angle(th) for th in filtered_states[:, 2]])
    
    # Error with proper wrapping (avoid 360° jumps)
    theta_error = np.array([angle_diff(theta_true_wrapped[i], x_ref[2]) 
                           for i in range(len(t))])

    plt.figure(figsize=(12, 8))

    # Angles
    plt.subplot(3, 1, 1)
    plt.plot(t, np.rad2deg(theta_true_wrapped), 'b-', label="True", lw=1.5)
    plt.plot(t, np.rad2deg(theta_filt_wrapped), 'r--', label="KF Est.", lw=1.2)
    plt.axhline(y=np.rad2deg(x_ref[2]), color='g', ls=':', label="Target", lw=1.5)
    plt.ylabel("Angle (deg)")
    plt.title("LQR: Pendulum Angle")
    plt.legend()
    plt.grid(True)
    plt.ylim([-30, 30])

    # Error
    plt.subplot(3, 1, 2)
    plt.plot(t, np.rad2deg(theta_error), 'b-')
    plt.axhline(y=0, color='k', ls='--', lw=0.8)
    plt.ylabel("Error (deg)")
    plt.title("LQR: Tracking Error (wrapped)")
    plt.grid(True)
    plt.ylim([-30, 30])

    # Control
    plt.subplot(3, 1, 3)
    plt.plot(t, controls, 'r-')
    plt.axhline(y=0, color='k', ls='--', lw=0.8)
    plt.xlabel("Time (s)")
    plt.ylabel("Control (N)")
    plt.title("LQR: Control Input")
    plt.grid(True)

    plt.tight_layout()
    plt.show()
