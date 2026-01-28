import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle, Circle, FancyArrow
from matplotlib.widgets import Button
disturbance = {"force": 0.0}

def animate_with_kalman(t, true_states, filtered_states,
                        controls=None, disturbances=None,control_mode=None, params=None, z0=None):
    if control_mode is None:
        control_mode = {"mode": "LQR"}
    # FIGURE + GRID
    fig = plt.figure(figsize=(16, 10))
    gs = fig.add_gridspec(3, 4, hspace=0.35, wspace=0.3)

    ax_anim_true = fig.add_subplot(gs[0, 0:2])
    ax_anim_filt = fig.add_subplot(gs[0, 2:4])
    ax_info      = ax_anim_filt.inset_axes([-0.25, 0.3, 1, 0.3])

    #  SWITCH BUTTON 
    ax_button = fig.add_axes([0.82, 0.92, 0.12, 0.05])  
    other_mode = "PID" if control_mode["mode"] == "LQR" else "LQR"
    btn_mode = Button(ax_button, f"Switch to mode: {other_mode}")

    def toggle_mode(event):
        from demos import kalman_demo_PID, kalman_demo_LQR

        # flip mode
        if control_mode["mode"] == "LQR":
            control_mode["mode"] = "PID"
            plt.close("all")
            btn_mode.label.set_text("Switch to mode: LQR")
            kalman_demo_PID(params, z0)
        else:
            control_mode["mode"] = "LQR"
            plt.close("all")
            btn_mode.label.set_text("Switch to mode: PID")
            kalman_demo_LQR(params, z0)

    btn_mode.on_clicked(toggle_mode)

    # DISTURBANCE BUTTONS
    ax_d1 = fig.add_axes([0.82, 0.86, 0.12, 0.04])
    ax_d2 = fig.add_axes([0.82, 0.81, 0.12, 0.04])

    btn_push = Button(ax_d1, "+1 N Disturbance")
    btn_pull = Button(ax_d2, "-1 N Disturbance")

    def apply_plus(event):
        disturbance["force"] = 1.0

    def apply_minus(event):
        disturbance["force"] = -1.0

    btn_push.on_clicked(apply_plus)
    btn_pull.on_clicked(apply_minus)



    ax_pos   = fig.add_subplot(gs[1, 0])
    ax_angle = fig.add_subplot(gs[1, 1])
    ax_vel   = fig.add_subplot(gs[1, 2])

    ax_control = fig.add_subplot(gs[1, 3]) 
    line_control, = ax_control.plot([], [], 'r-', lw=2, label='Control')
    ax_control.set(title="Control Action", xlabel="Time (s)", ylabel="Force (N)",
                xlim=(0, t[-1]), ylim=(min(controls)*1.1 if controls is not None else -5, 
                                        max(controls)*1.1 if controls is not None else 5))
    ax_control.grid(True, alpha=0.3)
    ax_control.legend()


    # ANIMATION AXES SETUP
    for ax, title in [(ax_anim_true, "True system"),
                      (ax_anim_filt, "Kalman filtered")]:
        ax.set_aspect("equal")
        ax.set_xlim(-3, 3)
        ax.set_ylim(-0.5, 1.5)
        ax.grid(True, alpha=0.3)
        ax.set_title(title, fontweight="bold")

    ax_info.axis("off")

    # PENDULUM GEOMETRY
    L = 0.3
    pivot_y = 0.15

    # TRUE pendulum
    cart_t = Rectangle((-0.15, 0), 0.3, 0.15, fc="white", ec="blue", lw=1.5)
    rod_t, = ax_anim_true.plot([], [], "k-", lw=4)
    bob_t = Circle((0, 0), 0.05, color="red")
    ax_anim_true.add_patch(cart_t)
    ax_anim_true.add_patch(bob_t)

    # FILTERED pendulum
    cart_f = Rectangle((-0.15, 0), 0.3, 0.15, fc="white", ec="green", lw=1.5)
    rod_f, = ax_anim_filt.plot([], [], "k-", lw=4)
    bob_f = Circle((0, 0), 0.05, color="green")
    ax_anim_filt.add_patch(cart_f)
    ax_anim_filt.add_patch(bob_f)

    control_arrow = None
    dist_arrow    = None

    # TIME PLOTS
    line_pos_true, = ax_pos.plot([], [], "b-",  lw=2, label="True")
    line_pos_filt, = ax_pos.plot([], [], "g--", lw=2, label="Filtered")

    line_ang_true, = ax_angle.plot([], [], "b-",  lw=2, label="True")
    line_ang_filt, = ax_angle.plot([], [], "g--", lw=2, label="Filtered")

    line_vel_true, = ax_vel.plot([], [], "b-", lw=2, label="Cart vel")
    line_avel_true,= ax_vel.plot([], [], "r-", lw=2, label="Angular vel")

    # Plot formatting
    ax_pos.set(title="Cart position", xlabel="Time (s)", ylabel="m",
               xlim=(0, t[-1]), ylim=(-3, 3))
    ax_pos.grid(True, alpha=0.3)
    ax_pos.legend()

    ax_angle.set(title="Pendulum angle", xlabel="Time (s)", ylabel="deg",
                 xlim=(0, t[-1]), ylim=(-90, 90))
    ax_angle.grid(True, alpha=0.3)
    ax_angle.legend()

    ax_vel.set(title="Velocities", xlabel="Time (s)",
               xlim=(0, t[-1]), ylim=(-3, 3))
    ax_vel.grid(True, alpha=0.3)
    ax_vel.legend()

    # HELPER
    def draw_pendulum(cart, rod, bob, state):
        x, theta = state[0], state[2]
        cart.set_xy((x - 0.15, 0))
        bx = x + L*np.sin(theta)
        by = pivot_y + L*np.cos(theta)
        rod.set_data([x, bx], [pivot_y, by])
        bob.center = (bx, by)

    # INIT
    def init():
        return (line_pos_true, line_pos_filt,
                line_ang_true, line_ang_filt,
                line_vel_true, line_avel_true,
                cart_t, rod_t, bob_t,
                cart_f, rod_f, bob_f, line_control)

    # UPDATE
    def update(i):
        # Apply disturbance if any
        if disturbance["force"] != 0 and controls is not None:
            controls[i] += disturbance["force"]   # add disturbance to current control
            disturbance["force"] *= 0.98          # decay over time

        # Draw pendulums
        draw_pendulum(cart_t, rod_t, bob_t, true_states[i])
        draw_pendulum(cart_f, rod_f, bob_f, filtered_states[i])

        # Update control plot
        if controls is not None:
            line_control.set_data(t[:i+1], controls[:i+1])

        # Update time plots
        line_pos_true.set_data(t[:i+1], true_states[:i+1, 0])
        line_pos_filt.set_data(t[:i+1], filtered_states[:i+1, 0])
        line_ang_true.set_data(t[:i+1], np.rad2deg(true_states[:i+1, 2]))
        line_ang_filt.set_data(t[:i+1], np.rad2deg(filtered_states[:i+1, 2]))
        line_vel_true.set_data(t[:i+1], true_states[:i+1, 1])
        line_avel_true.set_data(t[:i+1], true_states[:i+1, 3])

        # Yellow info box
        ax_info.clear()
        ax_info.axis("off")
        theta = true_states[i, 2]
        ax_info.text(
            0.05, 0.95,
            f"TIME: {t[i]:.2f} s\n"
            f"━━━━━━━━━━━━━━\n"
            f"CONTROL MODE\n  {control_mode['mode']}\n"
            f"━━━━━━━━━━━━━━\n"
            f"CART\n  x = {true_states[i,0 ]:.3f} m\n  ẋ = {true_states[i,1 ]:.3f} m/s\n\n"
            f"PENDULUM\n  θ = {np.rad2deg(theta):.2f}°\n  ω = {true_states[i,3 ]:.3f} rad/s",
            transform=ax_info.transAxes,
            fontfamily="monospace",
            fontsize=10,
            bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.85)
        )

        return (line_pos_true, line_pos_filt,
                line_ang_true, line_ang_filt,
                line_vel_true, line_avel_true,
                cart_t, rod_t, bob_t,
                cart_f, rod_f, bob_f, line_control)


    # START ANIMATION
    step = max(1, len(t)//500)
    ani = FuncAnimation(fig, update,
                        frames=range(0, len(t), step),
                        init_func=init,
                        interval=20,
                        blit=False)

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
