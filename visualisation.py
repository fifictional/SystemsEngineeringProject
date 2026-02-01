import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle, Circle, FancyArrow
from matplotlib.widgets import Button
disturbance_force = 0.0
from demos import *

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

    btn_push = Button(ax_d1, "+15 N Disturbance")
    btn_pull = Button(ax_d2, "-15 N Disturbance")

    def apply_disturbance(force):
        plt.close('all')  # close current figure
        if control_mode["mode"] == "PID":
            from demos import kalman_demo_PID
            kalman_demo_PID(params, z0, disturbance_force=force)
        else:
            from demos import kalman_demo_LQR
            kalman_demo_LQR(params, z0, disturbance_force=force)

    btn_push.on_clicked(lambda event: apply_disturbance(15.0))
    btn_pull.on_clicked(lambda event: apply_disturbance(-15.0))

        

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
    for ax, title in [(ax_anim_true, "Noiseless system"),
                      (ax_anim_filt, "Noisy + Kalman filtered")]:
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


# ========================
# REAL TIME VISUALIZATION
# ========================
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle, Circle
from matplotlib.widgets import Slider, Button, RadioButtons
import numpy as np

def animate_realtime_control(params, z0, max_time=20.0):
    from dynamics import nonlinear_dynamics, rk4_step
    from kalman_filter import Sensor, KalmanFilter
    from controllers import PID_controller, LQRController
    
    # Simulation settings
    dt = 0.02  # Time step
    L = params["l"]
    cart_w = 0.3
    cart_h = 0.15
    
    # Initialize state
    z = np.array(z0, dtype=float)
    t_current = 0.0
    
    # Initialize sensor and Kalman filter
    sensor = Sensor(pos_std=0.01, angle_std=0.05)
    kf = KalmanFilter(dt)
    kf.x = np.array([0.0, 0.0, np.deg2rad(10.0), 0.0])
    
    # Initialize controllers
    pid = PID_controller(Kp=55, Ki=0.001, Kd=1.5, N=10)
    Q = np.diag([10.0, 1.0, 200.0, 5.0])
    R = np.array([[2.0]])
    lqr = LQRController(params, Q=Q, R=R, u_max=20.0)
    
    # Control mode: 'PID' or 'LQR'
    control_mode = {'current': 'PID'}
    disturbance = {'force': 0.0, 'active': False}
    
    # History for plotting
    history = {
        't': [],
        'x': [],
        'theta': [],
        'u': [],
        'x_est': [],
        'theta_est': []
    }
    fig = plt.figure(figsize=(14, 10))
    
    ax_anim = plt.subplot(3, 2, (1, 2))
    ax_anim.set_xlim(-3, 3)
    ax_anim.set_ylim(-0.3, 1.0)
    ax_anim.set_aspect('equal', adjustable='box')
    ax_anim.grid(True)
    ax_anim.set_title('Cart-Pole System (Real-time)', fontsize=14, fontweight='bold')
    
    # Ground line
    ax_anim.plot([-3, 3], [0, 0], 'k-', lw=2)
    
    # Cart
    cart = Rectangle((z[0] - cart_w/2, 0), cart_w, cart_h, fill=True, 
                     facecolor='steelblue', edgecolor='black', lw=2)
    ax_anim.add_patch(cart)
    
    # Pendulum
    pivot_y = cart_h
    rod, = ax_anim.plot([], [], 'r-', lw=4)
    bob = Circle((0, 0), 0.05, fill=True, color='darkred')
    ax_anim.add_patch(bob)
    
    # Info text
    info_text = ax_anim.text(0.02, 0.95, '', transform=ax_anim.transAxes, 
                             fontsize=10, verticalalignment='top',
                             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    # History plots 
    ax_pos = plt.subplot(3, 2, 3)
    ax_pos.set_xlabel('Time (s)')
    ax_pos.set_ylabel('Cart Position (m)')
    ax_pos.grid(True)
    line_pos, = ax_pos.plot([], [], 'b-', lw=2, label='True')
    line_pos_est, = ax_pos.plot([], [], 'g--', lw=1.5, label='Estimated')
    ax_pos.legend()
    ax_pos.set_xlim(0, max_time)
    ax_pos.set_ylim(-2, 2)
    
    ax_angle = plt.subplot(3, 2, 4)
    ax_angle.set_xlabel('Time (s)')
    ax_angle.set_ylabel('Angle (deg)')
    ax_angle.grid(True)
    line_angle, = ax_angle.plot([], [], 'b-', lw=2, label='True')
    line_angle_est, = ax_angle.plot([], [], 'g--', lw=1.5, label='Estimated')
    ax_angle.legend()
    ax_angle.set_xlim(0, max_time)
    ax_angle.set_ylim(-30, 30)
    
    ax_control = plt.subplot(3, 2, 5)
    ax_control.set_xlabel('Time (s)')
    ax_control.set_ylabel('Control Force (N)')
    ax_control.grid(True)
    line_control, = ax_control.plot([], [], 'r-', lw=2)
    ax_control.set_xlim(0, max_time)
    ax_control.set_ylim(-25, 25)
    
    # Control panel 
    ax_panel = plt.subplot(3, 2, 6)
    ax_panel.axis('off')
    
    # PID Sliders
    ax_pid_kp = plt.axes([0.60, 0.30, 0.30, 0.015])
    ax_pid_ki = plt.axes([0.60, 0.26, 0.30, 0.015])
    ax_pid_kd = plt.axes([0.60, 0.22, 0.30, 0.015])
    
    slider_pid_kp = Slider(ax_pid_kp, 'PID Kp', 0, 200, valinit=55, valstep=1)
    slider_pid_ki = Slider(ax_pid_ki, 'PID Ki', 0, 1, valinit=0.001, valstep=0.001)
    slider_pid_kd = Slider(ax_pid_kd, 'PID Kd', 0, 10, valinit=1.5, valstep=0.1)
    
    # LQR Sliders
    ax_lqr_qpos = plt.axes([0.60, 0.30, 0.30, 0.015])
    ax_lqr_qvel = plt.axes([0.60, 0.26, 0.30, 0.015])
    ax_lqr_qang = plt.axes([0.60, 0.22, 0.30, 0.015])
    ax_lqr_qangvel = plt.axes([0.60, 0.18, 0.30, 0.015])
    ax_lqr_r = plt.axes([0.60, 0.14, 0.30, 0.015])
    
    slider_lqr_qpos = Slider(ax_lqr_qpos, 'Q_pos', 0, 50, valinit=10, valstep=1)
    slider_lqr_qvel = Slider(ax_lqr_qvel, 'Q_vel', 0, 10, valinit=1, valstep=0.1)
    slider_lqr_qang = Slider(ax_lqr_qang, 'Q_angle', 0, 500, valinit=250, valstep=5)
    slider_lqr_qangvel = Slider(ax_lqr_qangvel, 'Q_ang_vel', 0, 20, valinit=2, valstep=0.5)
    slider_lqr_r = Slider(ax_lqr_r, 'R', 0.1, 10, valinit=2, valstep=0.1)
    
    # Initially hide LQR sliders
    ax_lqr_qpos.set_visible(False)
    ax_lqr_qvel.set_visible(False)
    ax_lqr_qang.set_visible(False)
    ax_lqr_qangvel.set_visible(False)
    ax_lqr_r.set_visible(False)
    
    # Store LQR parameters
    lqr_params = {
        'Q_pos': 10.0,
        'Q_vel': 1.0,
        'Q_angle': 250.0,
        'Q_angular_vel': 2.0,
        'R': 2.0
    }
    
    # Radio buttons for controller selection
    ax_radio = plt.axes([0.62, 0.40, 0.12, 0.10])
    radio = RadioButtons(ax_radio, ('PID', 'LQR'), active=0)
    
    # Disturbance button
    ax_button = plt.axes([0.75, 0.42, 0.12, 0.04])
    button_dist = Button(ax_button, 'Apply Disturbance')
    
    # Reset button
    ax_reset = plt.axes([0.75, 0.37, 0.12, 0.04])
    button_reset = Button(ax_reset, 'Reset System')
    
    # Callback functions
    def update_pid_params(val):
        # pdate PID controller parameters
        pid.Kp = slider_pid_kp.val
        pid.Ki = slider_pid_ki.val
        pid.Kd = slider_pid_kd.val
        pid.integral = 0
        print(f"PID: Kp={pid.Kp:.1f}, Ki={pid.Ki:.3f}, Kd={pid.Kd:.1f}")
    
    slider_pid_kp.on_changed(update_pid_params)
    slider_pid_ki.on_changed(update_pid_params)
    slider_pid_kd.on_changed(update_pid_params)
    
    def update_lqr_params(val):
        # Update LQR controller parameters
        lqr_params['Q_pos'] = slider_lqr_qpos.val
        lqr_params['Q_vel'] = slider_lqr_qvel.val
        lqr_params['Q_angle'] = slider_lqr_qang.val
        lqr_params['Q_angular_vel'] = slider_lqr_qangvel.val
        lqr_params['R'] = slider_lqr_r.val
        
        # Rebuild Q matrix
        lqr.Q = np.diag([
            lqr_params['Q_pos'],
            lqr_params['Q_vel'],
            lqr_params['Q_angle'],
            lqr_params['Q_angular_vel']
        ])
        
        # Update R matrix
        lqr.R = np.array([[lqr_params['R']]])
        
        # Recompute optimal gain
        lqr.update_gains()
        
        print(f"LQR: Q=[{lqr_params['Q_pos']:.1f}, {lqr_params['Q_vel']:.1f}, "
              f"{lqr_params['Q_angle']:.1f}, {lqr_params['Q_angular_vel']:.1f}], R={lqr_params['R']:.2f}")
    
    slider_lqr_qpos.on_changed(update_lqr_params)
    slider_lqr_qvel.on_changed(update_lqr_params)
    slider_lqr_qang.on_changed(update_lqr_params)
    slider_lqr_qangvel.on_changed(update_lqr_params)
    slider_lqr_r.on_changed(update_lqr_params)
    
    def switch_controller(label):
        """Switch between PID and LQR control"""
        control_mode['current'] = label
        print(f"Controller switched to: {label}")
        
        if label == 'PID':
            # Show PID sliders, hide LQR sliders
            ax_pid_kp.set_visible(True)
            ax_pid_ki.set_visible(True)
            ax_pid_kd.set_visible(True)
            
            ax_lqr_qpos.set_visible(False)
            ax_lqr_qvel.set_visible(False)
            ax_lqr_qang.set_visible(False)
            ax_lqr_qangvel.set_visible(False)
            ax_lqr_r.set_visible(False)
            
            pid.reset()
            print(f"   PID mode: Kp={pid.Kp}, Ki={pid.Ki}, Kd={pid.Kd}")
            
        else:  # LQR
            # Hide PID sliders, show LQR sliders
            ax_pid_kp.set_visible(False)
            ax_pid_ki.set_visible(False)
            ax_pid_kd.set_visible(False)
            
            ax_lqr_qpos.set_visible(True)
            ax_lqr_qvel.set_visible(True)
            ax_lqr_qang.set_visible(True)
            ax_lqr_qangvel.set_visible(True)
            ax_lqr_r.set_visible(True)
            
            print(f" LQR mode: Q=[{lqr_params['Q_pos']}, {lqr_params['Q_vel']}, "
                  f"{lqr_params['Q_angle']}, {lqr_params['Q_angular_vel']}], R={lqr_params['R']}")
        
        plt.draw()
    
    radio.on_clicked(switch_controller)
    
    def apply_disturbance(event):
        """Apply a temporary disturbance force"""
        disturbance['force'] = 10.0
        disturbance['active'] = True
        disturbance['duration'] = 0.4
        disturbance['start_time'] = t_current
        print(f" Disturbance: {disturbance['force']} N for {disturbance['duration']}s")
    
    button_dist.on_clicked(apply_disturbance)
    
    def reset_system(event):
        """Reset system to initial state"""
        nonlocal z, t_current, kf
        z = np.array(z0, dtype=float)
        t_current = 0.0
        kf.x = np.array([0.0, 0.0, np.deg2rad(10.0), 0.0])
        pid.reset()
        history['t'].clear()
        history['x'].clear()
        history['theta'].clear()
        history['u'].clear()
        history['x_est'].clear()
        history['theta_est'].clear()
        print("System reset to initial state")
    
    button_reset.on_clicked(reset_system)
    
    # Animation update function
    def update(frame):
        nonlocal z, t_current
        
        if t_current >= max_time:
            return rod, bob, cart, info_text
        
        # Get noisy measurement
        y = sensor.add_noise(z[[0, 2]])
        
        # Update Kalman filter
        kf.update(y)
        
        # Compute control input based on current mode
        if control_mode['current'] == 'PID':
            theta_ref = 0.0
            theta_hat = kf.x[2]
            u = -pid.step(theta_ref, theta_hat, dt)
        else:  # LQR
            x_ref = np.zeros(4)
            x_for_lqr = kf.x.copy()
            x_for_lqr[2] = (x_for_lqr[2] + np.pi) % (2.0 * np.pi) - np.pi  # Wrap angle
            u = lqr.compute_control(x_for_lqr, x_ref)
        
        # Apply disturbance if active
        if disturbance['active']:
            if t_current - disturbance['start_time'] < disturbance['duration']:
                u += disturbance['force']
            else:
                disturbance['active'] = False
        
        # Predict next state (Kalman)
        kf.predict(u=u)
        
        # Propagate true system dynamics
        z = rk4_step(nonlinear_dynamics, t_current, z, dt, u, params)
        
        # Update history
        history['t'].append(t_current)
        history['x'].append(z[0])
        history['theta'].append(np.rad2deg(z[2]))
        history['u'].append(u)
        history['x_est'].append(kf.x[0])
        history['theta_est'].append(np.rad2deg(kf.x[2]))
        
        # Update animation elements
        cart.set_xy((z[0] - cart_w/2, 0))
        
        px = z[0]
        py = pivot_y
        bx = px + L * np.sin(z[2])
        by = py + L * np.cos(z[2])
        
        rod.set_data([px, bx], [py, by])
        bob.center = (bx, by)
        
        # Update info text
        info_text.set_text(
            f"Time: {t_current:.2f} s\n"
            f"Mode: {control_mode['current']}\n"
            f"θ: {np.rad2deg(z[2]):.1f}°\n"
            f"x: {z[0]:.2f} m\n"
            f"u: {u:.1f} N"
        )
        
        # Update history plots
        line_pos.set_data(history['t'], history['x'])
        line_pos_est.set_data(history['t'], history['x_est'])
        line_angle.set_data(history['t'], history['theta'])
        line_angle_est.set_data(history['t'], history['theta_est'])
        line_control.set_data(history['t'], history['u'])
        
        # Auto-scale plots
        if t_current > 1.0:
            ax_pos.set_xlim(max(0, t_current - 10), t_current + 1)
            ax_angle.set_xlim(max(0, t_current - 10), t_current + 1)
            ax_control.set_xlim(max(0, t_current - 10), t_current + 1)
        
        t_current += dt
        
        return rod, bob, cart, info_text, line_pos, line_angle, line_control
    
    # Create animation (interval in ms)
    ani = FuncAnimation(fig, update, interval=dt*1000, blit=False, cache_frame_data=False)
    
    plt.tight_layout()
    plt.show()
    
    return ani
