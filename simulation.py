import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle, Circle

def state_cal(t, z, F):
    # Constants
    M = 1   # mass of cart
    m = 0.1 # mass of pendulum
    l = 0.3 # length to pendulum center of mass
    I = 0.05 # moment of inertia of pendulum about its center of mass
    g = 9.81 # acceleration due to gravity

    x = z[0]
    x_dot = z[1]
    theta = z[2]
    theta_dot = z[3]

    A = np.array([[M + m, m*l*np.cos(theta)],
                  [m*l*np.cos(theta), I + m*l**2]])

    b = np.array([F + m*l*(theta_dot**2)*np.sin(theta),
                  m*g*l*np.sin(theta)])
    
    x_ddot, theta_ddot = np.linalg.solve(A, b)

    # Computing state vectors
    dz1 = x_dot
    dz2 = x_ddot
    dz3 = theta_dot
    dz4 = theta_ddot

    dz = np.array([dz1, dz2, dz3, dz4])
    return dz

def step_rk(t,dt,z, F):
    # RK4 Method
    f = state_cal
    k1 = f(t, z, F)
    k2 = f(t + dt/2, z + dt/2*k1, F)
    k3 = f(t + dt/2, z + dt/2*k2, F)
    k4 = f(t + dt,   z + dt*k3, F)

    znext = z + dt/6*(k1 + 2*k2 + 2*k3 + k4)
    return znext

def solve_ivp(t0, tmax, dt, z0, F=0):
    # Set initial conditions
    t = np.array([t0])
    z = z0

    # Continue stepping until the end time is exceeded
    n=0
    while t[n] <= tmax:
        # Increment by one time step and append to the time axis
        t = np.append(t,t[-1]+dt)

        # Obtain next state vector using one step of RK method
        znext = step_rk(t[n], dt, z[:,n], F)
        
        # Append to the solution
        znext = znext[:,np.newaxis]
        z = np.append(z,znext,axis=1)
        
        n = n+1
    
    return t, z

def animate_cart_pendulum(t, z, L=0.3, cart_w=0.3, cart_h=0.15, pivot_h=0):
    x = z[0, :]
    theta = z[2, :]

    # --- Figure setup ---
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True)

    # Set axis limits based on motion range
    xpad = 1.0
    ax.set_xlim(np.min(x) - xpad, np.max(x) + xpad)
    ax.set_ylim(-0.3, 1.0)

    # Ground line
    ground, = ax.plot([np.min(x) - xpad, np.max(x) + xpad], [0, 0], lw=2)

    # Cart
    cart = Rectangle((x[0] - cart_w/2, 0), cart_w, cart_h, fill=False, lw=2)
    ax.add_patch(cart)

    # Pivot point height
    pivot_y = cart_h + pivot_h

    # Pendulum rod and bob
    rod, = ax.plot([], [], lw=3)
    bob = Circle((0, 0), 0.03, fill=True)
    ax.add_patch(bob)

    # Time text
    time_text = ax.text(0.02, 0.92, "", transform=ax.transAxes)

    def init():
        rod.set_data([], [])
        bob.center = (x[0], pivot_y)
        time_text.set_text("")
        return rod, bob, cart, time_text

    def update(i):
        # Cart position
        cart.set_xy((x[i] - cart_w/2, 0))

        # Pivot position
        px = x[i]
        py = pivot_y

        # Pendulum tip/bob position
        bx = px + L * np.sin(theta[i])
        by = py + L * np.cos(theta[i])

        # Update drawings
        rod.set_data([px, bx], [py, by])
        bob.center = (bx, by)
        time_text.set_text(f"t = {t[i]:.2f} s")

        return rod, bob, cart, time_text

    # Frame step to control speed
    step = max(1, len(t)//600) 
    ani = FuncAnimation(fig, update, frames=range(0, len(t), step),
                        init_func=init, blit=True, interval=20)

    plt.show()
    return ani

if __name__ == '__main__':
    
    # Start time, end time, and time step in seconds
    t0 = 0
    tmax = 120
    dt = 0.1
    F = 0 # change if simulating with external force

    # Set initial conditions
    theta = np.deg2rad(90) # 0 is top
    theta_dot = 0
    x = 0
    x_dot = 0
    z0 = np.array([[x], [x_dot], [theta], [theta_dot]])
    
    # Run the solver
    [t,z] = solve_ivp(t0,tmax,dt,z0, F)
    ani = animate_cart_pendulum(t, z, L=0.3)
