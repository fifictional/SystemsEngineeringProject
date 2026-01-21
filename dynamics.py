import numpy as np

def state_cal(t, z, F):
    # Constants
    M = 1   # mass of cart
    m = 0.1 # mass of pendulum
    l = 0.3 # length to pendulum center of mass
    I = 0.05 # moment of inertia of pendulum about its center of mass
    g = 9.81 # acceleration due to gravity

    b_theta = 0.01 # damping coefficient for pendulum
    b_x = 0.1     # damping coefficient for cart

    rho = 1.225 # air density
    Cd = 0.5 # drag coeff
    A_pend = 0.01 # pendulum frontal area
    A_cart = 0.02 # cart frontal area

    x = z[0]
    x_dot = z[1]
    theta = z[2]
    theta_dot = z[3]

    # Air drag forces
    F_drag_cart = -0.5 * rho * Cd * A_cart * abs(x_dot) * x_dot
    F_drag_pend = -0.5 * rho * Cd * A_pend * abs(l*theta_dot) * (l*theta_dot)

    A = np.array([[M + m, m*l*np.cos(theta)],
                  [m*l*np.cos(theta), I + m*l**2]])

    b = np.array([F + F_drag_cart - b_x*x_dot + m*l*(theta_dot**2)*np.sin(theta),
                  m*g*l*np.sin(theta)-b_theta*theta_dot])
    
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