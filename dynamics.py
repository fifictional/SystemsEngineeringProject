import numpy as np
import matplotlib.pyplot as plt

params = {
    "M": 1.0,
    "m": 0.2,
    "l": 0.3,
    "I": 0.006,
    "g": 9.81,
    "b_x": 0.1,
    "b_theta": 0.05  
}

def nonlinear_dynamics(t, z, u, p):
    x, x_dot, theta, theta_dot = z

    M = p["M"]
    m = p["m"]
    l = p["l"]
    I = p["I"]
    g = p["g"]
    b_x = p["b_x"]
    b_theta = p["b_theta"]

    A = np.array([
        [0, 1, 0, 0],
        [0, -b_x*(I + m*l**2)/denom, -(m**2 * g * l**2)/denom, b_theta*m*l/denom],  
        [0, 0, 0, 1],
        [0, b_x*m*l/denom, m*g*l*(M + m)/denom, -b_theta*(M + m)/denom]
    ])

    b = np.array([
        [0],
        [(I + m*l**2)/denom],  
        [0],
        [-m*l/denom] 
    ])

    x_ddot, theta_ddot = np.linalg.solve(A, b)

    return np.array([
        x_dot,
        x_ddot,
        theta_dot,
        theta_ddot
    ])


def linearised_state_space(p):
    M = p["M"]
    m = p["m"]
    l = p["l"]
    I = p["I"]
    g = p["g"]
    b_x = p["b_x"]     
    b_theta = p["b_theta"] 

    denom = (M + m) * (I + m * l**2) - (m * l)**2

    A = np.array([
        [0, 1, 0, 0],
        [0, -b_x*(I + m*l**2)/denom, (m**2 * g * l**2)/denom, b_theta*m*l/denom],
        [0, 0, 0, 1],
        [0, b_x*m*l/denom, m*g*l*(M + m)/denom, -b_theta*(M + m)/denom]
    ])

    B = np.array([
        [0],
        [(I + m*l**2)/denom],
        [0],
        [m*l/denom]
    ])

    return A, B


def linearised_dynamics(t, z, u, A, B):
    return A @ z + (B.flatten() * u)


def rk4_step(f, t, z, dt, u, *args):
    k1 = f(t, z, u, *args)
    k2 = f(t + dt/2, z + dt/2 * k1, u, *args)
    k3 = f(t + dt/2, z + dt/2 * k2, u, *args)
    k4 = f(t + dt, z + dt * k3, u, *args)
    return z + dt/6 * (k1 + 2*k2 + 2*k3 + k4)


def simulate(f, z0, t0, tf, dt, u, *args):
    t = np.arange(t0, tf, dt)
    z = np.zeros((len(z0), len(t)))
    z[:, 0] = z0

    for k in range(len(t) - 1):
        z[:, k+1] = rk4_step(f, t[k], z[:, k], dt, u, *args)

    return t, z

