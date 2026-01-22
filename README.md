# Inverted Pendulum Simulation

Nonlinear dynamics, linearization, and Kalman filtering for cart-pole system.

## Setup
```bash
pip install numpy matplotlib
```

## File Structure

- `dynamics.py` - Nonlinear and linearized dynamics models
- `kalman_filter.py` - Kalman filter implementation with noisy sensors
- `visualisation.py` - Animation and plotting functions
- `demos.py` - Demo functions for testing
- `main.py` - Run simulations here

## Usage

### Run Linear vs Nonlinear Comparison
```python
from demos import run_linear_vs_nonlinear_demo
import numpy as np

params = {
    "M": 1.0,      # Cart mass (kg)
    "m": 0.2,      # Pendulum mass (kg)
    "l": 0.3,      # Pendulum length (m)
    "I": 0.006,    # Pendulum inertia (kg⋅m²)
    "g": 9.81,     # Gravity (m/s²)
    "b_x": 0.1,    # Cart damping
    "b_theta": 0.05 # Pendulum damping
}

z0 = [0, 0, np.deg2rad(5), 0]  # [x, x_dot, theta, theta_dot]
t, z_nl, z_lin = run_linear_vs_nonlinear_demo(params, z0)
```

### Run Kalman Filter Demo
```python
from demos import run_kalman_demo

run_kalman_demo(params)
```

## State Vector

All functions use state vector: `[x, x_dot, theta, theta_dot]`

- `x` - Cart position (m)
- `x_dot` - Cart velocity (m/s)
- `theta` - Pendulum angle from vertical (rad)
- `theta_dot` - Pendulum angular velocity (rad/s)

## Key Functions

### `simulate(f, z0, t0, tf, dt, u, *args)`
RK4 integration of dynamics function `f`

### `nonlinear_dynamics(t, z, u, p)`
Full nonlinear equations of motion

### `linearised_state_space(p)`
Returns A, B matrices for linearized system around upright position

### `linearised_dynamics(t, z, u, A, B)`
Linear state-space dynamics

## Notes

- Linearization valid for small angles (< ~20°)
- Inverted pendulum is unstable (positive eigenvalue ~4.2)
- Use `dt=0.001` for accurate simulation
