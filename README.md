# Inverted Pendulum — Simulation and Firmware

Nonlinear dynamics, linearisation, PID/LQR control, and Kalman filtering for a cart–pole system, plus Arduino sketches for the physical prototype.

## Setup

```bash
pip install numpy matplotlib scipy
```

SciPy is required for LQR (`solve_continuous_are`).

## Repository layout

| Path | Purpose |
|------|---------|
| `dynamics.py` | Nonlinear and linearised dynamics, RK4 integration |
| `controllers.py` | PID and LQR controllers |
| `kalman_filter.py` | Extended Kalman filter and noisy sensor model |
| `demos.py` | Closed-loop demos (regulation, 2 m sprint, disturbances) |
| `visualisation.py` | Plots, batch animations, real-time interactive UI |
| `main.py` | Default entry point for the simulator |
| `simulation.py` | Legacy/commented; not used by `main.py` |
| `ArduinoCode/` | MCU firmware (balance, sprint tests, motor/encoder tests) |

## Quick start (simulation)

From the project root:

```bash
python main.py
```

This opens the **real-time** interactive demo (`animate_realtime_control`): PID or LQR, sliders for tuning, disturbance and reset. Default horizon is 30 s; `params` and `z0` are set in `main.py`.

## Programmatic usage

### Linear vs nonlinear comparison

```python
from demos import run_linear_vs_nonlinear_demo
import numpy as np

params = {
    "M": 1.0,
    "m": 0.2,
    "l": 0.3,
    "I": 0.006,
    "g": 9.81,
    "b_x": 0.1,
    "b_theta": 0.05,
}

z0 = [0, 0, np.deg2rad(5), 0]  # [x, x_dot, theta, theta_dot]
t, z_nl, z_lin = run_linear_vs_nonlinear_demo(params, z0)
```

### Kalman-only demo

```python
from demos import run_kalman_demo

run_kalman_demo(params)
```

Other batch demos (`kalman_demo_PID`, `kalman_demo_LQR`, `kalman_demo_*_move2m`, etc.) are imported from `demos` — see `main.py` comments for examples.

## State vector

All dynamics use:

`[x, x_dot, theta, theta_dot]`

- `x` — cart position (m)  
- `x_dot` — cart velocity (m/s)  
- `theta` — pendulum angle from vertical (rad), upright at 0  
- `theta_dot` — angular velocity (rad/s)

## Key API (Python)

| Symbol | Role |
|--------|------|
| `simulate(f, z0, t0, tf, dt, u, *args)` | RK4 integration of dynamics `f` |
| `nonlinear_dynamics(t, z, u, p)` | Full nonlinear equations |
| `linearised_state_space(p)` | Continuous-time `A`, `B` around upright |
| `linearised_dynamics(t, z, u, A, B)` | Linear state-space dynamics |

## Hardware

Firmware under `ArduinoCode/` (e.g. `PIDBalance.ino`, `LQRBalance.ino`, motor/encoder tests) targets the on-cart microcontroller; it is separate from the Python simulator.

## Notes

- Small-angle linearisation is used for LQR and analysis; nonlinear simulation uses the full model.
- For open-loop comparison, `dt=0.001` in `run_linear_vs_nonlinear_demo` gives a tight match; closed-loop demos often use `dt=0.02`.
