import numpy as np
from scipy.linalg import solve_continuous_are
from dynamics import linearised_state_space


class LQRController:
    def __init__(self, params, Q=None, R=None, u_max=None):

        # Linearised system
        self.A, self.B = linearised_state_space(params)
        self.Q = Q
        self.R = R
        self.u_max = u_max
        # Solve CARE P
        self.P = solve_continuous_are(self.A, self.B, self.Q, self.R)
        # LQR gain K calculated by using CARE
        self.K = np.linalg.inv(self.R) @ self.B.T @ self.P

    def compute_control(self, x_hat, x_ref=None):
        # compute control for feedback
        if x_ref is None:
            x_ref = np.zeros(4)
        # State error
        e = x_hat - x_ref

        # LQR control law defined -- finding u
        u = -self.K @ e
        u = float(u)

        # Optional saturation
        if self.u_max is not None:
            u = np.clip(u, -self.u_max, self.u_max)

        return u

class PID_controller:
    def __init__(self, Kp, Ki, Kd, N=10, integral_limit=None):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0
        self.prev_error = 0
        self.d_filt = 0
        self.N = N
        self.prev_measurement = None
        self.integral_limit = integral_limit

    def step(self, setpoint, measurement, dt):    
        error = setpoint - measurement

        self.integral += error * dt
        
        if self.integral_limit is not None:
            self.integral = np.clip(self.integral, -self.integral_limit, self.integral_limit)

        if self.prev_measurement is not None:
            de_dt = -(measurement - self.prev_measurement) / dt
        else:
            de_dt = 0
            
        Tf = 1 / self.N
        alpha = Tf / (Tf + dt)
        self.d_filt = alpha * self.d_filt + (1 - alpha) * de_dt

        output = self.Kp * error + self.Ki * self.integral + self.Kd * self.d_filt
        
        self.prev_error = error
        self.prev_measurement = measurement

        return output