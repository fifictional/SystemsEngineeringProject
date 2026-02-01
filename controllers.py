import numpy as np
from scipy.linalg import solve_continuous_are

class PID_controller:
    def __init__(self, Kp=50.0, Ki=0.0, Kd=1.0, N=10):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.N = N
        
        # Internal state
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_filtered_deriv = 0.0
        
    def step(self, setpoint, measurement, dt):
        # Calculate error
        error = setpoint - measurement
        
        # Proportional term
        P = self.Kp * error
        
        # Integral term
        self.integral += error * dt
        I = self.Ki * self.integral
        
        # Derivative term with low-pass filter
        raw_derivative = (error - self.prev_error) / dt
        filtered_derivative = (self.N * raw_derivative + self.prev_filtered_deriv) / (1 + self.N)
        D = self.Kd * filtered_derivative
        
        # Update state
        self.prev_error = error
        self.prev_filtered_deriv = filtered_derivative
        
        # Total control output
        u = P + I + D
        
        return u
    
    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_filtered_deriv = 0.0


class LQRController:
    def __init__(self, params, Q=None, R=None, u_max=20.0):
        from dynamics import linearised_state_space
        
        self.params = params
        self.u_max = u_max
        
        # Default cost matrices
        if Q is None:
            Q = np.diag([10.0, 1.0, 200.0, 5.0])
        if R is None:
            R = np.array([[2.0]])
            
        self.Q = Q
        self.R = R
        
        # Get linearized system matrices
        self.A, self.B = linearised_state_space(params)
        
        # Solve Riccati equation for optimal gain
        self.update_gains()
    
    def update_gains(self):
        try:
            P = solve_continuous_are(self.A, self.B, self.Q, self.R)
            self.K = np.linalg.inv(self.R) @ self.B.T @ P
        except Exception as e:
            print(f"Warning: LQR gain computation failed: {e}")
            self.K = np.zeros((1, 4))
    
    def compute_control(self, x_current, x_ref):
        # State error
        x_error = x_current - x_ref
        
        # Optimal control: u = -K * x_error
        u = -self.K @ x_error
        u = float(u[0]) if hasattr(u, '__iter__') else float(u)
        
        # Saturation
        u = np.clip(u, -self.u_max, self.u_max)
        
        return u
