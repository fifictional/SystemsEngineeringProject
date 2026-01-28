import numpy as np

class LinearisedPendulumModel:
    # linearised model around upright position (for kalman filter)
    def __init__(self, dt=0.01, M=1.0, m=0.1, l=0.3, g=9.81, b_x=0.1, b_theta=0.01):
        self.M = M
        self.m = m
        self.l = l
        self.g = g
        self.dt = dt

        self.b_x = b_x
        self.b_theta = b_theta

        self.M_total = self.M + self.m

    def compute_jacobian_F(self, state, u=0):
        # compute jacobian of nonlinear dynamics at current state 
        x_pos, x_dot, theta, theta_dot = state
        sin_theta = np.sin(theta)
        cos_theta = np.cos(theta)
        denom = self.M_total - self.m * cos_theta**2

        # initialise Jacobian matrix
        F = np.zeros((4,4))
        # row 1
        F[0,0] = 0
        F[0,1] = 1
        F[0,2] = 0
        F[0,3] = 0

        # row 2
        d_denom_dtheta = 2 * self.m * cos_theta * sin_theta
        # Terms for x_ddot
        term1 = u - self.b_x * x_dot
        term2 = self.m * self.l * theta_dot**2 * sin_theta
        numerator_x = term1 + term2

        # Derivatives of numerator
        dnum_dx = 0
        dnum_dxdot = -self.b_x
        dnum_dtheta = self.m * self.l * theta_dot**2 * cos_theta
        dnum_dthetadot = 2 * self.m * self.l * theta_dot * sin_theta
        
        # Compute F[1, :] using quotient rule
        F[1, 0] = 0  
        F[1, 1] = dnum_dxdot / denom
        F[1, 2] = (dnum_dtheta * denom - numerator_x * d_denom_dtheta) / denom**2
        F[1, 3] = dnum_dthetadot / denom
        
        #row 3
        F[2, 0] = 0
        F[2, 1] = 0
        F[2, 2] = 0
        F[2, 3] = 1

        # row4
        # Terms for θ_ddot
        numerator_theta = (self.M_total * self.g * sin_theta - 
                          self.m * self.l * theta_dot**2 * sin_theta * cos_theta - 
                          u * cos_theta - 
                          self.b_theta * theta_dot / self.l)
        
        # For small deviations from upright (θ ≈ π), sin(θ) ≈ -(θ - π), cos(θ) ≈ -1
        theta_dev = theta - np.pi
        
        # linearised approximations
        if abs(theta_dev) < 0.3:  # saa
            F[3, 0] = 0
            F[3, 1] = self.b_x / (self.M * self.l)
            F[3, 2] = -(self.M_total * self.g) / (self.M * self.l)
            F[3, 3] = -self.b_theta / (self.M * self.l**2)
        else:
            # numerical approximation
            F[3, :] = self.numerical_jacobian(state, u)[3, :]
        
        return F
    
    def numerical_jacobian(self, state, u=0, epsilon=1e-6):
        # numerical approximation of Jacobian using finite differences
        n = len(state)
        J = np.zeros((n, n))
        
        for j in range(n):
            # Perturb j-th component
            state_plus = state.copy()
            state_minus = state.copy()
            state_plus[j] += epsilon
            state_minus[j] -= epsilon
            
            # Compute state derivatives
            f_plus = self.nonlinear_dynamics(state_plus, u)
            f_minus = self.nonlinear_dynamics(state_minus, u)
            
            # Finite difference
            J[:, j] = (f_plus - f_minus) / (2 * epsilon)
        
        return J
    
    def nonlinear_dynamics(self, state, u=0):
        # nonlinear dynamics for numerical Jacobian computation
        x, x_dot, theta, theta_dot = state
        
        # compute accelerations using the same equations as state_cal
        sin_theta = np.sin(theta)
        cos_theta = np.cos(theta)
        
        A = np.array([[self.M_total, self.m * self.l * cos_theta],
                      [self.m * self.l * cos_theta, self.m * self.l**2]])
        
        b = np.array([u - self.b_x * x_dot + self.m * self.l * theta_dot**2 * sin_theta,
                      self.m * self.g * self.l * sin_theta - self.b_theta * theta_dot])
        
        x_ddot, theta_ddot = np.linalg.solve(A, b)
        
        return np.array([x_dot, x_ddot, theta_dot, theta_ddot])
    






class Sensor:
    def __init__(self, angle_std=0.02, pos_std=0.005):
        self.angle_std = angle_std
        self.pos_std = pos_std

    def add_noise(self, true_state):
        # add guassian noise to the true state
        noisy_state = true_state.copy()
        if len(true_state) == 2:
            # input is already the x, theta measurements 
            noisy_state[0] += np.random.normal(0, self.pos_std)   
            noisy_state[1] += np.random.normal(0, self.angle_std) 
        elif len(true_state) == 4:
            # input is full state [x, x_dot, θ, θ_dot]
            noisy_state[0] += np.random.normal(0, self.pos_std)   
            noisy_state[2] += np.random.normal(0, self.angle_std) 
        else:
            raise ValueError(f"true_state has unexpected length: {len(true_state)}")
        
        return noisy_state
        
    def lowpass_filter(self, measurement, prev_filtered, alpha=0.2):
        #  simple low pass filter
        return alpha*measurement + (1-alpha) * prev_filtered
    



class KalmanFilter:
    def __init__(self, dt=0.01):
        self.dt = dt
        self.model = LinearisedPendulumModel(dt)
        self.x = np.array([0, 0, np.pi, 0])  # state
        self.P = np.diag([0.2, 0.15, 0.01, 0.1])  #  initial uncertainty
        self.Q = np.diag([1e-6, 1e-6, 1e-6, 1e-5])
        position_std = 0.005 
        angle_std = 0.02      
        self.R = np.diag([position_std**2, angle_std**2])
        self.H = np.array([[1,0,0,0], [0,0,1,0]])
        self.state_history = []
        self.cov_history = []

    def predict(self, u=0):
        F = self.model.compute_jacobian_F(self.x, u)
        F_discrete = np.eye(4) + F * self.dt
        
        # Predict state
        f = self.model.nonlinear_dynamics(self.x, u)
        self.x = self.x + f * self.dt
        
        self.x[2] = np.arctan2(np.sin(self.x[2]), np.cos(self.x[2]))
        
        # Predict covariance
        self.P = F_discrete @ self.P @ F_discrete.T + self.Q
        
        self.state_history.append(self.x.copy())
        self.cov_history.append(np.diag(self.P).copy())

    def update(self, z):
        # Innovation with angle wrapping
        y = z - self.H @ self.x
        
        y[1] = np.arctan2(np.sin(y[1]), np.cos(y[1]))
        
        # Innovation covariance
        S = self.H @ self.P @ self.H.T + self.R
        
        # Kalman gain
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        # Update state
        self.x = self.x + K @ y
        
        # Wrap theta again after update
        self.x[2] = np.arctan2(np.sin(self.x[2]), np.cos(self.x[2]))
        
        I = np.eye(4)
        self.P = (I - K @ self.H) @ self.P @ (I - K @ self.H).T + K @ self.R @ K.T

        return self.x.copy()

    
    def get_estimate(self):
        return self.x.copy(), np.sqrt(np.diag(self.P))