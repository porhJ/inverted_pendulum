import numpy as np
from scipy.linalg import solve_continuous_are


class InvertedPendulum:
    def __init__(self, m_c, m_p, l, y_bar, u_bar, g=9.8):
        self.m_c = m_c
        self.m_p = m_p
        self.l = l
        self.g = g
        self.y_bar = y_bar
        self.u_bar = u_bar

    def dynamics(self, q, u):
        x, x_dot, theta, theta_dot = q
        M = np.array(
            [
                [self.m_c + self.m_p, self.m_p * self.l * np.cos(theta)],
                [self.m_p * self.l * np.cos(theta), self.m_p * (self.l**2)],
            ]
        )
        C = np.array([[0, -self.m_p * self.l * theta_dot * np.sin(theta)], [0, 0]])
        tau_g = np.array([0, -self.m_p * self.g * self.l * np.sin(theta)])
        B = np.array([1, 0])
        q_dot = np.array([x_dot, theta_dot])
        forces = tau_g + (B * u) - (C @ q_dot)

        accel = np.linalg.solve(M, forces)
        x_ddot, theta_ddot = accel

        return np.array([x_dot, x_ddot, theta_dot, theta_ddot])

    def enforce_rail_limits(self, X, x_min, x_max, restitution=0.2):
        x, x_dot, theta, theta_dot = X

        if x <= x_min:
            x = x_min
            if x_dot < 0:
                x_dot = -restitution * x_dot

        elif x >= x_max:
            x = x_max
            if x_dot > 0:
                x_dot = -restitution * x_dot

        return np.array([x, x_dot, theta, theta_dot])

    def get_jacobian(self, y_bar, u_bar, eps=1e-6):
        n = len(y_bar)
        m = 1
        A = np.zeros((n, n))
        B = np.zeros((n, m))

        f0 = self.dynamics(y_bar, u_bar)

        for i in range(n):
            y_perturb = y_bar.copy()
            y_perturb[i] += eps
            f_perturb = self.dynamics(y_perturb, u_bar)
            A[:, i] = (f_perturb - f0) / eps

        u_perturb = u_bar + eps
        f_perturb_u = self.dynamics(y_bar, u_perturb)
        B[:, 0] = (f_perturb_u - f0) / eps

        return A, B

    def linearized_dynamics(self, X, u, A, B):
        # X here is state vector not position x naja
        X_dot = A @ X + B @ u
        return X_dot

    def control_law(self, X, K):
        return -K @ (X - self.y_bar)

    def get_K(self, A, B, Q, R):
        P = solve_continuous_are(A, B, Q, R)
        return np.linalg.inv(R) @ B.T @ P

    def step(self, X, K, A, B, dt=1.0e-3):
        u = self.control_law(X, K)
        X_dot = self.dynamics(X, u)
        return X + X_dot * dt
