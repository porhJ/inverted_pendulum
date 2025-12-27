import numpy as np

from main import InvertedPendulum
from utils.utils import CartPoleAnimator, is_converged, plot_inverted_pendulum_LQR

# initialization
y_fixed = np.array(
    [0.0, 0.0, 0.0, 0.0]
)  # equilibrium we wanna reach, there is 2 equilibruim points, we choose the upright one. because, obviously, this is inverted pendulum, not a normal pendulum.
u_fixed = 0.0  # actuator what it reaches equilibrium, why would u wanna act on the cart anymore, so set it to be zero
epsilon = 1e-6
m_c = 2  # mass of the cart, up to u.
m_p = 5  # mass of the point masss, up to u too.
lo = 2  # length of the rigid string u can change to whatever u like
Q = np.diag([1, 1, 1, 1])  # baseline, note: its [Qx, Qxdot, Qtheta, Qthetadot]
R = np.diag([1])  # baseline
hist = []
x_hist = []
x_dot_hist = []
theta_hist = []
theta_dot_hist = []
t_con = 0
sys = InvertedPendulum(m_c=m_c, m_p=m_p, l=lo, y_bar=y_fixed, u_bar=u_fixed)
A, B = sys.get_jacobian(y_fixed, u_fixed)
K = sys.get_K(A, B, Q, R)
X = np.array(
    [0.0, 0.0, np.pi + 0.1, 0.0]
)  # X initial, it must be a lil off from equilibrium
# from the dynamic eq. i used, theta = 0 -> upright, theta = pi -> down right
# our goal is theta = 0

# loop
for step in range(100000):
    X = sys.step(X, K, A, B)
    X = sys.enforce_rail_limits(X, x_min=-2.0, x_max=2.0)
    hist.append(X)
    x, x_dot, theta, theta_dot = X
    theta_wrapped = np.arctan2(np.sin(theta), np.cos(theta))
    x_hist.append(x)
    x_dot_hist.append(x_dot)
    theta_hist.append(theta_wrapped)
    theta_dot_hist.append(theta_dot)
    if is_converged([theta_wrapped, theta_dot], [0, 0]):
        t_con = step
        break

plot_inverted_pendulum_LQR(theta_hist, x_hist, theta_dot_hist, x_dot_hist, t_con, Q, R)


animator = CartPoleAnimator(hist, pole_length=sys.l)
ani = animator.animate()
animator.show()
