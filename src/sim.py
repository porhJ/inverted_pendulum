import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle

from main import InvertedPendulum
from utils import is_converged, sim_animation

# initialization
y_fixed = np.array([0.0, 0.0, 0.0, 0.0])
u_fixed = 0.0
epsilon = 1e-6
m_c = 2
m_p = 5
lo = 2  # length of the rigid string
Q = np.diag([1, 1, 1, 1])  # baseline
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

plt.plot(theta_hist, label="theta")
# plt.plot(x_hist, label="x")
plt.plot(theta_dot_hist, label="theta_dot")
# plt.plot(x_dot_hist, label="x_dot")
plt.title("Inverted Pendulum LQR-Control")
plt.axhline(y=0, color="r", linestyle="--", label="Target equilibrium (theta=0)")
plt.axvline(x=t_con, color="g", linestyle="--", label=f"Convergent time (x={t_con})")
fig = plt.gcf()
fig.text(
    0.5,
    0.005,  # ← smaller = lower (0 is absolute bottom)
    f"Q = diag({np.diag(Q)})    R = diag({np.diag(R)})",
    ha="center",
    va="bottom",
    fontsize=9,
)
plt.xlabel("Time Steps")
plt.legend()
Q_vals = "_".join(f"{v:g}" for v in np.diag(Q))
R_vals = "_".join(f"{v:g}" for v in np.diag(R))

plt.savefig(f"images/baseline/Q{Q_vals}_R{R_vals}NoXXdot.png", dpi=300)
plt.show()


cart_width = 0.4
cart_height = 0.2
lo = sys.l

fig, ax = plt.subplots()
ax.set_xlim(-2, 2)
ax.set_ylim(-1, 1)
ax.set_aspect("equal")
ax.grid()

cart = Rectangle((0, 0), cart_width, cart_height, fc="black")
(pole,) = ax.plot([], [], lw=2)
ax.add_patch(cart)


def init():
    cart.set_xy((-cart_width / 2, -cart_height / 2))
    pole.set_data([], [])
    return cart, pole


def update(i):
    x, _, theta, _ = hist[i]
    cart.set_xy((x - cart_width / 2, -cart_height / 2))
    px = x + lo * np.sin(theta)
    py = lo * np.cos(theta)
    pole.set_data([x, px], [0, py])
    return cart, pole


ani = FuncAnimation(
    fig,
    update,
    frames=range(0, len(hist), 10),
    init_func=init,
    interval=20,  # ms between frames
    blit=True,
)

plt.show()
