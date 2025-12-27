import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle


def is_converged(x, target_x, tol_pos=0.01, tol_vel=0.01):
    pos_err = abs(x[0] - target_x[0])
    vel_err = abs(x[1] - target_x[1])

    if pos_err < tol_pos and vel_err < tol_vel:
        return True
    else:
        return False


class CartPoleAnimator:
    def __init__(
        self,
        hist,
        pole_length,
        cart_width=0.4,
        cart_height=0.2,
        xlim=(-2, 2),
        ylim=(-1, 1),
        frame_step=10,
        interval=20,
    ):
        self.hist = hist
        self.l = pole_length
        self.cart_width = cart_width
        self.cart_height = cart_height
        self.frame_step = frame_step
        self.interval = interval

        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(*xlim)
        self.ax.set_ylim(*ylim)
        self.ax.set_aspect("equal")
        self.ax.grid()

        self.cart = Rectangle((0, 0), cart_width, cart_height, fc="black")
        (self.pole,) = self.ax.plot([], [], lw=2)
        self.ax.add_patch(self.cart)

    def init(self):
        self.cart.set_xy((-self.cart_width / 2, -self.cart_height / 2))
        self.pole.set_data([], [])
        return self.cart, self.pole

    def update(self, i):
        x, _, theta, _ = self.hist[i]
        self.cart.set_xy((x - self.cart_width / 2, -self.cart_height / 2))
        px = x + self.l * np.sin(theta)
        py = self.l * np.cos(theta)
        self.pole.set_data([x, px], [0, py])
        return self.cart, self.pole

    def animate(self):
        self.ani = FuncAnimation(
            self.fig,
            self.update,
            frames=range(0, len(self.hist), self.frame_step),
            init_func=self.init,
            interval=self.interval,
            blit=True,
        )
        return self.ani

    def show(self):
        plt.show()


def plot_inverted_pendulum_LQR(
    theta_hist, x_hist, theta_dot_hist, x_dot_hist, t_con, Q, R
):
    plt.plot(theta_hist, label="theta")
    plt.plot(x_hist, label="x")
    plt.plot(theta_dot_hist, label="theta_dot")
    plt.plot(x_dot_hist, label="x_dot")
    plt.title("Inverted Pendulum LQR-Control")
    plt.axhline(y=0, color="r", linestyle="--", label="Target equilibrium (theta=0)")
    plt.axvline(
        x=t_con, color="g", linestyle="--", label=f"Convergent time (x={t_con})"
    )
    fig = plt.gcf()
    fig.text(
        0.5,
        0.005,
        f"Q = diag({np.diag(Q)})    R = diag({np.diag(R)})",
        ha="center",
        va="bottom",
        fontsize=9,
    )
    plt.xlabel("Time Steps")
    plt.legend()
    Q_vals = "_".join(f"{v:g}" for v in np.diag(Q))
    R_vals = "_".join(f"{v:g}" for v in np.diag(R))

    # plt.savefig(f"images/baseline/Q{Q_vals}_R{R_vals}NoXXdot.png", dpi=300)
    plt.show()
