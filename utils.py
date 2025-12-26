import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle


def is_converged(x, target_x, tol_pos=0.01, tol_vel=0.01):
    pos_err = abs(x[0] - target_x[0])
    vel_err = abs(x[1] - target_x[1])

    if pos_err < tol_pos and vel_err < tol_vel:
        return True
    else:
        return False


class sim_animation:
    def __init__(self, lo, hist, cart_width=0.4, cart_height=0.2):
        self.hist = hist
        self.cart_width = cart_width
        self.cart_height = cart_height
        self.l = lo
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(-2, 2)
        self.ax.set_ylim(-1, 1)
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
        self.ax.set_xlim(x - 2.0, x + 2.0)

        self.cart.set_xy((x - self.cart_width / 2, -self.cart_height / 2))
        px = x + self.l * np.sin(theta)
        py = self.l * np.cos(theta)
        self.pole.set_data([x, px], [0, py])
        return self.cart, self.pole
