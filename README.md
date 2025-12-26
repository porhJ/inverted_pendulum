
# Project Overview

This project studies stabilization of an inverted pendulum on a cart using LQR control. The controller is designed via linearization but applied to the full nonlinear dynamics. We investigate the effect of rail constraints on stability and Q/R tuning.

# System modelling

## Coordinates and assumptions
* State $$q = [x, \theta], \\ \underline{x} = [q, \dot{q}] = [x, \dot{x}, \theta, \dot{\theta}]$$
* Angle convention $$Upright: \theta = 0$$
* The pendulum is a small bob, or a point mass, ($r \ll l$, where $l$ is length of the rigid string) thus the moment of inertial $$I = m_pl^2$$
* No friction and any disturbance

## System Dynamics
* $m_c$ : mass of the cart, $m_p$: mass of the point mass, $l$: length of the rigid string
* $x$: vector from origin to CM of the cart, $\theta$: angle of the rigid string from the upright 
*  Using  Lagrangian mechanics to derive non-linear dynamics 
$$T = \frac1 2 m_c \dot{x}^2 + \frac 1 2 m_p (\dot{x} + l\dot{\theta}\sin{\theta})^2 + \frac 1 2m_p(l\dot{\theta}\sin{\theta})^2$$
$$U = -m_pgl\cos{\theta}$$ 
$$L = T - U$$
$$\frac{d}{dt} \frac {\partial}{\partial \dot{\underline{x}}}L  - \frac {\partial}{\partial \underline{x}}L = \tau$$
* From the Manipulator equation, $M(\underline{q})\underline{\ddot{q}} +  C(\underline{q}, \underline{\dot{q}}) \underline{\dot{q}}= \tau_g(\underline{q}) + Bu$, where:

$$M(\underline{q}) = \begin{bmatrix}  
m_c+m_p & m_pl\cos{\theta} \\  
m_pl\cos{\theta} & m_pl^2  
\end{bmatrix}, 
C(\underline{q}, \dot{\underline{q}}) = \begin{bmatrix}  
0 & -m_pl\dot{\theta}\sin{\theta} \\  
0 & 0  
\end{bmatrix},$$
$$
\tau_g(\underline{q}) = \begin{bmatrix}  
0 \\  
-m_pl\sin{\theta} 
\end{bmatrix},
B = \begin{bmatrix}  
1 \\  
0
\end{bmatrix},$$

* To linearize the equation, we need this form $\underline{\dot{x}} = \underline{f}(\underline{x})$
* Note that $\underline{\dot{x}} = [\dot{x}, \ddot{x}, \dot\theta, \ddot\theta]$ and we can obtain $\dot{x}$ and $\dot{\theta}$ from $\underline{x}$, thus, in order to know $\underline{\dot{x}}$, we only need to know $\underline{\ddot{q}}$, where $\underline{\ddot{q}} = [\ddot{x}, \ddot{\theta}]$. And we can obtain it from:
$$\underline{\ddot{q}}  = M^{-1}(\tau_g + Bu - C \underline{\dot{q}})$$ 
* $u$ is the input, and, in this case, force on the cart.
