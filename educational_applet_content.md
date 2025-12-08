# Project Proposal: Comparative Control Analysis of a 6-DOF Planar Quadrotor System

## Motivation

The planar quadrotor represents a fundamental **underactuated system** where horizontal motion is achieved indirectly through attitude changes. This coupling between lateral position and pitch angle presents significant control challenges, making it an ideal platform to compare linear and nonlinear control techniques. Understanding the **limitationgs of linearization-based methods** versus **exact feedback linearization** provides crucial insights for real-world autonomous flight applications.

## System Description

We consider a 6-DOF planar quadrotor with **state vector** $x = [y, z, \theta, \dot{y}, \dot{z}, \dot{\theta}]^T$ and **control inputs** $u = [F, M]^T$ (thrust and torque).

The system dynamics are:
$$ m\ddot{y} = -F \sin \theta $$
$$ m\ddot{z} = F \cos \theta - mg $$
$$ J\ddot{\theta} = M $$

In **control-affine form**: $\dot{x} = f(x) + g(x)u$, where:

$$ f(x) = \begin{bmatrix} \dot{y} \ \dot{z} \ \dot{\theta} \ 0 \ -g \ 0 \end{bmatrix} $$

$$ g(x) = \begin{bmatrix} 0 & 0 \ 0 & 0 \ 0 & 0 \ -\frac{\sin \theta}{m} & 0 \ \frac{\cos \theta}{m} & 0 \ 0 & \frac{1}{J} \ \end{bmatrix} $$

## Project Objectives

### 1. Stability Analysis (Detailed)

The stability of the unforced planar quadrotor system (when control inputs are constant at hover: $F = mg$, $M = 0$) around a hover equilibrium $x_e = [d, h, 0, 0, 0, 0]^T$ (where $d$ and $h$ are constant position values) can be analyzed using Lyapunov theory. For an **underactuated** system like the quadrotor, where there are fewer control inputs than degrees of freedom, achieving global asymptotic stability to a point can be challenging or impossible if the equilibrium is non-hyperbolic.

In this case, the hover equilibrium is **unstable**. Intuitively, this is because the horizontal position ($y$) is only controllable through changes in the pitch angle ($\theta$). If the quadrotor is at a desired horizontal position but has any horizontal velocity, a perfectly vertical thrust ($F=mg, \theta=0$) will not correct this velocity. To move horizontally, the quadrotor must tilt ($\theta \neq 0$), which then means the vertical thrust component ($F \cos\theta$) is less than $mg$, causing it to fall, or the vertical thrust component is greater than $mg$, causing it to accelerate upwards. This coupling makes stabilizing both horizontal position and attitude simultaneously at $y=0, \theta=0$ difficult.

**Sketch of Proof for Instability using Lyapunov Theory (or related concepts):**

Consider the system dynamics:
$m\ddot{y} = -F \sin \theta$
$m\ddot{z} = F \cos \theta - mg$
$J\ddot{\theta} = M$

At the hover equilibrium, $F = mg$ and $M = 0$. The state is $x_e = [d, h, 0, 0, 0, 0]^T$.
Let's analyze the dynamics around this equilibrium. The equations for $y$ and $\theta$ are particularly insightful for instability.

If we consider small perturbations around the equilibrium, a common approach for analyzing instability is to examine the linearized system eigenvalues. However, for a formal proof of instability of the *nonlinear* system, a Lyapunov-based approach like **Chetaev's Instability Theorem** is often used.

A simple demonstration of instability can be made by noting the implications of underactuation:
1.  **Horizontal dynamics:** The $\ddot{y}$ equation depends on $\sin\theta$. If $\theta=0$ (desired hover attitude), then $\ddot{y}=0$, meaning any non-zero $\dot{y}$ cannot be directly counteracted if $\theta$ is fixed at zero. This suggests that without tilting, horizontal position is not directly controllable to zero velocity if it starts with one.
2.  **Coupling:** To control $y$, $\theta$ must be non-zero. But a non-zero $\theta$ affects $z$ dynamics (unless $F$ is dynamically adjusted).

For a formal proof of instability using Chetaev's Theorem, one would typically:
*   Identify an equilibrium point (e.g., $x_e = [0, 0, 0, 0, 0, 0]^T$ for simplicity after coordinate transformation).
*   Find a continuously differentiable function $V(x)$ (a "Chetaev function") such that:
    *   $V(x_e) = 0$.
    *   In an arbitrarily small neighborhood $U$ of $x_e$, there exists a region $V_0 = \{x \in U \mid V(x) > 0 \}$ such that $\dot{V}(x) > 0$ for all $x \in V_0$.
*   This would imply that starting in $V_0$, the system state will move away from the equilibrium.

For the planar quadrotor, such a function $V(x)$ might relate to the horizontal kinetic energy or potential energy related to pitch, demonstrating that any slight deviation in horizontal velocity or pitch angle, without immediate correcting action, leads to an increase in this "energy" function, driving the system away from equilibrium. The constant $u=[mg, 0]^T$ in the unforced system context means no active control is applied to stabilize deviations, thus leading to instability. The underactuation (3 DOFs but only 2 independent controls, $F$ and $M$) is key. The number of independent controls is less than the number of generalized coordinates (y, z, theta), making it impossible to arbitrarily choose accelerations for all coordinates directly.

### 2. LQR Control

*Linearize the system around hover equilibrium and design an LQR controller. Demonstrate stabilization performance and identify the limitations arising from linearization approximations (e.g., restricted region of attraction, performance degradation for large angles).*

**Derivation Hint:**
1.  **Linearization:** Linearize the nonlinear system $\dot{x} = f(x) + g(x)u$ around an equilibrium point $(x_e, u_e)$.
    The linearized system is $\dot{\delta}x = A\delta x + B\delta u$, where $\delta x = x - x_e$ and $\delta u = u - u_e$.
    $A = \frac{\partial f}{\partial x}|_{x_e, u_e} + \frac{\partial g}{\partial x}|_{x_e, u_e} u_e$
    $B = g(x_e)$
2.  **LQR Controller Design:** For the linearized system, find the control law $\delta u = -K\delta x$ that minimizes the quadratic cost function:
    $J = \int_0^\infty (\delta x^T Q \delta x + \delta u^T R \delta u) dt$
    The gain matrix $K$ is given by $K = R^{-1} B^T P$, where $P$ is the unique positive definite solution to the Algebraic Riccati Equation (ARE):
    $A^T P + P A - P B R^{-1} B^T P + Q = 0$

### 3. Feedback Linearization

*Apply exact feedback linearization to transform the nonlinear system into a linear form without approximations. Design controllers for the linearized subsystems and demonstrate trajectory tracking capabilities.*

**Derivation Hint:**
Feedback linearization involves finding a diffeomorphism $z = T(x)$ and a control law $u = \alpha(x) + \beta(x)v$ such that the transformed system $\dot{z} = Az + Bv$ is linear. For a single-input single-output (SISO) system, this typically involves finding the relative degree $r$ and using Lie derivatives. For MIMO systems, it's more complex, often involving input-output linearization or full state linearization.

For the planar quadrotor, we can decouple the altitude ($z$) and lateral position ($y$) control.

**Altitude ($z$) Subsystem:**
Let $z_1 = z$ and $z_2 = \dot{z}$.
$\\dot{z}_1 = z_2$
$\\dot{z}_2 = \ddot{z} = \frac{F \cos \theta}{m} - g$
We can define a virtual control input $v_z = \frac{F \cos \theta}{m} - g$.
Then $\\ddot{z} = v_z$. We can design a linear controller for $v_z$ (e.g., pole placement) to control $z$.
The control input $F$ can then be found from $F = \frac{m(v_z + g)}{\cos \theta}$.

**Lateral Position ($y$) Subsystem:**
Let $y_1 = y$, $y_2 = \dot{y}$, $y_3 = \theta$, $y_4 = \dot{\theta}$. This transformation is more intricate, involving the dynamics of $y$ and $\theta$.
The approach often taken is to define the output as $h_y(x) = y$.
$L_f h_y = \dot{y}$
$L_f^2 h_y = \ddot{y} = -\frac{F \sin \theta}{m}$
This is where the direct control input $F$ appears. To linearize the $y$ dynamics, we typically need to control the attitude $\theta$. This leads to a cascaded control structure where $F$ controls $z$ and $M$ controls $\theta$ (which in turn affects $y$). 

The implementation details involve careful selection of states for linearization and designing pole-placement controllers for each linearized subsystem, as seen in the provided MATLAB and Julia code.

### 4. Comparative Analysis

*Compare LQR and feedback linearization approaches in terms of region of attraction, tracking performance, and robustness.*

This section will visually demonstrate the differences in performance between the two controllers, especially when operating far from equilibrium or attempting to track dynamic trajectories.

## Expected Outcomes

This project will provide a comprehensive comparison between linearization-based (LQR) and exact nonlinear (feedback linearization) control techniques. It will demonstrate that while LQR is effective near equilibrium, feedback linearization offers superior performance for large maneuvers and trajectory tracking. All results will be validated through MATLAB simulations with clear visualizations, making the project easily reproducible for evaluation purposes.