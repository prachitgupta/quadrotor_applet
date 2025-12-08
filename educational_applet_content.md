# Project Proposal: Comparative Control Analysis of a 6-DOF Planar Quadrotor System

## Motivation

The planar quadrotor represents a fundamental **underactuated system** where horizontal motion is achieved indirectly through attitude changes. This coupling between lateral position and pitch angle presents significant control challenges, making it an ideal platform to compare linear and nonlinear control techniques. Understanding the **limitations of linearization-based methods** versus **exact feedback linearization** provides crucial insights for real-world autonomous flight applications.

## System Description

We consider a 6-DOF planar quadrotor with **state vector** $x = [y, z, \theta, \dot{y}, \dot{z}, \dot{\theta}]^T$ and **control inputs** $u = [F, M]^T$ (thrust and torque).

The system dynamics are:
$$ m\ddot{y} = -F \sin \theta $$
$$ m\ddot{z} = F \cos \theta - mg $$
$$ J\ddot{\theta} = M $$

In **control-affine form**: $\dot{x} = f(x) + g(x)u$, where:

$$ f(x) = \begin{bmatrix} \dot{y} \\ \dot{z} \\ \dot{\theta} \\ 0 \\ -g \\ 0 \end{bmatrix} $$

$$ g(x) = \begin{bmatrix} 0 & 0 \\ 0 & 0 \\ 0 & 0 \\ -\frac{\sin \theta}{m} & 0 \\ \frac{\cos \theta}{m} & 0 \\ 0 & \frac{1}{J} \end{bmatrix} $$

## Project Objectives

### 1. Stability Analysis

*Use Lyapunov theory to prove that the unforced system with $u = [mg, 0]^T$ is unstable around any hover equilibrium $x_e = [d, h, 0, 0, 0, 0]^T$.*

**Derivation Hint:**
For Lyapunov stability analysis, one typically defines a Lyapunov candidate function $V(x)$ such that $V(x) > 0$ for $x \ne x_e$ and $V(x_e) = 0$. The derivative $\dot{V}(x)$ along the system trajectories must be negative definite or negative semi-definite for stability. For instability, one might show that $\dot{V}(x)$ is positive definite in some region, or use Chetaev's instability theorem.

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
$\dot{z}_1 = z_2$
$\dot{z}_2 = \ddot{z} = \frac{F \cos \theta}{m} - g$
We can define a virtual control input $v_z = \frac{F \cos \theta}{m} - g$.
Then $\ddot{z} = v_z$. We can design a linear controller for $v_z$ (e.g., pole placement) to control $z$.
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
