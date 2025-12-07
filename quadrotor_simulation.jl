### A Pluto.jl notebook ###
# v0.20.21

# ╔═╡ 00000000-0000-0000-0000-000000000001
begin
	using PlutoUI
	using LinearAlgebra
	using ControlSystems
	using DifferentialEquations
	using Plots
	md"
	# Comparative Control Analysis of a 6-DOF Planar Quadrotor System

	This notebook implements and compares two control strategies for a 6-DOF planar quadrotor:
	1.  **LQR (Linear-Quadratic Regulator) Control:** A linear control technique based on a linearized model of the system.
	2.  **Feedback Linearization:** A nonlinear control technique that exactly linearizes the system dynamics.

	The goal is to analyze the performance of these controllers in terms of stability, trajectory tracking, and robustness to large initial errors.

	"
end

# ╔═╡ 00000000-0000-0000-0000-000000000002
md"
## System Parameters
We define the physical parameters of the quadrotor.
"

# ╔═╡ 00000000-0000-0000-0000-000000000003
begin
	const g = 9.81  # Acceleration due to gravity (m/s^2)
	const m = 0.5   # Mass of the quadrotor (kg)
	const J = 0.1   # Moment of inertia of the quadrotor (kg*m^2)
end

# ╔═╡ 00000000-0000-0000-0000-000000000004
md"
## System Dynamics
The state of the system is given by `x = [x, z, θ, x_dot, z_dot, θ_dot]`. The control inputs are `u = [F, M]`, where `F` is the total thrust and `M` is the torque.

The nonlinear dynamics of the quadrotor are given by the following equations:
```
m * x_ddot = -F * sin(θ)
m * z_ddot =  F * cos(θ) - m * g
J * θ_ddot =  M
```
We can write this in control-affine form `x_dot = f(x) + g(x)u`.
"

# ╔═╡ 00000000-0000-0000-0000-000000000005
function planar_quadrotor_dynamics!(dx, x, p, t, controller, r)
    # Get control input from the specified controller
    u = controller(x, r)

    # Dynamics
    dx[1] = x[4]
    dx[2] = x[5]
    dx[3] = x[6]
    dx[4] = -u[1] * sin(x[3]) / m
    dx[5] =  u[1] * cos(x[3]) / m - g
    dx[6] =  u[2] / J
end

# ╔═╡ 00000000-0000-0000-0000-000000000006
md"
## LQR Controller
The LQR controller is designed based on a linearization of the system around the hover equilibrium point: `x_eq = [0, 0, 0, 0, 0, 0]` and `u_eq = [m*g, 0]`.
"

# ╔═╡ 00000000-0000-0000-0000-000000000007
begin
	# Equilibrium point
	const x_eq = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
	const u_eq = [m * g, 0.0]

	# Linearized system dynamics (A and B matrices)
	const A = [0.0 0.0 0.0 1.0 0.0 0.0;
			   0.0 0.0 0.0 0.0 1.0 0.0;
			   0.0 0.0 0.0 0.0 0.0 1.0;
			   0.0 0.0 -g  0.0 0.0 0.0;
			   0.0 0.0 0.0 0.0 0.0 0.0;
			   0.0 0.0 0.0 0.0 0.0 0.0]

	const B = [0.0 0.0;
			   0.0 0.0;
			   0.0 0.0;
			   0.0 0.0;
			   1/m 0.0;
			   0.0 1/J]

	# LQR Controller design
	const Q = diagm([10.0, 10.0, 1.0, 5.0, 5.0, 1.0])
	const R = diagm([1.0, 1.0])

	const K_lqr = lqr(A, B, Q, R)

	# LQR control law
	lqr_controller(x, r) = u_eq - K_lqr * (x - r)
end


# ╔═╡ 00000000-0000-0000-0000-000000000008
md"
## Feedback Linearization Controller
Feedback linearization is a nonlinear control technique that algebraically transforms the nonlinear system dynamics into a linear one. We can then apply linear control techniques to the linearized system.

For the quadrotor, we define the output as `y = [x, z]`. By differentiating the output twice with respect to time, we can find the relationship between the output and the control inputs.

The control law is:
`u = g(x)^-1 * (v - f(x))`
where `v` is the new linear control input.
"

# ╔═╡ 00000000-0000-0000-0000-000000000009
function feedback_linearization_controller(x, r)
    # Desired trajectory (reference and its derivatives)
    x_d, z_d = r[1], r[2]
    x_dot_d, z_dot_d = r[4], r[5]
    x_ddot_d, z_ddot_d = 0.0, 0.0 # Assume zero acceleration for simplicity

    # Controller for the linearized system (PD controller)
    kp_x, kd_x = 2.0, 3.0
    kp_z, kd_z = 2.0, 3.0
    
    vx = x_ddot_d - kp_x * (x[1] - x_d) - kd_x * (x[4] - x_dot_d)
    vz = z_ddot_d - kp_z * (x[2] - z_d) - kd_z * (x[5] - z_dot_d)

    # Feedback linearization control law
    u1 = m * sqrt(vx^2 + (vz + g)^2)
    θ_d = atan(-vx, vz + g)
    
    # PD controller for attitude
    kp_θ, kd_θ = 5.0, 1.0
    u2 = J * ( -kp_θ * (x[3] - θ_d) - kd_θ * x[6] )
    
    return [u1, u2]
end

# ╔═╡ 00000000-0000-0000-0000-000000000010
md"
## Simulation
We simulate the quadrotor's response with both controllers for a given set of initial conditions.
"

# ╔═╡ 00000000-0000-0000-0000-000000000011
function simulate(controller, x0, t_span, r)
    prob = ODEProblem((dx, x, p, t) -> planar_quadrotor_dynamics!(dx, x, p, t, controller, r), x0, t_span)
    sol = solve(prob, Tsit5())
    return sol
end

# ╔═╡ 00000000-0000-0000-0000-000000000012
md"
## Interactive Comparison
Use the sliders to set the initial position and attitude of the quadrotor. The plots show the trajectory of the quadrotor with both LQR and Feedback Linearization controllers.
"

# ╔═╡ 00000000-0000-0000-0000-000000000013
begin
	@bind x0_slider PlutoUI.Slider(-5.0:0.1:5.0, default=1.0)
	@bind z0_slider PlutoUI.Slider(-5.0:0.1:5.0, default=1.0)
	@bind theta0_slider PlutoUI.Slider(-pi:0.1:pi, default=0.5)
end

# ╔═╡ 00000000-0000-0000-0000-000000000014
begin
	x0_vec = [x0_slider, z0_slider, theta0_slider, 0.0, 0.0, 0.0]
	t_span = (0.0, 10.0)
	r = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # Reference is the origin (hover)

	sol_lqr = simulate(lqr_controller, x0_vec, t_span, r)
	sol_fl = simulate(feedback_linearization_controller, x0_vec, t_span, r)

	p = plot(sol_lqr, vars=(1, 2), xlabel="x (m)", ylabel="z (m)", label="LQR", legend=:bottomright, title="Quadrotor Trajectories")
	plot!(p, sol_fl, vars=(1, 2), label="Feedback Linearization")
	scatter!(p, [x0_vec[1]], [x0_vec[2]], label="Initial Position", markersize=5)
	
	p
end


# ╔═╡ Cell order:
# ╠═00000000-0000-0000-0000-000000000001
# ╠═00000000-0000-0000-0000-000000000002
# ╠═00000000-0000-0000-0000-000000000003
# ╠═00000000-0000-0000-0000-000000000004
# ╠═00000000-0000-0000-0000-000000000005
# ╠═00000000-0000-0000-0000-000000000006
# ╠═00000000-0000-0000-0000-000000000007
# ╠═00000000-0000-0000-0000-000000000008
# ╠═00000000-0000-0000-0000-000000000009
# ╠═00000000-0000-0000-0000-000000000010
# ╠═00000000-0000-0000-0000-000000000011
# ╠═00000000-0000-0000-0000-000000000012
# ╠═00000000-0000-0000-0000-000000000013
# ╠═00000000-0000-0000-0000-000000000014
