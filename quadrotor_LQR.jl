### A Pluto.jl notebook for LQR control of a planar quadrotor

using PlutoUI
using LinearAlgebra
using ControlSystems
using DifferentialEquations
using Plots

# System Parameters
const g = 9.81
const m = 0.5
const J = 0.1

# Equilibrium point
const x_eq = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
const u_eq = [m * g, 0.0]

# Linearized system dynamics
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

# LQR Controller
const Q = diagm([10.0, 10.0, 1.0, 5.0, 5.0, 1.0])
const R = diagm([1.0, 1.0])

const K = lqr(A, B, Q, R)

# Closed-loop system
const Ac = A - B * K

# Simulation
function quadrotor_dynamics!(dx, x, p, t)
    u = -K * (x - x_eq) + u_eq
    dx[1] = x[4]
    dx[2] = x[5]
    dx[3] = x[6]
    dx[4] = -u[1] * sin(x[3]) / m
    dx[5] = u[1] * cos(x[3]) / m - g
    dx[6] = u[2] / J
end

function simulate_quadrotor(x0, t_span)
    prob = ODEProblem(quadrotor_dynamics!, x0, t_span)
    sol = solve(prob, Tsit5())
    return sol
end

# Interactive Plot
@bind x0_slider PlutoUI.Slider(-5.0:0.1:5.0, default=1.0)
@bind z0_slider PlutoUI.Slider(-5.0:0.1:5.0, default=1.0)
@bind theta0_slider PlutoUI.Slider(-pi:0.1:pi, default=0.5)


function plot_trajectory(x0, z0, theta0)
    x0_vec = [x0, z0, theta0, 0.0, 0.0, 0.0]
    t_span = (0.0, 10.0)
    sol = simulate_quadrotor(x0_vec, t_span)

    plot(sol, vars=(1, 2), xlabel="x (m)", ylabel="z (m)", label="Quadrotor Trajectory", legend=:bottomright)
    scatter!([x0], [z0], label="Initial Position", markersize=5)
    quiver!([x0], [z0], quiver=([cos(theta0)], [sin(theta0)]), label="Initial Attitude", color=:red)
end

plot_trajectory(x0_slider, z0_slider, theta0_slider)
