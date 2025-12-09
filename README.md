# Comparative Control Analysis of a 6-DOF Planar Quadrotor

This project demonstrates and compares two control strategies for a 6-DOF planar quadrotor. The Julia implementation of **Feedback Linearization** in `quadrotor_simulation.jl` has been updated to align more closely with the theoretical principles demonstrated in the companion MATLAB code, providing a more rigorous application of the technique.

The simulation is implemented in a Pluto.jl notebook, providing an interactive environment to explore and compare the performance of these controllers.

## Live Demo (GitHub Pages)

You can access a static HTML version of the `quadrotor_simulation.jl` notebook, including all visualizations and results, directly online via GitHub Pages:

[(https://prachitgupta.github.io/quadrotor_applet_github/quadrotor_simulation.html)]

## Project Goal

The primary goal of this project is to provide a clear and interactive demonstration of LQR and Feedback Linearization controllers for a planar quadrotor, as outlined in the original project proposal. This serves as a foundation for understanding the trade-offs between linear and nonlinear control techniques.

## Getting Started

To run the interactive simulation, you will need to have Julia and Pluto.jl installed.

### Prerequisites

- [Julia](https://julialang.org/downloads/) (version 1.6 or later)

### Installation

1.  **Clone the repository:**
    ```bash
    git clone https://github.com/prachitgupta/quadrotor_applet.git
    cd quadrotor_applet
    ```

2.  **Install the required Julia packages:**
    Open a Julia REPL in the project directory and run the following commands:
    ```julia
    using Pkg
    Pkg.activate(".")
    Pkg.instantiate()
    ```

### Running the Interactive Notebook

1.  **Start Pluto.jl:**
    In the Julia REPL, run the following command:
    ```julia
    using Pluto
    Pluto.run()
    ```
    This will open a new tab in your web browser.

2.  **Open the notebook:**
    In the Pluto.jl interface, open the `quadrotor_simulation.jl` file from the project directory.

3.  **Interact with the simulation:**
    Use the sliders to adjust the initial position (`x0`, `z0`) and attitude (`theta0`) of the quadrotor. The plot will update in real-time to show the resulting trajectories for both LQR and Feedback Linearization controllers. Note : Feedback linearisation controller converges to singularity with large values of z0 

## Controllers

### LQR Controller

The LQR controller is designed based on a linearization of the system around the hover equilibrium point. It provides optimal control for the linearized system, but its performance may degrade for large deviations from the equilibrium.

### Feedback Linearization Controller

Feedback linearization is a nonlinear control technique that algebraically transforms the nonlinear system dynamics into a fully linear one. This allows for the application of linear control techniques to the exact, nonlinear system, providing better performance over a wider range of operating conditions. The implementation in `quadrotor_simulation.jl` now reflects a more detailed application of this technique, consistent with the reference MATLAB implementation.

## Project Structure

- `quadrotor_simulation.jl`: The main Pluto.jl notebook containing the LQR and updated Feedback Linearization controller designs, simulation, and interactive comparison.
- `Project.toml`: Specifies the Julia package dependencies for this project.
- `Manifest.toml`: Contains the exact versions of the dependencies, ensuring reproducibility.
- `matlab_implementation/`: Contains the original MATLAB implementation which was used as a reference for the feedback linearization update.
- `projecct_proposal.pdf`: The original project proposal outlining the goals and motivation for the control system analysis.
