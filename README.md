# Planar Quadrotor Control with LQR

This project demonstrates the design and simulation of a Linear-Quadratic Regulator (LQR) for stabilizing a 6-DOF planar quadrotor. The simulation is implemented in a Pluto.jl notebook, providing an interactive environment to explore the controller's performance.

## Project Goal

The primary goal of this project is to provide a clear and interactive demonstration of an LQR controller for a planar quadrotor. This serves as a foundation for understanding more advanced control techniques and for comparing different control strategies, as outlined in the original project proposal.

## Getting Started

To run the interactive simulation, you will need to have Julia and Pluto.jl installed.

### Prerequisites

- [Julia](https://julialang.org/downloads/) (version 1.6 or later)

### Installation

1.  **Clone the repository:**
    ```bash
    git clone <repository-url>
    cd <repository-name>
    ```

2.  **Install the required Julia packages:**
    Open a Julia REPL in the project directory and run the following commands:
    ```julia
    using Pkg
    Pkg.activate(".")
    Pkg.instantiate()
    ```

### Running the Simulation

1.  **Start Pluto.jl:**
    In the Julia REPL, run the following command:
    ```julia
    using Pluto
    Pluto.run()
    ```
    This will open a new tab in your web browser.

2.  **Open the notebook:**
    In the Pluto.jl interface, open the `quadrotor_LQR.jl` file from the project directory.

3.  **Interact with the simulation:**
    Use the sliders to adjust the initial position (`x0`, `z0`) and attitude (`theta0`) of the quadrotor. The plot will update in real-time to show the resulting trajectory.

## Project Structure

- `quadrotor_LQR.jl`: The main Pluto.jl notebook containing the LQR controller design, simulation, and interactive visualization.
- `Project.toml`: Specifies the Julia package dependencies for this project.
- `Manifest.toml`: Contains the exact versions of the dependencies, ensuring reproducibility.
- `matlab_implementation/`: Contains the original MATLAB implementation of the MRAC controller, which was used as a reference for the system dynamics and LQR design.
- `projecct_proposal.pdf`: The original project proposal outlining the goals and motivation for the control system analysis.

## Next Steps

This project can be extended in several ways:

-   Implement the feedback linearization controller as described in the project proposal.
-   Compare the performance of the LQR and feedback linearization controllers.
-   Add trajectory tracking capabilities.
-   Extend the simulation to 3D.
