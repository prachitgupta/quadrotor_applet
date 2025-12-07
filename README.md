# Comparative Control Analysis of a 6-DOF Planar Quadrotor

This project demonstrates and compares two control strategies for a 6-DOF planar quadrotor:
1.  **LQR (Linear-Quadratic Regulator) Control:** A linear control technique based on a linearized model of the system.
2.  **Feedback Linearization:** A nonlinear control technique that exactly linearizes the system dynamics.

The simulation is implemented in a Pluto.jl notebook, providing an interactive environment to explore and compare the performance of these controllers.

## Project Goal

The primary goal of this project is to provide a clear and interactive demonstration of LQR and Feedback Linearization controllers for a planar quadrotor, as outlined in the original project proposal. This serves as a foundation for understanding the trade-offs between linear and nonlinear control techniques.

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
    In the Pluto.jl interface, open the `quadrotor_simulation.jl` file from the project directory.

3.  **Interact with the simulation:**
    Use the sliders to adjust the initial position (`x0`, `z0`) and attitude (`theta0`) of the quadrotor. The plot will update in real-time to show the resulting trajectories for both LQR and Feedback Linearization controllers.

## Controllers

### LQR Controller

The LQR controller is designed based on a linearization of the system around the hover equilibrium point. It provides optimal control for the linearized system, but its performance may degrade for large deviations from the equilibrium.

### Feedback Linearization Controller

Feedback linearization is a nonlinear control technique that algebraically transforms the nonlinear system dynamics into a fully linear one. This allows for the application of linear control techniques to the exact, nonlinear system, providing better performance over a wider range of operating conditions.

## Project Structure

- `quadrotor_simulation.jl`: The main Pluto.jl notebook containing the LQR and Feedback Linearization controller designs, simulation, and interactive comparison.
- `Project.toml`: Specifies the Julia package dependencies for this project.
- `Manifest.toml`: Contains the exact versions of the dependencies, ensuring reproducibility.
- `matlab_implementation/`: Contains the original MATLAB implementation which was used as a reference.
- `projecct_proposal.pdf`: The original project proposal outlining the goals and motivation for the control system analysis.

## Next Steps

This project can be extended in several ways:

-   **Deploy as a Web Applet:** Export the Pluto notebook to a static HTML file to be hosted on GitHub Pages, making it accessible to anyone without needing to install Julia.
-   **Add Trajectory Tracking:** Implement trajectory tracking capabilities for the feedback linearization controller to follow a given path.
-   **Robustness Analysis:** Investigate the robustness of the controllers to model uncertainties and external disturbances.
-   **Extend to 3D:** Extend the simulation and control design to a full 3D quadrotor model.
