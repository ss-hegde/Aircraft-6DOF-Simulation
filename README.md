# Aircraft-6DOF-Simulation
Simulink-based nonlinear 6-DOF aircraft model with joystick input and FlightGear visualization.

---
This repository contains a nonlinear 6-degree-of-freedom (6-DOF) aircraft simulation model developed in **MATLAB Simulink**. The model supports **joystick input** and is capable of **real-time flight visualization in FlightGear**.

---
## Features

- Nonlinear 6-DOF rigid-body dynamics
- Modular Simulink model architecture
- External inputs for joystick, gravity, and environmental factors (expandable) 
- Outputs compatible with FlightGear for real-time flight visualization
- Clean and reusable structure for control system integration

## Project Structure

- **init.m**: Provides initial conditions
- **aircraft_6DOF_model.m**: Nonlinear aircraft model
- **plant.slx**: Simulink model for Simulation
- **aircraft_trimLin.slx**: simulink model for trimming and linearization 

## How to run the Simulation

-1. Install FlightGear (if not already installed)
-2. Open MATLAB and navigate to the project folder
-3. Run the provided FlightGear launch script:**runfg.bat**
-4. Run the **init.m** MATLAB file to initialize the model.
-5. Open the Simulink model **plant.slx** and start the simulation.

---

## Further Inprovements

- Add autopilot or control systems 
- Integrate full 6-DOF trim and linearization tools
- Improve terrain and wind modeling
- Record and plot state trajectories for post-analysis

## About the Author

Hi! I am Sharath Hegde, an aerospace engineer passionate about simulation, control systems, and flight dynamics.
This project is part of my ongoing work in developing and analyzing 6-DOF aircraft models for research in flight dynamics, control systems, and simulation environments.

Feel free to connect or drop feedback!