# COMP0128: Robotic Control Theory and Systems Coursework 1

## Overview

This repository contains MATLAB code for simulating a differential drive robot with obstacle avoidance capabilities. The coursework tasks involve estimating wheel radii and distances between wheels to perform wheel odometry, and implementing various controllers to navigate around obstacles. For details please see `COMP0128__Robotic_Control_Theory_and_Systems_CW1.pdf`.

## Installation

To run the simulations, ensure MATLAB is installed on your system.

## Running the Simulations

Each task in the coursework is separated into different MATLAB script files within respective directories (Q1 to Q6). Follow the instructions below to run each simulation.

### Q1: Wheel Radius and Distance Calculation

- Navigate to the `Q1` directory.
- Run the `Sim_DifferentialDriveWithObstacles.m` script to start the simulation.
- This script calculates the radius and distance between the wheels of the robot using odometry and sensor data.

### Q2: Obstacle Navigation

- Navigate to the `Q2` directory.
- Run the `Sim_DifferentialDriveWithObstacles.m` script.
- The robot will navigate around an obstacle counterclockwise while maintaining a constant distance.

### Q3: Odometry-based Controller

- Navigate to the `Q3` directory.
- Run the `Sim_DifferentialDriveWithObstacles.m` script.
- This controller uses only odometry data to perform the navigation task.

### Q4: System Transfer Function Estimation

- Navigate to the `Q4` directory.
- Run the `Sim_DifferentialDriveWithObstacles.m` script.
- The script estimates system transfer function parameters using robot measurements.

### Q5: PD Controller Tuning

- Navigate to the `Q5` directory.
- Open the `pidTuning.m` and `model.slx` for PD parameter tuning and Simulink modeling.

### Q6: Digital Filter for PD Controller

- Navigate to the `Q6` directory.
- This section includes the discretization of the PD controller and its implementation as a digital filter.
- Course instructors and TAs
