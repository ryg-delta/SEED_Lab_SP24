# Simulink code
This folder contains the simulink models used for this project.

## position_control
This file contains the main model used for controlling the position of the wheel. A simple PI controller was implemented using the current position as the input and the desired position as the target. Anti-ntegrator windup was implemented to reduce overshoot.

## motor_control
This file contains the velocity control system used in the previous arduino assignment. It is included in this folder mainly for reference.