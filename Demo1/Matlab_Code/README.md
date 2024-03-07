# Simulink code
This folder contains the simulink models used for this project.

## Control_Model
This folder contains the main model used for controlling the rotation and position of the robot. It closely follows an example model given in class. It takes a desired position and angle and uses a PI controler to for position and angle and a P controler for velocities. These controllers implement antiwindup and saturation schemes to reduce overshoot. It is meant to model the controller code as closely as possible. 

## Experimental
This folder contains experimental files to determine transfer functions of different systems for the robot. Motors contains step response data for each individual motor and MatLab code and models to determine the transfer function of each motor. Velocity contains MatLab code and models to determine the transfer function of the instataneuos velocity and rotational velocity transfer functions for the robot. 

## Provided
This folder contains MatLab files that were given as examples in class. 