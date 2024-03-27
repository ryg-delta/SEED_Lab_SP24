# Experimental
This folder contains the simulink models used to determine the transfer functions of the robot.

## forwardVelocitySim.slx
Simulates the robot moving forward.

## motorRotationalVelocityData.mat
Contains rotational velocity data (time,voltage,rad/s).

## motorStepVelocityInput.mat
Contains forward velocity data (time,voltage,m/s).

## ReadFromArduino.mlx
Reads data from Arduino Serial port while experiments are in progress. 

## rotationalVelocityControl.slx
Simulates rotational velocity.

## rotationalVelocitySim.slx
Simulates rotational velocity.

## RunForwardVelocitySim.m
Runs forwardVelocitySim.slx and reads motorStepVelocityInput.mat and plots results to determine transfer function.

## RunRotationalVelocitySim.m
Runs rotationalVelocitySim.slx and reads motorRotationalVelocityData.mat and plots results to determine transfer function.


