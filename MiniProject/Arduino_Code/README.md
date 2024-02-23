      
# Arduino to RPI Interfacing
- SCL: A5 to Pin 5
- SDA: A4 to Pin 3
- GND: GND to Pin 6
- Arduino connections are specified in application files

# Description of Arduino Code
## Main File
- **wheel_position_control**
- - This file is the main file for Mini Project 1. The Arduino takes wheel location input via I2C from the 
RasPi and rotates the wheels to reflect the current quadrant of the marker seen by the RasPi camera. 
  - First, pins, I2C, Motors, and Motor Encoders are setup.
  - Then, using an I2C ISR, input from the RasPi is processed to determine the requested orientation of the wheels.
  - Next, the desired wheel location is applied to our PI controller.
  - The motors are driven and the wheels are rotated to their target positions using the output of our controller.
  - The Arduino keeps the current wheel position until it recieves more input from the RasPi, at which point the process repeats. 

## Suplementary Files
- **motor_velocity_control**
  - This is the Assignment 2 application code. It was used as a starting point for Mini Project 1. It
takes an input of a desired speed in radians/second and spins one wheel at that speed.
- **wheel_position_control_test**
  - This is the file where Mini Project 1 code was developed and tested. This file tests the functionality of our control system by 
rotating each wheel 3.14 radians every 5 seconds and printing various variables at regular intervals. 