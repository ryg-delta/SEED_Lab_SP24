# control-systems
This folder contains the Arduino code for our control systems. Much of this code is soon to be refactored and encapsulated inside a file in Arduino_code/utils

## Files
- phi_position_ctrl
  - Contains the code we used to develop and tune the angular position control system.
- phi_velocity_ctrl
  - Contains the code we used to develop and tune the angular velocity control system.
- rho_position_ctrl
   - Contains the code we used to develop and tune the radial position control system.
- rho_velocity_ctrl
   - Contains the code we used to develop and tune the radial velocity control system.
- position_ctrl
  - This is the main file for the control systems code. It puts combines all of the control systems and allows for time-series of setpoints for both the angular and radial components.
