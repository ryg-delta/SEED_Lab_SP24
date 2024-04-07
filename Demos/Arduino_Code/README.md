# Arduino Code

## folders

### utils
The utils folder stores functions, classes, headers, etc. written by our team for use on the project. It also contains unit tests for all objects that can be unit tested.

### libs
The libs folder stores C++ libraries from outside sources.

### testing
Contains tests for arduino subsystems.

### control-systems
Contains code used to test individual robot commands.

### Demos
Contains code used in Demos.

## dependencies
Arduino libraries used

- Encoder
  - Header: Encoder.h
  - Link: https://github.com/PaulStoffregen/Encoder
  - Use: Keeps track of encoder positions.
- FIR filter
  - Header: FIR.h
  - Link: https://github.com/LeemanGeophysicalLLC/FIR_Filter_Arduino_Library
  - Use: Implements a simple FIR filter given filter coefficients.
- Streaming 
  - Header: Streaming.h
  - Link: https://github.com/janelia-arduino/Streaming
  - Use: Allows use of C++ style text-streaming with the << operator.
- PID
  - Header: PID_v1.h
  - Link: https://github.com/br3ttb/Arduino-PID-Library
  - Use: Implements a robust PID algorithm given the controller parameters, inputs, and outputs.
- DualMC33926MotorShield
  - Header: DualMC33926MotorShield.h
  - Link: https://github.com/pololu/dual-mc33926-motor-shield
  - Use: Controls the open-source motor driver shield.