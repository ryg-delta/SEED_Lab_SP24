# Arduino Code

## folders

### utils
The utils folder stores functions, classes, headers, etc. written by our team for use on the project. It also contains unit tests for all objects that can be unit tested.

### libs
The libs folder stores C++ libraries from outside sources.

### testing
Contains tests for arduino subsystems.


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