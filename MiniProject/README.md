# Mini Project Overview

## Localization and Control
Goal: Position wheels to either a `1` or `0` position based on instructions from the RPi.

Structure: The Arduino recieves an I2C byte with the two LSBs corresponding to the left and right wheel position targets accordingly. A PI controller is then used to turn the wheel to the desired position.

Metrics: The target position will update as soon as a message is recieved. The average rise time of the system is about `0.4` Sec.