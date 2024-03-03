/**
 * @file robotConstants.h
 * @author Blake Billharz
 * @brief Define physical constants used for the entire robot.
 * @version 0.1
 * @date 2024-02-27
 * 
 */

#ifndef ROBOTCONSTANTS_H
#define ROBOTCONSTANTS_H

#define WHEEL_RADIUS_M  0.074 
#define WHEEL_BASE_M    0.27 
#define ENC_CNT_PER_REV  3200

#define pi 3.141592

#define K_VBAR       0.0585
#define SIGMA_VBAR   5
#define K_DELTAV     0.405
#define SIGMA_DELTAV 5

#define FEET_PER_MEETER  3.28084
#define meters2feet(m)   (m * 3.28084)

/*
Left motor:
    encoderPwr -> 3v3
    encoderGnd -> GND
    encoderA -> 2
    encoderB -> 5
    direction -> 7
    pwm -> 9
Right motor:
    encoderPwr -> 3v3
    encoderGnd -> GND
    encoderA -> 3
    encoderB -> 6
    direction -> 8
    pwm -> 10
I2C:
    SCL -> A5
    SDA -> A4
    GND -> GND
*/

#define ENCL_A 2
#define ENCR_A 3
#define nD2    4
#define ENCL_B 5
#define ENCR_B 6
#define M1DIR  7
#define M2DIR  8
#define M1PWM  9
#define M2PWM  10



#endif