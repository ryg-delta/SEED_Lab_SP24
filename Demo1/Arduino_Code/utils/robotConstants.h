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

// wheels
#define WHEEL_RADIUS_M  0.074 
#define WHEEL_BASE_M    0.27 
#define ENC_CNT_PER_REV  3200

// laplace domain
#define K_VBAR       0.0585
#define SIGMA_VBAR   5
#define K_DELTAV     0.405
#define SIGMA_DELTAV 5

// math
#define pi 3.141592
#define FEET_PER_MEETER  3.28084
#define meters2feet(m)   (m * 3.28084)

// motors

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
#define ENCL_B 5
#define ENCR_B 6

#define MAX_VOLTAGE 7.3173
#define UNUSABLE_VOLTAGE 0.4827
#define volts2pwm(volts) ((volts/(MAX_VOLTAGE+UNUSABLE_VOLTAGE))*255)
#define volts2speed(volts) ((volts/(MAX_VOLTAGE+UNUSABLE_VOLTAGE))*400) // max speed for the motor driver library is +- 400


// control system
#define PHI_VEL_KP 3
#define PHI_VEL_KI 0
#define PHI_VEL_KD 0

#define PHI_POS_KP 35
#define PHI_POS_KI 12
#define PHI_POS_KD 0

#define RHO_VEL_KP 30
#define RHO_VEL_KI 0
#define RHO_VEL_KD 0

#define RHO_POS_KP 14.24
#define RHO_POS_KI 31.56
#define RHO_POS_KD 0



#endif