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



#endif