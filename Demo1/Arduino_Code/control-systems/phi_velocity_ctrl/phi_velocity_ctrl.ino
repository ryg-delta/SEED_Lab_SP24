/**
 * @file phi_velocity_ctrl.ino
 * @author Blake Billharz, Ben Sprik
 * @brief Implement a velocity control system for the angular velocity
 * @version 0.1
 * @date 2024-03-02
 * 
 */

#include <Arduino.h>
#include "utils/Tracker.h"
#include "utils/Vbase.h"
#include "utils/robotConstants.h"
#include <Encoder.h>
#include <PID_v1.h>
#include <Streaming.h>
#include <DualMC33926MotorShield.h>

// keeps track of position
Tracker tracker(&Encoder(ENCR_A, ENCR_B), &Encoder(ENCL_A, ENCL_B));

// voltage converter
Vbase voltages;

// motor driver
DualMC33926MotorShield motorDriver;

// PID system
double phi_vel_des, phi_vel_act, Vrot;
//FIXME need to find kp
double kp = 2, ki = 0, kd = 0;    // just proportional - KISS
PID controller(&phi_vel_des, &Vrot, &phi_vel_des, kp, ki, kd, DIRECT);

// test
double startTimeS;
double currTimeS;
const int NUM_SETPOINTS = 4;
// each setpoint will last for 5 seconds
double setpointTimeseries[NUM_SETPOINTS] = {0, 1.5, -3, 0};  // radians per second

// printing
double printIntervalMs = 50;
double lastPrintTimeMs =0;


void setup() {

    // settings
    tracker.filterInputs(true);
    controller.SetMode(AUTOMATIC);
    controller.SetOutputLimits(-2*MAX_VOLTAGE, 2*MAX_VOLTAGE);

    // init
    Serial.begin(115200);
    motorDriver.init();
    phi_vel_des = 0;
    startTimeS = millis();

}

void loop() {

    // get velocity
    tracker.update();
    phi_vel_act = tracker.getPhiSpeedRpS();
    // compute output
    controller.Compute();
    // update voltages
    voltages.setVoltages(0, Vrot);
    // drive motor
    motorDriver.setM1Speed(volts2speed(voltages.getVleft()));
    motorDriver.setM2Speed(volts2speed(voltages.getVright()));


    // FSM to decide set point
    currTimeS = (millis()/1000.0) - startTimeS;

    if (currTimeS < NUM_SETPOINTS * 5) {
        int index = floor(currTimeS / 5);
        phi_vel_des = setpointTimeseries[index];
    }

    else {
        Serial << "Finished" << endl;
        while(1);
    }

    
    // print information
    if (millis() - lastPrintTimeMs >= printIntervalMs) {
        lastPrintTimeMs = millis();
        // time  angular-velocity
        Serial << ((lastPrintTimeMs/1000.0) - startTimeS) << " " << phi_vel_act << endl;
    }


}