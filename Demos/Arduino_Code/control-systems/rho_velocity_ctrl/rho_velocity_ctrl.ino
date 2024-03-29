/**
 * @file rho_velocity_ctrl.ino
 * @author Blake Billharz, Ben Sprik
 * @brief Implement a velocity control system for the forward velocity
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
Encoder rightEnc(ENCR_A, ENCR_B);
Encoder leftEnc(ENCL_A, ENCL_B);
Tracker tracker(&rightEnc, &leftEnc);

// voltage converter
Vbase voltages;

// motor driver
DualMC33926MotorShield motorDriver;

// PID system
double rho_vel_des, rho_vel_act, Vforward;
double kp = 30, ki = 0, kd = 0;    // just proportional - KISS
PID controller(&rho_vel_act, &Vforward, &rho_vel_des, kp, ki, kd, DIRECT);

// test
double startTimeS;
double currTimeS;
const int NUM_SETPOINTS = 6;
// each setpoint will last for 5 seconds
double setpointTimeseries[NUM_SETPOINTS] = {0, 0.05, 0.15, 0.25, -0.15, 0};  // meters per second

// printing
double printIntervalMs = 50;
double lastPrintTimeMs = 0;


void setup() {

    // settings
    tracker.filterInputs(true);
    controller.SetMode(AUTOMATIC);
    controller.SetOutputLimits(-2*MAX_VOLTAGE, 2*MAX_VOLTAGE);

    // init
    Serial.begin(115200);
    Serial.println("Time Velocity");
    motorDriver.init();
    rho_vel_des = 0;
    rho_vel_act = tracker.getRhoSpeedMpS();
    startTimeS = millis();

}

void loop() {

    // get velocity
    tracker.update();
    rho_vel_act = tracker.getRhoSpeedMpS();
    // compute output
    controller.Compute();
    // update voltages
    voltages.setVoltages(Vforward, 0);
    // drive motor
    motorDriver.setM1Speed(-volts2speed(voltages.getVleft()));
    motorDriver.setM2Speed(-volts2speed(voltages.getVright()));


    // FSM to decide set point
    currTimeS = (millis()/1000.0) - startTimeS;

    if (currTimeS < NUM_SETPOINTS * 5) {
        int index = floor(currTimeS / 5);
        rho_vel_des = setpointTimeseries[index];
    }

    else {
        Serial << "Finished" << endl;
        while(1);
    }

    
    // print information
    if (millis() - lastPrintTimeMs >= printIntervalMs) {
        lastPrintTimeMs = millis();
        // time  setpoint velocity
        Serial.print(((lastPrintTimeMs/1000.0) - startTimeS));
        Serial.print(" ");
        Serial.print(rho_vel_des, 4);
        Serial.print(" ");
        Serial.print(rho_vel_act, 4);
        Serial.println();
    }


}