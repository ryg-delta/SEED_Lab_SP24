/**
 * @file rho_position_ctrl.ino
 * @author Blake Billharz, Ben Sprik
 * @brief Implement the control system modeled in simulink to control the radial position of the robot.
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

// Velocity control system
double rhoVelDes, rhoVelAct, Vforward;
//FIXME need to find kp
double kpv = 5, kiv = 0, kdv = 0;    // just proportional - KISS
PID rhoVelCtrl(&rhoVelAct, &Vforward, &rhoVelDes, kpv, kiv, kdv, DIRECT);

// Position control system
double rhoPosDes, rhoPosAct;
//FIXME determine control parameters
double kpp = 0, kip = 0, kdp = 0;
double maxRhoVel;
PID rhoPosCtrl(&rhoPosAct, &rhoVelDes, &rhoPosDes, kpp, kip, kdp, DIRECT);

// test
double startTimeS;
double currTimeS;
const int NUM_SETPOINTS = 4;
// each setpoint will last for 5 seconds
double setpointTimeseries[NUM_SETPOINTS] = {0, 2, 0};

// printing
double printIntervalMs = 50;
double lastPrintTimeMs = 0;

void setup() {

    // settings
    tracker.filterInputs(false);
    rhoVelCtrl.SetMode(AUTOMATIC);
    rhoPosCtrl.SetMode(AUTOMATIC);
    rhoVelCtrl.SetOutputLimits(-2*MAX_VOLTAGE, 2*MAX_VOLTAGE);
    rhoPosCtrl.SetOutputLimits(-maxRhoVel, maxRhoVel);

    // init
    Serial.begin(115200);
    Serial.println("Time  Setpoint  Position");
    motorDriver.init();
    rhoPosDes = 0;
    rhoVelDes = 0;
    rhoVelAct = tracker.getRhoSpeedMpS();
    rhoPosAct = tracker.getRhoPosM();
    startTimeS = millis();

}

void loop() {

    // update values
    tracker.update();
    rhoVelAct = tracker.getRhoSpeedMpS();
    rhoPosAct = tracker.getRhoPosM();
    // compute output
    rhoPosCtrl.Compute();
    rhoVelCtrl.Compute();
    // update voltages
    voltages.setVoltages(Vforward, 0);
    // drive motor
    motorDriver.setM1Speed(volts2speed(voltages.getVleft()));
    motorDriver.setM2Speed(volts2speed(voltages.getVright()));


    // FSM to decide set point
    currTimeS = (millis()/1000.0) - startTimeS;

    if (currTimeS < NUM_SETPOINTS * 5) {
        int index = floor(currTimeS / 5);
        rhoPosDes = setpointTimeseries[index];
    }

    else {
        Serial << "Finished" << endl;
        while(1);
    }

    
    // print information
    if (millis() - lastPrintTimeMs >= printIntervalMs) {
        lastPrintTimeMs = millis();
        // time  setpoint  radial-position
        Serial.print(((lastPrintTimeMs/1000.0) - startTimeS));
        Serial.print(" ");
        Serial.print(rhoPosDes, 4);
        Serial.print(" ");
        Serial.print(rhoPosAct, 4);
        Serial.println();
    }


}