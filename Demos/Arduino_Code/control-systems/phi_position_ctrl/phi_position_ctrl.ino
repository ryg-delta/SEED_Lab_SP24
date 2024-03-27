/**
 * @file phi_position_ctrl.ino
 * @author Blake Billharz, Ben Sprik
 * @brief Implement the control system modeled in simulink to control the angular position of the robot.
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
double phiVelDes, phiVelAct, Vrot;
double kpv = 3, kiv = 0, kdv = 0;    // just proportional - KISS
PID phiVelCtrl(&phiVelAct, &Vrot, &phiVelDes, kpv, kiv, kdv, DIRECT);

// Position control system
double phiPosDes, phiPosAct;
double kpp = 35, kip = 12, kdp = 0;
double maxPhiVel = pi/2;
PID phiPosCtrl(&phiPosAct, &phiVelDes, &phiPosDes, kpp, kip, kdp, DIRECT);

// test
double startTimeS;
double currTimeS;
const int NUM_SETPOINTS = 4;
// each setpoint will last for 10 seconds
double degreesTimeseries[NUM_SETPOINTS] = {0, 90, -90, 0};
double setpointTimeseries[NUM_SETPOINTS];

// printing
double printIntervalMs = 10;
double lastPrintTimeMs = 0;

void setup() {

    // settings
    tracker.filterInputs(true);
    phiVelCtrl.SetMode(AUTOMATIC);
    phiPosCtrl.SetMode(AUTOMATIC);
    phiVelCtrl.SetOutputLimits(-MAX_VOLTAGE, MAX_VOLTAGE);
    phiPosCtrl.SetOutputLimits(-maxPhiVel, maxPhiVel);

    // init
    Serial.begin(115200);
    Serial.println("Time  Setpoint  Position");
    motorDriver.init();
    phiPosDes = 0;
    phiVelDes = 0;
    phiVelAct = tracker.getPhiSpeedRpS();
    phiPosAct = tracker.getPhiPosRad();
    startTimeS = millis();

    // convert setpoints to radians per second
    for (int i = 0; i < NUM_SETPOINTS; i++) {
      setpointTimeseries[i] = degreesTimeseries[i] * (pi/180);
    }

}

void loop() {

    // update values
    tracker.update();
    phiVelAct = tracker.getPhiSpeedRpS();
    phiPosAct = tracker.getPhiPosRad();
    // compute output
    phiPosCtrl.Compute();
    phiVelCtrl.Compute();
    // update voltages
    voltages.setVoltages(0, Vrot);
    // drive motor
    motorDriver.setM1Speed(volts2speed(voltages.getVleft()));
    motorDriver.setM2Speed(volts2speed(voltages.getVright()));


    // FSM to decide set point
    currTimeS = (millis()/1000.0) - startTimeS;

    if (currTimeS < NUM_SETPOINTS * 7) {
        int index = floor(currTimeS / 7);
        phiPosDes = setpointTimeseries[index];
    }

    else {
        Serial << "Finished" << endl;
        motorDriver.setM1Speed(volts2speed(0));
        motorDriver.setM2Speed(volts2speed(0));
        while(1);
    }

    
    // print information
    if (millis() - lastPrintTimeMs >= printIntervalMs) {
        lastPrintTimeMs = millis();
        // time  setpoint  angular-position
        Serial.print(((lastPrintTimeMs/1000.0) - startTimeS));
        Serial.print(" ");
        Serial.print(phiPosDes, 4);
        Serial.print(" ");
        Serial.print(phiPosAct, 4);
        Serial.print(" ");
        Serial.print(phiVelDes, 4);
        Serial.println();
    }


}