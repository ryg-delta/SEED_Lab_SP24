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

// phi velocity control system
double phiVelDes, phiVelAct, Vrot;
double phiVelKp = 2.5, phiVelKi = 0, phiVelKd = 0;    // just proportional - KISS
PID phiVelCtrl(&phiVelAct, &Vrot, &phiVelDes, phiVelKp, phiVelKi, phiVelKd, DIRECT);

// phi position control system
double phiPosDes, phiPosAct;
double phiPosKp = 20, phiPosKi = 12, phiPosKd = 0;
double maxPhiVel = pi/2;
PID phiPosCtrl(&phiPosAct, &phiVelDes, &phiPosDes, phiPosKp, phiPosKi, phiPosKd, DIRECT);

// rho velocity control system
double rhoVelDes, rhoVelAct, Vforward;
double rhoVelKp = 10, rhoVelKi = 0, rhoVelKd = 0;    // just proportional - KISS
PID rhoVelCtrl(&rhoVelAct, &Vforward, &rhoVelDes, rhoVelKp, rhoVelKi, rhoVelKd, DIRECT);

// rho position control system
double rhoPosDes, rhoPosAct;
double rhoPosKp = 14.24, rhoPosKi = 31.56, rhoPosKd = 0;
double maxRhoVel = 0.5;
PID rhoPosCtrl(&rhoPosAct, &rhoVelDes, &rhoPosDes, rhoPosKp, rhoPosKi, rhoPosKd, DIRECT);

// test
double startTimeS;
double currTimeS;
const int NUM_SETPOINTS = 4;
// each setpoint will last for 25 seconds
double degreesTimeseries[NUM_SETPOINTS] = {90, 90, 0};
double feetTimeseries[NUM_SETPOINTS] = {0, 7, 7};
double rhoSetpointTimeseries[NUM_SETPOINTS];
double phiSetpointTimeseries[NUM_SETPOINTS];

// printing
double printIntervalMs = 10;
double lastPrintTimeMs = 0;

void setup() {

    // settings
    tracker.filterInputs(true);
    phiVelCtrl.SetMode(AUTOMATIC);
    phiPosCtrl.SetMode(AUTOMATIC);
    rhoVelCtrl.SetMode(AUTOMATIC);
    rhoPosCtrl.SetMode(AUTOMATIC);
    phiVelCtrl.SetOutputLimits(-MAX_VOLTAGE, MAX_VOLTAGE);
    rhoVelCtrl.SetOutputLimits(-MAX_VOLTAGE, MAX_VOLTAGE);
    phiPosCtrl.SetOutputLimits(-maxPhiVel, maxPhiVel);
    rhoPosCtrl.SetOutputLimits(-maxRhoVel, maxRhoVel);

    // init
    Serial.begin(115200);
    Serial.println("Time  Setpoint  Position");
    motorDriver.init();
    phiPosDes = 0;
    phiVelDes = 0;
    phiVelAct = tracker.getPhiSpeedRpS();
    phiPosAct = tracker.getPhiPosRad();
    rhoPosDes = 0;
    rhoVelDes = 0;
    rhoVelAct = tracker.getRhoSpeedMpS();
    rhoPosAct = tracker.getRhoPosM();
    startTimeS = millis();

    // convert setpoints to radians and feet
    for (int i = 0; i < NUM_SETPOINTS; i++) {
      phiSetpointTimeseries[i] = degreesTimeseries[i] * (pi/180);
      rhoSetpointTimeseries[i] = feetTimeseries[i] / FEET_PER_MEETER;
    }

    delay(3000);

}

void loop() {

    // update values
    tracker.update();
    phiVelAct = tracker.getPhiSpeedRpS();
    phiPosAct = tracker.getPhiPosRad();
    rhoVelAct = tracker.getRhoSpeedMpS();
    rhoPosAct = tracker.getRhoPosM();
    // compute output
    phiPosCtrl.Compute();
    phiVelCtrl.Compute();
    rhoPosCtrl.Compute();
    rhoVelCtrl.Compute();
    // update voltages
    voltages.setVoltages(Vforward, Vrot);
    // drive motor
    motorDriver.setM1Speed(- volts2speed(voltages.getVright()));
    motorDriver.setM2Speed(- volts2speed(voltages.getVleft()));


    // FSM to decide set point
    currTimeS = (millis()/1000.0) - startTimeS;

    if (currTimeS < NUM_SETPOINTS * 15) {
        int index = floor(currTimeS / 15);
        phiPosDes = phiSetpointTimeseries[index];
        rhoPosDes = rhoSetpointTimeseries[index];
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
        Serial.print(rhoPosDes, 4);
        Serial.print(" ");
        Serial.print(rhoPosAct, 4);
        Serial.print(" ");
        Serial.print(Vforward, 4);
        Serial.print(" ");
        Serial.print(Vrot);
        Serial.println();
    }


}