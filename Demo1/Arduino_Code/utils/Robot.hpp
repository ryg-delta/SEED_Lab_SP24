/**
 * @file Robot.h
 * @author Blake Billharz, Ben Sprik
 * @brief Implement control systems for discrete movement commands.
 * @version 0.1
 * @date 2024-03-02
 * 
 */

#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include "Tracker.h"
#include "Vbase.h"
#include "robotConstants.h"
#include <Encoder.h>
#include <PID_v1.h>
#include <DualMC33926MotorShield.h>
#include <Streaming.h>

class Robot {
    public:

    //TODO Add singleton pattern
    Robot(Encoder* rightEncoder, Encoder* leftEncoder, Tracker* tracker, DualMC33926MotorShield* motorDriver);

    ~Robot();

    void turnInPlace(double desAngleRad);

    void turnInPlaceDeg(double desAngleDeg);

    void goForwardM(double desDistanceMeters);

    void goForwardF(double desDistanceFeet);

    Tracker* getTracker();


    private:

    // keeps track of position
    Encoder* rightEnc;
    Encoder* leftEnc;
    Tracker* tracker;
    // voltage converter
    Vbase voltages;
    // motor driver
    DualMC33926MotorShield* motorDriver;

    // control systems //

    int controllerSampleTimeMs = 10;
    

    // phi velocity control system
    /*
        phi velocity -> V rotational
    */
    double phiVelDes, phiVelAct, Vrot;
    double phiVelKp = 2.5, phiVelKi = 0, phiVelKd = 0;    // just proportional - KISS
    PID* phiVelCtrl;


    // phi position control system
    /*
        phi position -> phi velocity
    */
    double phiPosDes, phiPosAct;
    double phiPosKp = 25, phiPosKi = 12, phiPosKd = 0;
    double maxPhiVel = pi/2;
    PID* phiPosCtrl;


    // y position control system
    /*
        y position -> phi position
    */
    double yPosDes, yPosAct;
    double yPosKp = 0, yPosKi = 0, yPosKd = 0;  //TODO - find controller gains
    double maxPhiAngle = pi/6;
    PID* yPosCtrl;


    // rho velocity control system
    /*
        rho velocity -> V forward
    */
    double rhoVelDes, rhoVelAct, Vforward;
    double rhoVelKp = 10, rhoVelKi = 0, rhoVelKd = 0;    // just proportional - KISS
    PID* rhoVelCtrl;


    // rho position control system
    /*
        rho position -> rho velocity
    */
    double rhoPosDes, rhoPosAct;
    double rhoPosKp = 14.24, rhoPosKi = 31.56, rhoPosKd = 0;
    double maxRhoVel = 0.35;
    PID* rhoPosCtrl;

    // x position control system
    /*
        x position -> rho velocity
    */
    double xPosDes, xPosAct;
    double xPosKp = 0, xPosKi = 0, xPosKd = 0;  //TODO - tune controller gains
    PID* xPosCtrl;

};


#endif





////////////////////////////////////   IMPLEMENTATION   //////////////////////////////////////////////////


Robot::Robot(Encoder* rightEncoder, Encoder* leftEncoder, Tracker* tracker, DualMC33926MotorShield* motorDriver) {
    // initialization of aggregate classes
    rightEnc = rightEncoder;
    leftEnc = leftEncoder;
    this->tracker = tracker;

    // motor driver
    this->motorDriver = motorDriver;
    // this->motorDriver->init();

    // control systems //
    phiVelCtrl = new PID(&phiVelAct, &Vrot, &phiVelDes, phiVelKp, phiVelKi, phiVelKd, DIRECT);
    phiPosCtrl = new PID(&phiPosAct, &phiVelDes, &phiPosDes, phiPosKp, phiPosKi, phiPosKd, DIRECT);
    yPosCtrl = new PID(&yPosAct, &phiPosDes, &yPosDes, yPosKp, yPosKi, yPosKd, P_ON_E, DIRECT);
    rhoVelCtrl = new PID(&rhoVelAct, &Vforward, &rhoVelDes, rhoVelKp, rhoVelKi, rhoVelKd, DIRECT);
    rhoPosCtrl = new PID(&rhoPosAct, &rhoVelDes, &rhoPosDes, rhoPosKp, rhoPosKi, rhoPosKd, DIRECT);
    xPosCtrl = new PID(&xPosAct, &rhoVelDes, &xPosDes, xPosKp, xPosKi, xPosKd, DIRECT);

    // all control systems default to off
    phiVelCtrl->SetMode(0);
    phiPosCtrl->SetMode(0);
    yPosCtrl->SetMode(0);
    rhoVelCtrl->SetMode(0);
    rhoPosCtrl->SetMode(0);
    xPosCtrl->SetMode(0);

    // sample time will remain constant
    phiVelCtrl->SetSampleTime(controllerSampleTimeMs);
    phiPosCtrl->SetSampleTime(controllerSampleTimeMs);
    yPosCtrl->SetSampleTime(controllerSampleTimeMs);
    rhoVelCtrl->SetSampleTime(controllerSampleTimeMs);
    rhoPosCtrl->SetSampleTime(controllerSampleTimeMs);
    xPosCtrl->SetSampleTime(controllerSampleTimeMs);
}

Robot::~Robot() {
    delete phiVelCtrl, phiPosCtrl, rhoVelCtrl, rhoPosCtrl, xPosCtrl, yPosCtrl;
}

void Robot::turnInPlace(double desAngleRad) {
    // tunings
    phiVelCtrl->SetTunings(2.5, 0, 0);
    phiPosCtrl->SetTunings(25, 12, 0);
    rhoVelCtrl->SetTunings(10, 0, 0);
    rhoPosCtrl->SetTunings(14.24, 31.56, 0);

    maxPhiVel = pi/2;
    maxRhoVel = 0.35;

    phiVelCtrl->SetOutputLimits(-MAX_VOLTAGE, MAX_VOLTAGE);
    phiPosCtrl->SetOutputLimits(-maxPhiVel, maxPhiVel);
    rhoVelCtrl->SetOutputLimits(-MAX_VOLTAGE, MAX_VOLTAGE);
    rhoPosCtrl->SetOutputLimits(-maxRhoVel, maxRhoVel);

    double delta = 1 * (pi/180);
    
    // turn on control systems
    phiVelCtrl->SetMode(AUTOMATIC);
    phiPosCtrl->SetMode(AUTOMATIC);
    rhoVelCtrl->SetMode(AUTOMATIC);
    rhoPosCtrl->SetMode(AUTOMATIC);

    // init
    phiPosDes = desAngleRad;
    phiVelDes = 0;
    phiVelAct = tracker->getPhiSpeedRpS();
    phiPosAct = tracker->getPhiPosRad();
    rhoPosDes = 0;
    rhoVelDes = 0;
    rhoVelAct = tracker->getRhoSpeedMpS();
    rhoPosAct = tracker->getRhoPosM();
    double error = phiPosDes - phiPosAct;

    // loop
    while (abs(error) > delta || abs(phiVelAct) > 0 || abs(rhoVelAct) > 0) {
        // update values
        tracker->update();
        phiVelAct = tracker->getPhiSpeedRpS();
        phiPosAct = tracker->getPhiPosRad();
        rhoVelAct = tracker->getRhoSpeedMpS();
        rhoPosAct = tracker->getRhoPosM();
        error = phiPosDes - phiPosAct;
        // compute output
        phiPosCtrl->Compute();
        phiVelCtrl->Compute();
        rhoPosCtrl->Compute();
        rhoVelCtrl->Compute();
        // update voltages
        voltages.setVoltages(Vforward, Vrot);
        // drive motor
        motorDriver->setM1Speed(-volts2speed(voltages.getVright()));
        motorDriver->setM2Speed(-volts2speed(voltages.getVleft()));
    }

    // turn off control systems
    phiVelCtrl->SetMode(0);
    phiPosCtrl->SetMode(0);
    rhoVelCtrl->SetMode(0);
    rhoPosCtrl->SetMode(0);

    // TODO could maybe return some kind of indication of the final error.
}

void Robot::turnInPlaceDeg(double desAngleDeg) {
    turnInPlace(desAngleDeg * (pi/180));
}

void Robot::goForwardM(double desDistanceMeters) {
    // tunings
    rhoVelCtrl->SetTunings(10, 0, 0);
    xPosCtrl->SetTunings(14.24, 31.56, 0);
    phiVelCtrl->SetTunings(2.5, 0, 0);
    phiPosCtrl->SetTunings(25, 12, 0);
    yPosCtrl->SetTunings(0.1745, 0, 0);  // 1 deg/mm

    maxRhoVel = 0.5;
    maxPhiVel = pi/2;
    maxPhiAngle = 1000 * DEG_TO_RAD;

    rhoVelCtrl->SetOutputLimits(-MAX_VOLTAGE, MAX_VOLTAGE);
    xPosCtrl->SetOutputLimits(-maxRhoVel, maxRhoVel);
    phiVelCtrl->SetOutputLimits(-MAX_VOLTAGE, MAX_VOLTAGE);
    phiPosCtrl->SetOutputLimits(-maxPhiVel, maxPhiVel);
    yPosCtrl->SetOutputLimits(-maxPhiAngle, maxPhiAngle);

    double delta = 0.001;  // 1 mm

    // init
    xPosAct = tracker->getXPosM();
    xPosDes = desDistanceMeters;
    yPosAct = tracker->getYPosM();
    yPosDes = 0;
    phiPosDes = 0;
    double error = xPosDes - xPosAct;

    // turn on control systems
    rhoVelCtrl->SetMode(AUTOMATIC);
    xPosCtrl->SetMode(AUTOMATIC);
    phiVelCtrl->SetMode(AUTOMATIC);
    phiPosCtrl->SetMode(AUTOMATIC);
    yPosCtrl->SetMode(AUTOMATIC);

    // loop
    while (abs(error) > delta || abs(phiVelAct) > 0 || abs(rhoVelAct) > 0) {
        // update values
        tracker->update();
        rhoVelAct = tracker->getRhoSpeedMpS();
        xPosAct = tracker->getXPosM();
        phiVelAct = tracker->getPhiSpeedRpS();
        phiPosAct = tracker->getPhiPosRad();
        yPosAct = tracker->getYPosM();
        error = xPosDes - xPosAct;
        // compute controller outputs
        xPosCtrl->Compute();
        rhoVelCtrl->Compute();
        Serial << "///////" << endl;
        yPosCtrl->Compute();
        Serial << "///////" << endl;
        phiPosCtrl->Compute();
        phiVelCtrl->Compute();
        // update voltages
        voltages.setVoltages(Vforward, Vrot);
        // drive motors
        motorDriver->setM1Speed(-volts2speed(voltages.getVright()));
        motorDriver->setM2Speed(-volts2speed(voltages.getVleft()));

        Serial << "ypos: ";
        Serial.print(yPosAct, 4);
        Serial << " phiDes: ";
        Serial.print(phiPosDes, 4);
        Serial << " y-error: ";
        Serial.print(yPosDes - yPosAct, 5);
        Serial << " phiPosDeg " << tracker->getPhiPosRad() * RAD_TO_DEG;
        Serial << " | ";
        Serial << "xpos: " << xPosAct << " rhoVelDes: " << rhoVelDes << endl;

        delay(10);
    }

    // turn off control systems
    rhoVelCtrl->SetMode(0);
    xPosCtrl->SetMode(0);
    phiVelCtrl->SetMode(0);
    phiPosCtrl->SetMode(0);
    yPosCtrl->SetMode(0);
}

void Robot::goForwardF(double desDistanceFeet) {
    goForwardM(desDistanceFeet / FEET_PER_MEETER);
}

Tracker* Robot::getTracker() {
    return tracker;
}