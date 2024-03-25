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
    Robot();

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

};


#endif





////////////////////////////////////   IMPLEMENTATION   //////////////////////////////////////////////////


Robot::Robot() {
    // initialization of aggregate classes
    rightEnc = new Encoder(ENCR_A, ENCR_B);
    leftEnc = new Encoder(ENCL_A, ENCL_B);
    this->tracker = new Tracker(rightEnc, leftEnc);

    // motor driver
    this->motorDriver = new DualMC33926MotorShield();
    this->motorDriver->init();

    // check for motor fault
    if (motorDriver->getFault()) {
        Serial << "Motor Driver fault. Exiting." << endl;
        while(1);
    }

    // control systems //
    phiVelCtrl = new PID(&phiVelAct, &Vrot, &phiVelDes, phiVelKp, phiVelKi, phiVelKd, DIRECT);
    phiPosCtrl = new PID(&phiPosAct, &phiVelDes, &phiPosDes, phiPosKp, phiPosKi, phiPosKd, DIRECT);
    rhoVelCtrl = new PID(&rhoVelAct, &Vforward, &rhoVelDes, rhoVelKp, rhoVelKi, rhoVelKd, DIRECT);
    rhoPosCtrl = new PID(&rhoPosAct, &rhoVelDes, &rhoPosDes, rhoPosKp, rhoPosKi, rhoPosKd, DIRECT);

    // all control systems default to off
    phiVelCtrl->SetMode(0);
    phiPosCtrl->SetMode(0);
    rhoVelCtrl->SetMode(0);
    rhoPosCtrl->SetMode(0);

    // sample time will remain constant
    phiVelCtrl->SetSampleTime(controllerSampleTimeMs);
    phiPosCtrl->SetSampleTime(controllerSampleTimeMs);
    rhoVelCtrl->SetSampleTime(controllerSampleTimeMs);
    rhoPosCtrl->SetSampleTime(controllerSampleTimeMs);
}

Robot::~Robot() {
    delete motorDriver, tracker, rightEnc, leftEnc;
    delete phiVelCtrl, phiPosCtrl, rhoVelCtrl, rhoPosCtrl;
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

//FIXME take out all x and y dependencies
//TODO add in missing rho position control
void Robot::goForwardM(double desDistanceMeters) {
    // tunings
    rhoVelCtrl->SetTunings(10, 0, 0);
    phiVelCtrl->SetTunings(2.5, 0, 0);
    phiPosCtrl->SetTunings(25, 12, 0);

    maxRhoVel = 0.2;
    maxPhiVel = pi/2;
    maxPhiAngle = 1000 * DEG_TO_RAD;

    rhoVelCtrl->SetOutputLimits(-MAX_VOLTAGE, MAX_VOLTAGE);
    phiVelCtrl->SetOutputLimits(-MAX_VOLTAGE, MAX_VOLTAGE);
    phiPosCtrl->SetOutputLimits(-maxPhiVel, maxPhiVel);

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
        yPosCtrl->Compute();
        phiPosCtrl->Compute();
        phiVelCtrl->Compute();
        // update voltages
        voltages.setVoltages(Vforward, Vrot);
        // drive motors
        motorDriver->setM1Speed(-volts2speed(voltages.getVright()));
        motorDriver->setM2Speed(-volts2speed(voltages.getVleft()));

        Serial << "ypos: ";
        Serial.print(yPosAct, 4);
        Serial << "  phiDes: ";
        Serial.print(phiPosDes, 4);
        Serial << "  phiPosDeg " << tracker->getPhiPosRad() * RAD_TO_DEG << "  |  ";

        Serial << "xPos: " << xPosAct << "  error: " << error << endl;

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