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

    void driveInCircleM(double circleRadiusMeters, double periodSec);

    void driveInCircleF(double circleRadiusFeet, double periodSec);

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

void Robot::goForwardM(double desDistanceMeters) {
    // tunings
    rhoVelCtrl->SetTunings(10, 0, 0);
    rhoPosCtrl->SetTunings(10, 10, 0); //FIXME tunings
    phiVelCtrl->SetTunings(2.5, 0, 0);
    phiPosCtrl->SetTunings(25, 12, 0);

    double maxRhoVel = 0.2;  //FIXME need faster performance
    double maxPhiVel = pi/2;  
    double maxPhiAngle = 100 * DEG_TO_RAD;

    rhoVelCtrl->SetOutputLimits(-MAX_VOLTAGE, MAX_VOLTAGE);
    rhoPosCtrl->SetOutputLimits(-maxRhoVel, maxRhoVel);
    phiVelCtrl->SetOutputLimits(-MAX_VOLTAGE, MAX_VOLTAGE);
    phiPosCtrl->SetOutputLimits(-maxPhiVel, maxPhiVel);

    double delta = 0.001;  // 1 mm

    // init
    phiPosDes = 0;
    phiVelDes = 0;
    phiVelAct = tracker->getPhiSpeedRpS();
    phiPosAct = tracker->getPhiPosRad();
    rhoPosDes = desDistanceMeters;
    rhoVelDes = 0;
    rhoVelAct = tracker->getRhoSpeedMpS();
    rhoPosAct = tracker->getRhoPosM();
    double error = rhoPosDes - rhoPosAct;

    // turn on control systems
    rhoVelCtrl->SetMode(AUTOMATIC);
    rhoPosCtrl->SetMode(AUTOMATIC);
    phiVelCtrl->SetMode(AUTOMATIC);
    phiPosCtrl->SetMode(AUTOMATIC);

    // loop
    while (abs(error) > delta || abs(phiVelAct) > 0 || abs(rhoVelAct) > 0) {
        // update values
        tracker->update();
        rhoVelAct = tracker->getRhoSpeedMpS();
        rhoPosAct = tracker->getRhoPosM();
        phiVelAct = tracker->getPhiSpeedRpS();
        phiPosAct = tracker->getPhiPosRad();
        error = rhoPosDes - rhoPosAct;
        // compute controller outputs
        rhoPosCtrl->Compute();
        rhoVelCtrl->Compute();
        phiPosCtrl->Compute();
        phiVelCtrl->Compute();
        // update voltages
        voltages.setVoltages(Vforward, Vrot);
        // drive motors
        motorDriver->setM1Speed(-volts2speed(voltages.getVright()));
        motorDriver->setM2Speed(-volts2speed(voltages.getVleft()));
    }

    // turn off control systems
    rhoVelCtrl->SetMode(0);
    rhoPosCtrl->SetMode(0);
    phiVelCtrl->SetMode(0);
    phiPosCtrl->SetMode(0);

}

void Robot::goForwardF(double desDistanceFeet) {
    goForwardM(desDistanceFeet / FEET_PER_MEETER);
}

void Robot::driveInCircleM(double circleRadiusMeters, double periodSec) {

     // tunings
    phiVelCtrl->SetTunings(2.5, 0, 0);
    rhoVelCtrl->SetTunings(10, 0, 0);

    maxPhiVel = pi/2;
    maxRhoVel = 0.35;

    // FIXME - what should delta be??? should velocitys have different deltas?
    double deltaRhoPos = 1 * (pi/180);   
    double deltaPhiPos = 0.001;  // 1 mm

    phiVelCtrl->SetOutputLimits(-MAX_VOLTAGE, MAX_VOLTAGE);
    rhoVelCtrl->SetOutputLimits(-MAX_VOLTAGE, MAX_VOLTAGE);
    
    // turn on control systems
    phiVelCtrl->SetMode(AUTOMATIC);
    rhoVelCtrl->SetMode(AUTOMATIC);

    // init
    phiVelDes = 2*pi*circleRadiusMeters / periodSec;
    phiVelAct = tracker->getPhiSpeedRpS();
    phiPosAct = tracker->getPhiPosRad();
    rhoVelDes = 2*pi / periodSec;
    rhoVelAct = tracker->getRhoSpeedMpS();
    rhoPosAct = tracker->getRhoPosM();
    
    // start the robot
    while (abs(phiPosAct) < 2*pi) {
        // update values
        tracker->update();
        rhoVelAct = tracker->getRhoSpeedMpS();
        rhoPosAct = tracker->getRhoPosM();
        phiVelAct = tracker->getPhiSpeedRpS();
        phiPosAct = tracker->getPhiPosRad();
        // compute controller outputs
        rhoVelCtrl->Compute();
        phiVelCtrl->Compute();
        // update voltages
        voltages.setVoltages(Vforward, Vrot);
        // drive motors
        motorDriver->setM1Speed(-volts2speed(voltages.getVright()));
        motorDriver->setM2Speed(-volts2speed(voltages.getVleft()));
    }
    // stop the robot where it is
    goForwardM(0);
     // turn off control systems
    rhoVelCtrl->SetMode(0);
    phiVelCtrl->SetMode(0);

}

void Robot::driveInCircleF(double circleRadiusFeet, double periodSec) {
    driveInCircleM(circleRadiusFeet / FEET_PER_MEETER, periodSec);
}

Tracker* Robot::getTracker() {
    return tracker;
}