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
#include "utils/Tracker.h"
#include "utils/Vbase.h"
#include "utils/robotConstants.h"
#include <Encoder.h>
#include <PID_v1.h>
#include <DualMC33926MotorShield.h>

class Robot {
    public:

    Robot();

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
    DualMC33926MotorShield motorDriver;

    // control systems //
    

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