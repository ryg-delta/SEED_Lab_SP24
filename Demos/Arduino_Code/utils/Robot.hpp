/**
 * @file Robot.h
 * @author Blake Billharz, Ben Sprik
 * @brief This class implements all necessary interfaces to control the robot based on discrete movement commands.
 *        A list of commands can be seen below in the class declaration.
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

    /**
     * @brief Turns in place, positive numbers correspond to CCW rotation.
     * 
     * @param desAngleRad The amount of radians to turn
     */
    void turnInPlace(double desAngleRad);

    /**
     * @brief Turns in place, positive numbers correspond to CCW rotation.
     * 
     * @param desAngleDeg The amount of degrees to turn
     */
    void turnInPlaceDeg(double desAngleDeg);

    /**
     * @brief Turns in increments, pausing for a second between turns, until stopCondition is met (becomes true)
     * 
     * @param stopCondition 
     */
    void scan(volatile bool& stopCondition);

    /**
     * @brief Turn slowly until stopCondition is met (becomes true)
     * 
     * @param stopCondition 
     */
    void scanContinuous(volatile bool& stopCondition);

    /**
     * @brief Drive in a circle to scan
     * 
     * @param stopCondition 
     */
    void scanInCircle(volatile bool& stopCondition);

    /**
     * @brief Finds and goes to the closest marker
     * 
     * @param markerSeen 
     * @param distanceToMarker 
     * @param angleToMarker
     */
    void findClosestMarker(volatile bool& markerSeen, volatile double& distanceToMarker, volatile double& angleToMarker);

     /**
     * @brief Finds and goes to marker 4 ft away
     * 
     * @param markerSeen 
     * @param distanceToMarker 
     * @param angleToMarker
     */
    void findMarker4ft(volatile bool& markerSeen, volatile double& distanceToMarker, volatile double& angleToMarker);

    /**
     * @brief Goes foreward in a straight line
     * 
     * @param desDistanceMeters The distance in meters to go forward
     */
    void goForwardM(double desDistanceMeters);

    /**
     * @brief Goes foreward in a straight line
     * 
     * @param desDistanceFeet The distance in feet to go forward
     */
    void goForwardF(double desDistanceFeet);

    /**
     * @brief On marker lock, this method will go a marker in view
     * 
     * @param distanceM Distance to marker. This value can be constantly updated via ISR
     * @param angleDeg Angle to makrer. This value can be constantly updated via ISR
     */
    void goToMarker(volatile double& distanceM, volatile double& angleDeg);

    /**
     * @brief Drives the robot in a circle starting at its current heading
     * 
     * @param circleRadiusMeters The radius of the circle to drive in meters
     * @param forwardSpeed The desired radial speed for the robot traversing the circle in m/s
     */
    void driveInCircleM(double circleRadiusMeters, double forwardSpeed);

    /**
     * @brief Drives the robot in a circle starting at its current heading
     * 
     * @param circleRadiusMeters The radius of the circle to drive in feet
     * @param forwardSpeed The desired radial speed for the robot traversing the circle in f/s
     */
    void driveInCircleF(double circleRadiusFeet, double forwardSpeed);

    /**
     * @brief Stops the motors from spinning
     * 
     */
    void stop();

    /**
     * @brief Get the Tracker instance
     * 
     * @return Tracker* 
     */
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

    // delay between rotations when scanning in place
    const int scanDelay = 1100;
    

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
    // tuning
    phiVelCtrl->SetTunings(2.5, 0, 0);
    phiPosCtrl->SetTunings(45, 0, 0.80);
    rhoVelCtrl->SetTunings(25, 0, 0);
    rhoPosCtrl->SetTunings(35, 0, 0);

    maxPhiVel = 1*pi;
    maxRhoVel = 1;

    phiVelCtrl->SetOutputLimits(-MAX_VOLTAGE, MAX_VOLTAGE);
    phiPosCtrl->SetOutputLimits(-maxPhiVel, maxPhiVel);
    rhoVelCtrl->SetOutputLimits(-MAX_VOLTAGE, MAX_VOLTAGE);
    rhoPosCtrl->SetOutputLimits(-maxRhoVel, maxRhoVel);

    double delta = radians(5);

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
    //Serial << "Entering turn in place" << endl;
    // loop
    int settledTimeCounter = 0;
    while ((abs(error) > delta || abs(phiVelAct) > 0.05 || abs(rhoVelAct) > 0.05) && (settledTimeCounter < 1000)) {

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

        delay(1);

        if (abs(error) < delta) {
            settledTimeCounter++;
        }
        else {
            settledTimeCounter = 0;
        }
    }
    //Serial << "Turned in place" << endl;
    // turn off control systems
    phiVelCtrl->SetMode(0);
    phiPosCtrl->SetMode(0);
    rhoVelCtrl->SetMode(0);
    rhoPosCtrl->SetMode(0);

    // re-zero tracker
    stop();
    tracker->zero();


    // TODO could maybe return some kind of indication of the final error.
}

void Robot::turnInPlaceDeg(double desAngleDeg) {
    turnInPlace(desAngleDeg * (pi/180));
}

void Robot::scan(volatile bool& stopCondition) {
    while (!stopCondition) {
        turnInPlaceDeg(45);
        //Serial << stopCondition << endl;
        delay(scanDelay);
        //turnInPlaceDeg(-45);
        // Serial << stopCondition << endl;
    }
    tracker->zero();

    stopCondition = false;
    while(!stopCondition);
}

void Robot::scanContinuous(volatile bool& stopCondition) {
    // tunings
    phiVelCtrl->SetTunings(0.01, 0.1, 0);
    rhoVelCtrl->SetTunings(5, 0, 0);
    rhoPosCtrl->SetTunings(5, 0, 0);

    maxRhoVel = 1;

    phiVelCtrl->SetOutputLimits(-MAX_VOLTAGE, MAX_VOLTAGE);
    rhoVelCtrl->SetOutputLimits(-MAX_VOLTAGE, MAX_VOLTAGE);
    rhoPosCtrl->SetOutputLimits(-maxRhoVel, maxRhoVel);

    // turn on control systems
    phiVelCtrl->SetMode(AUTOMATIC);
    phiPosCtrl->SetMode(AUTOMATIC);
    rhoVelCtrl->SetMode(AUTOMATIC);
    rhoPosCtrl->SetMode(AUTOMATIC);

    // init
    phiVelDes = radians(10);
    phiVelAct = tracker->getPhiSpeedRpS();
    rhoPosDes = 0;
    rhoVelDes = 0;
    rhoVelAct = tracker->getRhoSpeedMpS();
    rhoPosAct = tracker->getRhoPosM();
    
    while (!stopCondition) {
        //update values
        tracker->update();
        phiVelAct = tracker->getPhiSpeedRpS();
        rhoVelAct = tracker->getRhoSpeedMpS();
        rhoPosAct = tracker->getRhoPosM();
        // compute output
        phiVelCtrl->Compute();
        rhoPosCtrl->Compute();
        rhoVelCtrl->Compute();
        // update voltages
        voltages.setVoltages(Vforward, Vrot);
       // drive motor
        motorDriver->setM1Speed(-volts2speed(voltages.getVright()));
        motorDriver->setM2Speed(-volts2speed(voltages.getVleft()));
        // motorDriver->setM1Speed(-70);
        // motorDriver->setM2Speed(80);
    }

    stop();

    // turn off control systems
    phiVelCtrl->SetMode(0);
    rhoVelCtrl->SetMode(0);
    rhoPosCtrl->SetMode(0);

    // re-zero tracker
    tracker->zero();

}

void Robot::goForwardM(double desDistanceMeters) {
    // tunings
    rhoVelCtrl->SetTunings(10, 0, 0);
    rhoPosCtrl->SetTunings(25, 0, 0);
    phiVelCtrl->SetTunings(2.5, 0, 0);
    phiPosCtrl->SetTunings(40, 12, 0);

    double maxRhoVel = 1.5;
    double maxPhiVel = pi/2;  
    double maxPhiAngle = 100 * DEG_TO_RAD;

    rhoVelCtrl->SetOutputLimits(-MAX_VOLTAGE, MAX_VOLTAGE);
    rhoPosCtrl->SetOutputLimits(-maxRhoVel, maxRhoVel);
    phiVelCtrl->SetOutputLimits(-MAX_VOLTAGE, MAX_VOLTAGE);
    phiPosCtrl->SetOutputLimits(-maxPhiVel, maxPhiVel);

    double delta = 0.05;  // 1 mm

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

    // re-zero tracker
    tracker->zero();

}

void Robot::goForwardF(double desDistanceFeet) {
    goForwardM(desDistanceFeet / FEET_PER_MEETER);
}

void Robot::goToMarker(volatile double& distanceM, volatile double& angleDeg) {

    // turn to face marker
    turnInPlaceDeg(angleDeg);

    // set up controls

    // tunings
    rhoVelCtrl->SetTunings(10, 0, 0);
    rhoPosCtrl->SetTunings(25, 0, 0);
    phiVelCtrl->SetTunings(2.5, 0, 0);
    phiPosCtrl->SetTunings(40, 12, 0);

    double maxRhoVel = 1.5;
    double maxPhiVel = pi/2;  
    double maxPhiAngle = 100 * DEG_TO_RAD;

    rhoVelCtrl->SetOutputLimits(-MAX_VOLTAGE, MAX_VOLTAGE);
    rhoPosCtrl->SetOutputLimits(-maxRhoVel, maxRhoVel);
    phiVelCtrl->SetOutputLimits(-MAX_VOLTAGE, MAX_VOLTAGE);
    phiPosCtrl->SetOutputLimits(-maxPhiVel, maxPhiVel);

    double delta = 0.02;

    // init
    phiPosDes = angleDeg;
    phiVelDes = 0;
    phiVelAct = tracker->getPhiSpeedRpS();
    phiPosAct = tracker->getPhiPosRad();
    rhoPosDes = distanceM;
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

    // re-zero tracker
    tracker->zero();
}

void Robot::driveInCircleM(double circleRadiusMeters, double forwardSpeed) {

     // tunings
    phiVelCtrl->SetTunings(5, 9, 0);
    rhoVelCtrl->SetTunings(35, 15, 0);

    double deltaPhiPos = DEG_TO_RAD*10;   
    double deltaRhoPos = 0.001;  // 1 mm

    phiVelCtrl->SetOutputLimits(-MAX_VOLTAGE, MAX_VOLTAGE);
    rhoVelCtrl->SetOutputLimits(-MAX_VOLTAGE, MAX_VOLTAGE);
    
    // turn on control systems
    phiVelCtrl->SetMode(AUTOMATIC);
    rhoVelCtrl->SetMode(AUTOMATIC);

    // init
    phiVelDes = forwardSpeed / circleRadiusMeters;
    phiVelAct = tracker->getPhiSpeedRpS();
    phiPosAct = tracker->getPhiPosRad();
    rhoVelDes = forwardSpeed;
    rhoVelAct = tracker->getRhoSpeedMpS();
    rhoPosAct = tracker->getRhoPosM();
    
    // start the robot
    while (abs(phiPosAct) < 2*pi + deltaPhiPos) {
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
    
     // turn off control systems
    rhoVelCtrl->SetMode(0);
    phiVelCtrl->SetMode(0);

     // stop the robot where it is
    tracker->zero();
    stop();

}

void Robot::driveInCircleF(double circleRadiusFeet, double forwardSpeed) {
    driveInCircleM(circleRadiusFeet / FEET_PER_MEETER, forwardSpeed / FEET_PER_MEETER);
}

void Robot::scanInCircle(volatile bool& stopCondition) {
    double circleRadiusMeters = 0.36576;  // 1.2 ft
    double angularSpeed = radians(38); // was 45

     // tunings
    phiVelCtrl->SetTunings(3, 10, 0);
    rhoVelCtrl->SetTunings(35, 15, 0);

    double deltaPhiPos = DEG_TO_RAD*10;   
    double deltaRhoPos = 0.001;  // 1 mm

    phiVelCtrl->SetOutputLimits(-MAX_VOLTAGE, MAX_VOLTAGE);
    rhoVelCtrl->SetOutputLimits(-MAX_VOLTAGE, MAX_VOLTAGE);
    
    // turn on control systems
    phiVelCtrl->SetMode(AUTOMATIC);
    rhoVelCtrl->SetMode(AUTOMATIC);

    // init
    phiVelDes = angularSpeed;
    phiVelAct = tracker->getPhiSpeedRpS();
    phiPosAct = tracker->getPhiPosRad();
    rhoVelDes = angularSpeed * circleRadiusMeters;
    rhoVelAct = tracker->getRhoSpeedMpS();
    rhoPosAct = tracker->getRhoPosM();
    
    // start the robot
    // while (abs(phiPosAct) < 2*pi + deltaPhiPos) {
    while (!stopCondition && abs(phiPosAct) < 2.35619449) {  // 135 deg
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
    
    // if (!stopCondition) {
    //     stop();
    //     delay(100);
    //     // init
    //     phiVelDes = -angularSpeed;
    //     phiVelAct = tracker->getPhiSpeedRpS();
    //     phiPosAct = tracker->getPhiPosRad();
    //     rhoVelDes = -angularSpeed * circleRadiusMeters;
    //     rhoVelAct = tracker->getRhoSpeedMpS();
    //     rhoPosAct = tracker->getRhoPosM();
    //     while (!stopCondition && abs(phiPosAct) > 0.785398) {  // 45 deg
    //         // update values
    //         tracker->update();
    //         rhoVelAct = tracker->getRhoSpeedMpS();
    //         rhoPosAct = tracker->getRhoPosM();
    //         phiVelAct = tracker->getPhiSpeedRpS();
    //         phiPosAct = tracker->getPhiPosRad();
    //         // compute controller outputs
    //         rhoVelCtrl->Compute();
    //         phiVelCtrl->Compute();
    //         // update voltages
    //         voltages.setVoltages(Vforward, Vrot);
    //         // drive motors
    //         motorDriver->setM1Speed(-volts2speed(voltages.getVright()));
    //         motorDriver->setM2Speed(-volts2speed(voltages.getVleft()));
    //     }
    // }
     // turn off control systems
    rhoVelCtrl->SetMode(0);
    phiVelCtrl->SetMode(0);

     // stop the robot where it is
    tracker->zero();
    stop();
    

    // stop robot and wait for stable distance targets
    delay(300);
    stopCondition = false;
    while (!stopCondition);
}

void Robot::findClosestMarker(volatile bool& markerSeen, volatile double& distanceToMarker, volatile double& angleToMarker) {

    // setup
    const int iterationAngle = 30;
    const int numIterations = 360 / iterationAngle;
    double smallestDistance = 100;
    double distanceDelata_M = 0.1;

    // spin in a circle, finding the smallest distance
    for (int i = 0; i < numIterations; i++) {
        turnInPlaceDeg(iterationAngle);
        markerSeen = false;
        delay(scanDelay);
        if (markerSeen && distanceToMarker < smallestDistance) {
            smallestDistance = distanceToMarker;
        }
    }
    Serial << "Smallest Distance: " << smallestDistance << endl;

    for (int i = 0; i < numIterations; i++) {
        turnInPlaceDeg(iterationAngle);
        markerSeen = false;
        delay(scanDelay);
        if (markerSeen && abs(distanceToMarker - smallestDistance) < distanceDelata_M) {
            Serial << "Smallest Distance Found " << distanceToMarker << " " << angleToMarker << endl;
            turnInPlaceDeg(angleToMarker);
            goForwardM(distanceToMarker - 0.2032); // stop 8 inches away from marker
            break;
        }
    }

}

void Robot::findMarker4ft(volatile bool& markerSeen, volatile double& distanceToMarker, volatile double& angleToMarker) {

    // setup
    const int iterationAngle = 30;
    const int numIterations = 360 / iterationAngle;
    double distanceDelata_M = 0.1;
    double distanceToFirstMarker_M = 1.2192; // first marker should be 4 ft away

    for (int i = 0; i < numIterations; i++) {
        turnInPlaceDeg(iterationAngle);
        markerSeen = false;
        delay(scanDelay+200);
        double angleToCurrentMarker = angleToMarker;
        double distanceToCurrentMarker = distanceToMarker;
        if (markerSeen && abs(distanceToCurrentMarker - distanceToFirstMarker_M) < distanceDelata_M) {
            
            turnInPlaceDeg(angleToCurrentMarker);
            goForwardM(distanceToMarker - 0.2032); // stop 8 inches away from marker
            return;
        }
    }

    // if the 4 ft marker is not found after 1 rotation, find and go to closest marker
    findClosestMarker(markerSeen, distanceToMarker, angleToMarker);

}

void Robot::stop() {
    motorDriver->setSpeeds(0,0);
}

Tracker* Robot::getTracker() {
    return tracker;
}