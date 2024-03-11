#include "Robot.hpp"

Robot::Robot() {
    // initialization of aggregate classes
    rightEnc = &Encoder(ENCR_A, ENCR_B);
    leftEnc = &Encoder(ENCL_A, ENCL_B);
    tracker = &Tracker(rightEnc, leftEnc);
    motorDriver.init();

    // control systems //
    phiVelCtrl = &PID(&phiVelAct, &Vrot, &phiVelDes, phiVelKp, phiVelKi, phiVelKd, DIRECT);
    phiPosCtrl = &PID(&phiPosAct, &phiVelDes, &phiPosDes, phiPosKp, phiPosKi, phiPosKd, DIRECT);
    yPosCtrl = &PID(&yPosAct, &phiPosDes, &yPosDes, yPosKd, yPosKi, yPosKd, DIRECT);
    rhoVelCtrl = &PID(&rhoVelAct, &Vforward, &rhoVelDes, rhoVelKp, rhoVelKi, rhoVelKd, DIRECT);
    rhoPosCtrl = &PID(&rhoPosAct, &rhoVelDes, &rhoPosDes, rhoPosKp, rhoPosKi, rhoPosKd, DIRECT);
    xPosCtrl = &PID(&xPosAct, &rhoVelDes, &xPosDes, xPosKp, xPosKi, xPosKd, DIRECT);

    // all control systems default to off
    phiVelCtrl->SetMode(0);
    phiPosCtrl->SetMode(0);
    yPosCtrl->SetMode(0);
    rhoVelCtrl->SetMode(0);
    rhoPosCtrl->SetMode(0);
    xPosCtrl->SetMode(0);
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

    phiVelCtrl->SetSampleTime(controllerSampleTimeMs);
    phiPosCtrl->SetSampleTime(controllerSampleTimeMs);
    rhoVelCtrl->SetSampleTime(controllerSampleTimeMs);
    rhoPosCtrl->SetSampleTime(controllerSampleTimeMs);

    double delta = 1 * (pi/180);
    
    // turn on control systems
    phiVelCtrl->SetMode(AUTOMATIC);
    phiPosCtrl->SetMode(AUTOMATIC);
    rhoVelCtrl->SetMode(AUTOMATIC);
    rhoPosCtrl->SetMode(AUTOMATIC);

    // init
    tracker->update();
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
        // compute output
        phiPosCtrl->Compute();
        phiVelCtrl->Compute();
        rhoPosCtrl->Compute();
        rhoVelCtrl->Compute();
        // update voltages
        voltages.setVoltages(Vforward, Vrot);
        // drive motor
        motorDriver.setM1Speed(-volts2speed(voltages.getVright()));
        motorDriver.setM2Speed(-volts2speed(voltages.getVleft()));
    }

    // TODO could maybe return some kind of indication of the final error.
}

void Robot::turnInPlaceDeg(double desAngleDeg) {
    turnInPlace(desAngleDeg * (pi/180));
}

void Robot::goForwardM(double desDistanceMeters) {
    //TODO
}

void Robot::goForwardF(double desDistanceFeet) {
    goForwardM(desDistanceFeet / FEET_PER_MEETER);
}

Tracker* Robot::getTracker() {
    return tracker;
}