#include "Robot.h"

Robot::Robot() {
    // initialization of aggregate classes
    rightEnc = &Encoder(ENCR_A, ENCR_B);
    leftEnc = &Encoder(ENCL_A, ENCL_B);
    tracker = &Tracker(rightEnc, leftEnc);

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
    //TODO

    // tunings
    phiVelCtrl->SetTunings(2.5, 0, 0);
    phiPosCtrl->SetTunings(25, 12, 0);
    rhoVelCtrl->SetTunings(10, 0, 0);
    rhoPosCtrl->SetTunings(14.24, 31.56, 0);
    
    maxPhiVel = pi/2;
    maxRhoVel = 0.35;
    
    // turn on control systems


    // loop
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