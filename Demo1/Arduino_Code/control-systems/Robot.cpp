#include "Robot.h"

Robot::Robot() {
    // initialization of aggregate classes
    rightEnc = &Encoder(ENCR_A, ENCR_B);
    leftEnc = &Encoder(ENCL_A, ENCL_B);
    tracker = &Tracker(rightEnc, leftEnc);

    // control systems //
    phiVelCtrl = &PID(&phiVelAct, &Vrot, &phiVelDes, phiVelKp, phiVelKi, phiVelKd, DIRECT);
    phiPosCtrl = &PID(&phiPosAct, &phiVelDes, &phiPosDes, phiPosKp, phiPosKi, phiPosKd, DIRECT);
    xPosCtrl = &PID(&xPosAct, &phiPosDes, &xPosDes, xPosKd, xPosKi, xPosKd, DIRECT);
    rhoVelCtrl = &PID(&rhoVelAct, &Vforward, &rhoVelDes, rhoVelKp, rhoVelKi, rhoVelKd, DIRECT);
    rhoPosCtrl = &PID(&rhoPosAct, &rhoVelDes, &rhoPosDes, rhoPosKp, rhoPosKi, rhoPosKd, DIRECT);

    // all control systems default to off
    phiVelCtrl->SetMode(0);
    phiPosCtrl->SetMode(0);
    xPosCtrl->SetMode(0);
    rhoVelCtrl->SetMode(0);
    rhoPosCtrl->SetMode(0);
}

void Robot::turnInPlaceRad(double desAngleRad) {
    //TODO
}

void Robot::turnInPlaceDeg(double desAngleDeg) {
    turnInPlaceRad(desAngleDeg * (pi/180));
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