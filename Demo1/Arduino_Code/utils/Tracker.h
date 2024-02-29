/**
 * @file Tracker.h
 * @author Blake Billharz and Benjamin Sprik
 * @brief Given wheel encoder instances, keeps track of position, translational velocity, and rotational velocity.
 * @example // todo
 * @version 0.1
 * @date 2024-02-28
 * 
 * 
 */

#ifndef LOCALIZER_H
#define LOCALIZER_H

#include <Arduino.h>
#include <Encoder.h>

#define VELOCITY_READ_TIME_MS 10

class Tracker {
    public:

    /**
     * @brief Encoders must be pass-by-pointer. To instantiate, do Tracker(&encLeft, &encRight);
     * 
     * @param rightEncoder address of the right encoder
     * @param leftEncoder address of the left encoder
     */
    Tracker(Encoder* rightEncoder, Encoder* leftEncoder) {
        encRight = rightEncoder;
        encLeft = leftEncoder;
        zero();
    }

    /**
     * @brief Zero's out all data
     * 
     */
    void zero() {
        rightPosRad = 0;
        leftPosRad = 0;
        rightPosRadLastRead = 0;
        leftPosRadLastRead = 0;
        rhoPosM = 0;
        phiPosRad = 0;
        rightSpeedRpS = 0;
        leftSpeedRpS = 0;
        rhoSpeedMpS = 0;
        phiSpeedRpS = 0;
        lastVelocityReadTime = millis();
    }


    /**
     * @brief Update the data using the encoders. This method should be called once every loop, or at least once every 10ms.
     * 
     */
    void update() {
        // read encoders and calculate positions in radians

        // update rho and phi

        // update speeds if enough time has passed
    }


    // getters

    long getRightEncCnt() {
        return rightEncCnt;
    }

    long getLeftEncCnt() {
        return leftEncCnt;
    }

    double getRightPosRad() {
        return rightPosRad;
    }

    double getLeftPosRad() {
        return leftPosRad;
    }

    double getRhoPosM() {
        return rhoPosM;
    }

    double getPhiPosRad() {
        return phiPosRad;
    }

    double getXPosM() {
        //TODO
        return 0;
    }

    double getYPosM() {
        //TODO
        return 0;
    }

    double getRightSpeedRpS() {
        return rightSpeedRpS;
    }

    double getLeftSpeedRpS() {
        return leftSpeedRpS;
    }

    double getRhoSpeedMpS() {
        return rhoSpeedMpS;
    }

    double getPhiSpeedMpS() {
        return phiSpeedRpS;
    }

    double getXSpeedMpS() {
        //TODO
        return 0;
    }

    double getYSpeedMpS() {
        //TODO
        return 0;
    }


    private:

    Encoder* encRight;
    Encoder* encLeft;
    
    // position variables
    long rightEncCnt;
    long leftEncCnt;
    double rightPosRad;
    double leftPosRad;
    double rightPosRadLastRead; // these ones are for calculating velocity
    double leftPosRadLastRead;
    double rhoPosM;
    double phiPosRad;


    // speed variables
    double rightSpeedRpS;
    double leftSpeedRpS;
    double rhoSpeedMpS;
    double phiSpeedRpS;
    double lastVelocityReadTime;


};

#endif