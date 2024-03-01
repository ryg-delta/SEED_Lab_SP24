/**
 * @file Tracker.h
 * @author Blake Billharz and Benjamin Sprik
 * @brief Given wheel encoder instances, keeps track of position, translational velocity, and rotational velocity.
 * @example
 * Encoder leftEnc(pin1, pin2);
 * Encoder rightEnc(pin3, pin4);
 * Tracker tracker(&rightEnc, &leftEnc);
 * 
 * void setup() {
 *      tracker.zero();
 * }
 * 
 * void loop() {
 *      // other stuff
 * 
 *      tracker.update();
 * 
 *      double phiSpeed = tracker.getPhiSpeedMpS();
 *      double phiPos = tracker.getPhiPosRad();
 *      double forwardVelocity = tracker.getRhoSpeedMpS();
 * 
 *      // see the readme for a complete list of tracked metrics.
 * }
 * 
 * @version 0.1
 * @date 2024-02-28
 * 
 * 
 */

#ifndef LOCALIZER_H
#define LOCALIZER_H

#include <Arduino.h>
#include <Encoder.h>
#include "robotConstants.h"

#define VELOCITY_READ_INTERVAL_MS 10

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
        rightEncCntLast = 0;
        leftEncCntLast = 0;
        rightPosRadLastRead = 0;
        leftPosRadLastRead = 0;
        phiPosRad = 0;
        xPosM = 0;
        yPosM = 0;
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
        rightEncCnt = encRight->read();
        leftEncCnt = encLeft->read();

        // update speeds if enough time has passed
        long curTime = millis();
        if (curTime - lastVelocityReadTime >= VELOCITY_READ_INTERVAL_MS) {

            // setup variables
            long timeStepS = 1000 * (curTime - lastVelocityReadTime);
            rightPosRad = cnt2Rad(rightEncCnt);
            leftPosRad = cnt2Rad(leftEncCnt);

            // theta
            rightSpeedRpS = (rightPosRad - rightPosRadLastRead) / timeStepS;
            leftSpeedRpS = (leftPosRad - leftPosRadLastRead) / timeStepS;

            // rho and phi
            rhoSpeedMpS = (WHEEL_RADIUS_M / 2) * (rightSpeedRpS + leftSpeedRpS);
            phiSpeedRpS = (WHEEL_RADIUS_M / WHEEL_BASE_M) * (rightSpeedRpS - leftSpeedRpS);

            // save variables for next time
            lastVelocityReadTime = curTime;
            rightPosRadLastRead = rightPosRad;
            leftPosRadLastRead = leftPosRad;
        }


        // only update position when encoder counts change
        if (rightEncCnt != rightEncCntLast || leftEncCnt != leftEncCntLast) {

            // wheel positions
            rightPosRad = cnt2Rad(rightEncCnt);
            leftPosRad = cnt2Rad(leftEncCnt);
            double deltaRight = rightPosRad - rightPosRadLastRead;
            double deltaLeft = leftPosRad - leftPosRadLastRead;
            rightPosRadLastRead = rightPosRad;
            leftPosRadLastRead = leftPosRad;

            // x and y
            xPosM = xPosM + ((WHEEL_RADIUS_M/2) * cos(phiPosRad) * (deltaLeft + deltaRight));
            yPosM = yPosM + ((WHEEL_RADIUS_M/2) * sin(phiPosRad) * (deltaLeft + deltaRight));

            // phi
            phiPosRad = (WHEEL_RADIUS_M / WHEEL_BASE_M) * (rightPosRad - leftPosRad);

        }

    }


    /**
     * @brief Convert encoder counts to radians
     * 
     * @param count 
     * @return double 
     */
    double cnt2Rad(long count) {
        return 2 * pi * (float) count / ENC_CNT_PER_REV;
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

    double getXPosM() {
        return xPosM;
    }

    double getYPosM() {
        return yPosM;
    }

    double getPhiPosRad() {
        return phiPosRad;
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

    double getPhiSpeedRpS() {
        return phiSpeedRpS;
    }


    private:

    Encoder* encRight;
    Encoder* encLeft;
    
    // position variables
    long rightEncCnt;
    long leftEncCnt;
    long rightEncCntLast;
    long leftEncCntLast;
    double rightPosRad;
    double leftPosRad;
    double rightPosRadLastRead; // these ones are for calculating velocity
    double leftPosRadLastRead;
    double phiPosRad;
    double xPosM;
    double yPosM;

    // speed variables
    double rightSpeedRpS;
    double leftSpeedRpS;
    double rhoSpeedMpS;
    double phiSpeedRpS;
    double lastVelocityReadTime;


};

#endif