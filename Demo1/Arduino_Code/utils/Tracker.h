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
 *      // see the readme for a complete list of tracked metrics.
 *      double phiSpeed = tracker.getPhiSpeedMpS();
 *      double phiPos = tracker.getPhiPosRad();
 *      double forwardVelocity = tracker.getRhoSpeedMpS();
 * 
 *      // re-zero only rho and phi
 *      tracker.zeroRho();
 *      tracker.zeroPhi();
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
#include <FIR.h>
#include "robotConstants.h"

#define VELOCITY_READ_INTERVAL_MS 20
#define FILTER_TAP_NUM 7

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
        rightFilter.setFilterCoeffs(filter_taps);
        leftFilter.setFilterCoeffs(filter_taps);
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
        rhoPosM = 0;
        xPosM = 0;
        yPosM = 0;
        rightSpeedRpS = 0;
        leftSpeedRpS = 0;
        rhoSpeedMpS = 0;
        phiSpeedRpS = 0;
        lastVelocityReadTime = millis();
        rightPosRadLastVelRead = 0;
        leftPosRadLastVelRead = 0;
    }

    /**
     * @brief Zero out rho position
     * 
     */
    void zeroRho() {
        rhoPosM = 0;
        rhoSpeedMpS = 0;
    }

    /**
     * @brief Zero out phi position
     * 
     */
    void zeroPhi() {
        phiPosRad = 0;
        phiSpeedRpS = 0;
    }

    /**
     * @brief Turns on and off FIR filter for velocity inputs. Defaults to false
     * 
     * @param filterVelocity 
     */
    void filterInputs(bool filterVelocity) {
        this->filterVelocity = filterVelocity;
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
            double timeStepS = (curTime - lastVelocityReadTime)/1000.0;
            rightPosRad = cnt2Rad(rightEncCnt);
            leftPosRad = cnt2Rad(leftEncCnt);

            // theta
            rightSpeedRpS = (rightPosRad - rightPosRadLastVelRead) / timeStepS;
            leftSpeedRpS = (leftPosRad - leftPosRadLastVelRead) / timeStepS;

            if (filterVelocity) {
                rightSpeedRpS = rightFilter.processReading(rightSpeedRpS);
                leftSpeedRpS = leftFilter.processReading(leftSpeedRpS);
            }

            
            // rho and phi
            rhoSpeedMpS = (WHEEL_RADIUS_M / 2) * (rightSpeedRpS + leftSpeedRpS);
            phiSpeedRpS = (WHEEL_RADIUS_M / WHEEL_BASE_M) * (rightSpeedRpS - leftSpeedRpS);

            // save variables for next time
            lastVelocityReadTime = curTime;
            rightPosRadLastVelRead = rightPosRad;
            leftPosRadLastVelRead = leftPosRad;
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

            // phi and rho
            phiPosRad = (WHEEL_RADIUS_M / WHEEL_BASE_M) * (rightPosRad - leftPosRad);
            rhoPosM = (WHEEL_RADIUS_M / 2) * (rightPosRad + leftPosRad);

        }

    }


    /**
     * @brief Convert encoder counts to radians
     * 
     * @param count 
     * @return double 
     */
    double cnt2Rad(long count) {
        return 2 * pi * (double) count / ENC_CNT_PER_REV;
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

    double getRhoPosM() {
        return rhoPosM;
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
    double rightPosRadLastRead;
    double leftPosRadLastRead;
    double phiPosRad;
    double rhoPosM;
    double xPosM;
    double yPosM;

    // speed variables
    double rightSpeedRpS;
    double leftSpeedRpS;
    double rhoSpeedMpS;
    double phiSpeedRpS;
    long lastVelocityReadTime;
    double rightPosRadLastVelRead;
    double leftPosRadLastVelRead;

    // velocity sample filter
    static double filter_taps[FILTER_TAP_NUM];
    bool filterVelocity = false;
    FIR<double, FILTER_TAP_NUM> rightFilter;
    FIR<double, FILTER_TAP_NUM> leftFilter;

    


};

/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 50 Hz

* 0 Hz - 8 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = 3.4208755570234484 dB

* 20 Hz - 25 Hz
  gain = 0
  desired attenuation = -50 dB
  actual attenuation = -51.67855666318294 dB

*/

double Tracker::filter_taps[FILTER_TAP_NUM] = {
  -0.01598532973353844,
  0.05648712148047267,
  0.3139374054007945,
  0.4855364950248441,
  0.3139374054007945,
  0.05648712148047267,
  -0.01598532973353844
};


#endif