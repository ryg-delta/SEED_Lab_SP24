/**
 * @file phi_position_ctrl.ino
 * @author Blake Billharz, Ben Sprik
 * @brief Implement the control system modeled in simulink to control the angular position of the robot.
 * @version 0.1
 * @date 2024-03-02
 * 
 */

#include <Arduino.h>
#include "utils/Robot.hpp"
#include <Streaming.h>
#include <Wire.h>

#define I2CADDR 8

// test
double startTimeS;
double currTimeS;

// printing
double printIntervalMs = 10;
double lastPrintTimeMs = 0;

void setup() {

    Serial.begin(115200);
    while(!Serial);

    Wire.begin(I2CADDR);

    // I named him rob
    Robot rob;

    Serial << "Beginning" << endl;

    Serial << "Scanning turn in cirlce" << endl;
    volatile bool test = false;
    rob.scanInCircle(test);
    Serial << "Done" << endl;

    delay(1000);



    // Serial << "Go in a circle" << endl;
    // rob.driveInCircleF(1.1, 1.5);

    Serial << "Finished" << endl;

}

void loop() {
    delay(1000);
}