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


// test
double startTimeS;
double currTimeS;

// printing
double printIntervalMs = 10;
double lastPrintTimeMs = 0;

void setup() {

    Serial.begin(115200);
    while(!Serial);

    // I named him rob
    Robot rob;

    Serial << "Beginning" << endl;

    Serial << "Go in a circle" << endl;
    double startTime = millis();
    rob.driveInCircleF(1, 5);
    double endTime = millis() - startTime;
    Serial << "Finished" << endl;
    Serial << "Time: " << endTime << endl;

}

void loop() {
    delay(1000);
}