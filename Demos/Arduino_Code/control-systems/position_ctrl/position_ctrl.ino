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

    Serial << "Go forward" << endl;
    rob.goForwardF(6);
    Serial << "Done" << endl;

    delay(1000);

    Serial << "turn around" << endl;
    rob.turnInPlaceDeg(180);
    Serial << "Done" << endl;

    delay(1000);

    Serial << "go back" << endl;
    rob.goForwardF(6);
    Serial << "Done" << endl;

    delay(1000);

    Serial << "Turn again" << endl;
    rob.turnInPlaceDeg(-180);

    // Serial << "Go in a circle" << endl;
    // rob.driveInCircleF(3, 2);

    Serial << "Finished" << endl;

}

void loop() {
    delay(1000);
}