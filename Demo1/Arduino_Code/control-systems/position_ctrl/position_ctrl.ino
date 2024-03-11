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

// I named him rob

// test
double startTimeS;
double currTimeS;

// printing
double printIntervalMs = 10;
double lastPrintTimeMs = 0;

void setup() {

    Serial.begin(115200);
    while(!Serial);

    Serial << "Beginning" << endl;

    Robot rob;

    rob.turnInPlaceDeg(90);
    delay(1000);
    rob.turnInPlaceDeg(0);
    delay(1000);
    rob.turnInPlaceDeg(540);

    Serial << "Finished" << endl;

}

void loop() {
    delay(1000);
}