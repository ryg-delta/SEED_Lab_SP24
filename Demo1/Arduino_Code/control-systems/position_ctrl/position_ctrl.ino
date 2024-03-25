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


    // Serial << "turn left" << endl;
    // rob.turnInPlaceDeg(90);
    // delay(3000);
    // Serial << "turn back" << endl;
    // rob.turnInPlaceDeg(0);
    // delay(3000);
    Serial << "go forward" << endl;
    rob.goForwardF(10);

    Serial << "Finished" << endl;

}

void loop() {
    delay(1000);
}