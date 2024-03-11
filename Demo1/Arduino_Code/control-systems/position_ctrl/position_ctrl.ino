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
Encoder* rightEnc = new Encoder(ENCR_A, ENCR_B);
Encoder* leftEnc = new Encoder(ENCL_A, ENCL_B);
Tracker* tracker = new Tracker(rightEnc, leftEnc);
DualMC33926MotorShield* motorDriver = new DualMC33926MotorShield();

// test
double startTimeS;
double currTimeS;

// printing
double printIntervalMs = 10;
double lastPrintTimeMs = 0;

void setup() {

    Serial.begin(115200);
    while(!Serial);

    // while (1) {
    //     Serial << "Right: " << rightEnc->read() << " | ";
    //     Serial << "Left: " << leftEnc->read() << endl;
    // }

    motorDriver->init();

    if (motorDriver->getFault()) {
        Serial << "Motor Driver fault. Exiting." << endl;
        while(1);
    }

    Serial << "Beginning" << endl;

    Robot rob(rightEnc, leftEnc, tracker, motorDriver);

    // Serial << "turn left" << endl;
    // rob.turnInPlaceDeg(90);
    // delay(3000);
    // Serial << "turn back" << endl;
    // rob.turnInPlaceDeg(0);
    // delay(3000);
    Serial << "go forward" << endl;
    rob.goForwardF(10);

    Serial << "Finished" << endl;

    delete leftEnc, rightEnc, tracker, motorDriver;

}

void loop() {
    delay(1000);
}