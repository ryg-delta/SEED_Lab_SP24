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
#define FOV 60

volatile bool markerFound = false;
volatile double distanceToMarker;
volatile double angleToMarker;
double distanceToTarget;
double angleToTarget;
double adjustmentAngle;
double comfortableDistanceFromMarker;
const int NUM_MARKERS = 6;

// test
double startTimeS;
double currTimeS;

// printing
double printIntervalMs = 10;
double lastPrintTimeMs = 0;

// process the data from the pi
void recieveTargetISR(int howMany) {


    // read angle and distance data
    uint8_t angleConverted = Wire.read();
    uint8_t distanceCM = Wire.read();
    
    // convert to usable values for robot
    angleToMarker = FOV/2 - FOV*angleConverted/255 - 3;
    distanceToMarker = distanceCM / 100.0;
    //distanceToMarker = distanceToMarker - comfortableDistanceFromMarkerF;

    // the marker has been spotted
    if (distanceToMarker > 0.1) {
        markerFound = true;
    }
    //}
}

void setup() {

    Serial.begin(115200);
    while(!Serial);

    // setup i2c communication channel
    Wire.begin(I2CADDR);
    while (Wire.available()) {
        Wire.read(); // clear out any garbage
    }
    Wire.onReceive(recieveTargetISR);

    // I named him rob
    Robot rob;

    Serial << "Beginning" << endl;

    // Serial << "Turning in circle" << endl;
    // rob.driveInCircleM(0.25, 0.15);
    // Serial << "Done" << endl;

    delay(2000);

    // Serial << "Scanning turn in cirlce" << endl;    
    // rob.scanInCircle(markerFound);
    // Serial << "Done" << endl;
    // delay(1000);



    // Serial << "Go in a circle" << endl;
    // rob.driveInCircleF(1.1, 1.5);

    Serial << "Finished" << endl;

}

void loop() {
    if (markerFound) {
        Serial << "Distance to marker: " << meters2feet(distanceToMarker) << endl;
        markerFound = false;
    }
    delay(1000);
}