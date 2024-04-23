/**
 * @file final-demo-base.ino 
 * @author Blake Billharz and Ben Sprik
 * @brief This code uses a simple algortithm with the previously written movement methods to travers the demo course.
 * @version 0.1
 * @date 2024-04-07
 * 
 */

#include <Arduino.h>
#include "utils/Robot.hpp"
#include <Streaming.h>
#include <Wire.h>

#define I2CADDR 8
#define FOV 60

volatile bool markerFound;
volatile double distanceToMarker;
volatile double angleToMarker;
double distanceToTarget;
double angleToTarget;
double comfortableDistanceFromMarker;
const int NUM_MARKERS = 4;

// calculates the move target based on the marker telemetry
void calcTarget();

// process the data from the pi
void recieveTargetISR(int howMany) {


    // read angle and distance data
    uint8_t angleConverted = Wire.read();
    uint8_t distanceCM = Wire.read();
    
    // convert to usable values for robot
    angleToMarker = FOV/2 - FOV*angleConverted/255 - 3;
    distanceToMarker = distanceCM / 100.0;

    // the marker has been spotted
    if (distanceToMarker > 0.254 && distanceToMarker < 3) {
        markerFound = true;
    }
    //}
}


void setup() {
    
    // setup
    Serial.begin(115200);

    // setup i2c communication channel
    Wire.begin(I2CADDR);
    while (Wire.available()) {
        Wire.read(); // clear out any garbage
    }
    Wire.onReceive(recieveTargetISR);

    comfortableDistanceFromMarker = 0.2032;


    // main routine

    Robot rob;

    // find closest marker and go to it
    rob.findClosestMarker(markerFound, distanceToMarker, angleToMarker);

    // find second marker
    rob.turnInPlaceDeg(-90);
    markerFound = false;
    delay(100);
    rob.scan(markerFound);
    calcTarget();
       
    // go to second marker
    rob.turnInPlaceDeg(angleToTarget);
    rob.goForwardM(distanceToTarget);


    // continue to seek markers until a circle is complete
    for (int i = 1; i < NUM_MARKERS - 1; i++) {

        // find next marker
        markerFound = false;
        rob.scanInCircle(markerFound);
        
        // calculate where to go
        calcTarget();

        // go to next marker
        rob.turnInPlaceDeg(angleToTarget);
        rob.goForwardM(distanceToTarget);

    }

}

void loop() {
    //
}


void calcTarget() {
    angleToTarget = degrees(-atan2(comfortableDistanceFromMarker, distanceToMarker)) + angleToMarker;
    distanceToTarget = sqrt( sq(comfortableDistanceFromMarker) + sq(distanceToMarker) );
}