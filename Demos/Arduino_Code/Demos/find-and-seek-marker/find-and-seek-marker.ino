/**
 * @file find-and-seek-marker.ino
 * @author Blake Billharz and Ben Sprik
 * @brief Scan until a marker has been detected. On detection, process telemetry, go to marker, and do a circle around it.
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
double comfortableDistanceFromMarkerF = 0.75;


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
    
    // setup
    Serial.begin(115200);

    // setup i2c communication channel
    Wire.begin(I2CADDR);
    while (Wire.available()) {
        Wire.read(); // clear out any garbage
    }
    Wire.onReceive(recieveTargetISR);


    // main routine

    Robot rob;

    Serial << "Scanning for marker" << endl;
    rob.scanInCircle(markerFound);
    Serial << "Marker found" << endl;

    // wait for another data sample
    // int detectTime = millis();
    // while(!markerFound && (millis() - detectTime) < 3000);
    // if (!markerFound) {
    //     rob.turnInPlaceDeg(45);
    //     while(!markerFound);
    // }

    
    //recieveI2C = false;

    //Serial << "Angle: " << angleToMarker << endl;
    //Serial << "Distance: " << distanceToMarker << endl;

    //Serial << "Turning to face marker" << endl;
    rob.turnInPlaceDeg(angleToMarker);
    
    //Serial << "Heading to marker" << endl;
    
    double targetDistanceF = meters2feet(distanceToMarker) - comfortableDistanceFromMarkerF;    
    rob.goForwardF(targetDistanceF);
    //Serial << "Made it to marker" << endl;

    //Serial << "Doing a circle" << endl;
    rob.turnInPlaceDeg(-90);
    delay(2);
    rob.driveInCircleF(1.25, 1.75);

    Serial << "Done" << endl;

}

void loop() {
    //
}
