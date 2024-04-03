#include <Arduino.h>
#include "utils/Robot.hpp"
#include <Streaming.h>

#define TIMER_INTERRUPT_DEBUG         2
#define _TIMERINTERRUPT_LOGLEVEL_     0
#define USE_TIMER_2     true
#include "TimerInterrupt.h"


Robot rob;
volatile bool markerFound;
volatile double distanceToMarker;
volatile double angleToMarker;
double comfortableDistanceFromMarkerF = 0.75;

void isr() {
    markerFound = true;
    distanceToMarker = 1;
    angleToMarker = 20;
}

void setup() {
    
    Serial.begin(115200);

    ITimer2.init();
    if (ITimer2.attachInterruptInterval(5000, isr, 0))
    {
        Serial.print(F("Starting  ITimer1 OK, millis() = ")); Serial.println(millis());
    }
    else {
        Serial.println(F("Can't set ITimer1. Select another freq. or timer"));
    }

    rob.turnInPlaceDeg(90);
    rob.turnInPlaceDeg(20);

    Serial << "Scanning for marker" << endl;
    rob.scan(markerFound);
    Serial << "Marker found" << endl;

    // wait for another data sample
    markerFound = false;
    while(!markerFound);

    Serial << "Turning to face marker" << endl;
    rob.turnInPlaceDeg(angleToMarker);

    Serial << "Heading to marker" << endl;
    double targetDistanceF = meters2feet(distanceToMarker) - comfortableDistanceFromMarkerF;
    rob.goForwardF(targetDistanceF);
    Serial << "Made it to marker" << endl;

    Serial << "Doing a circle" << endl;
    rob.turnInPlaceDeg(-90);
    delay(2);
    rob.driveInCircleF(comfortableDistanceFromMarkerF, 1.75);

    Serial << "Done" << endl;

}

void loop() {
    //
}
