#include <Wire.h>
#include <Streaming.h>

#define I2CADDR 8
#define FOV 60

volatile bool targetAquired = false;

double desiredAngleDeg;
double desiredDistanceM;

void recieveTargetISR(int howMany) {
    // read offset (register address)
    Wire.read(); 

    // read angle and distance data
    uint8_t angleHigh = Wire.read();
    uint8_t angleLow = Wire.read();
    uint8_t distanceHigh = Wire.read();
    uint8_t distanceLow = Wire.read();

    // piece values together
    double angleConverted = (angleHigh << 8) | (angleLow & 0xFF);
    double distanceCM = (distanceHigh << 8) | (distanceLow & 0xFF);
    
    // convert to usable values for robot
    desiredAngleDeg = FOV/2 - FOV*angleConverted/255;
    desiredDistanceM = distanceCM / 100;

    // the marker has been spotted
    targetAquired = true;
}

void setup() {
    // set up isr to recieve target
    Wire.begin(I2CADDR);
    while (Wire.available()) {
        Wire.read(); // clear out any garbage
    }
    Wire.onReceive(recieveTargetISR);

    Serial.begin(115200);
    while(!Serial);
}

void loop() {
    // print marker location when found
    if (targetAquired) {
        Serial.print("Desired Angle: ");
        Serial.println(desiredAngleDeg, 4);
        Serial.print("Desired Distance: ");
        Serial.println(desiredDistanceM, 4);
        targetAquired = false;
    }
}




// //Take angle input from i2c
// FOV = 60;
// actualAngle = FOV/2 - FOV*angle/255;
// //Take cm distance angle from i2c
// distance_in_m = distance*100;

