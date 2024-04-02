#include <Wire.h>
#include <Streaming.h>

#define I2CADDR 8
#define FOV 60

volatile bool targetAquired = false;

double desiredAngle;
double desiredDistanceM;

void recieveTargetISR(int howMany) {
    // Wire.read(); // read offset??

    uint8_t angle = Wire.read();
    uint8_t distance = Wire.read();

    desiredAngle = FOV/2 - FOV*angle/255;
    desiredDistanceM = distance * 100;

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
    if (targetAquired) {
        Serial << "Desired Angle: " << desiredAngle << endl;
        Serial << "Desired Distance: " << desiredDistanceM << endl;
        targetAquired = false;
    }
}




// //Take angle input from i2c
// FOV = 60;
// actualAngle = FOV/2 - FOV*angle/255;
// //Take cm distance angle from i2c
// distance_in_m = distance*100;

