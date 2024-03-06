/**
 * @file id_wheel_tf.ino
 * @author Blake Billharz
 * @brief Figure out the lowest voltage that will make the wheels move.
 * @version 0.1
 * @date 2024-03-06
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <Arduino.h>
#include <Streaming.h>
#include <DualMC33926MotorShield.h>
#include <Encoder.h>
#include "utils/Tracker.h"

// keeps track of position
Encoder rightEnc(ENCR_A, ENCR_B);
Encoder leftEnc(ENCL_A, ENCL_B);
Tracker tracker(&rightEnc, &leftEnc);

// motor driver
DualMC33926MotorShield motorDriver;

// timing
long delayTimeMs = 100;
long lastReadTime;

void setup() {
    Serial.begin(115200);
    motorDriver.init();
    lastReadTime = millis();
}

double voltage = 0;

void loop() {
    // increment voltage on the wheels until the robot starts to move

    motorDriver.setSpeeds(volts2speed(voltage), volts2speed(voltage));
    tracker.update();

    if (millis() - lastReadTime > delayTimeMs) {
        if (tracker.getLeftSpeedRpS() > 0 && tracker.getRightSpeedRpS() > 0) {
            Serial.print("Bot started to move at ");
            Serial.print(voltage, 4);
            Serial.println(" volts");
            while(1);
        }
        else {
            voltage += 0.0001;
        }
        lastReadTime = millis();
    }

    Serial.print(voltage, 4);
    Serial << " | " << tracker.getLeftSpeedRpS() << " | " << tracker.getRightSpeedRpS() << endl;

    delay(5);

    

}