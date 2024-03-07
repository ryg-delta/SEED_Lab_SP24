/**
 * @file testTracker.ino
 * @author Blake Billharz
 * @brief Run the tracker with and without the filter and print the data out to be plotted in matlab.
 * @version 0.1
 * @date 2024-03-02
 * 
 * 
 */

#include ".\utils\robotConstants.h"
#include ".\utils\Tracker.h"
#include <Encoder.h>
#include <Streaming.h>

Encoder rightEnc(ENCR_A, ENCR_B);
Encoder leftEnc(ENCL_A, ENCL_B);

Tracker unfilteredTracker(&rightEnc, &leftEnc);
Tracker tracker(&rightEnc, &leftEnc);

long sampleTimeMs = 100;
long lastReadMs = 0;
double timeS = 0;


void setup() {
  tracker.filterInputs(true);

  Serial.begin(115200);
  while(!Serial);
  Serial << "Ready" << endl;

}

void loop() {

  // uncomment what you would like to print

  if (millis() - lastReadMs >= sampleTimeMs) {
    lastReadMs = millis();

    // wheel positions
    // Serial << tracker.getRightPosRad() << " | ";
    // Serial << tracker.getLeftPosRad() << " | ";

    // // rho and phi pos
    // Serial << tracker.getRhoPosM() << " | ";
    // Serial << tracker.getPhiPosRad() << " | ";

    // // x and y pos
    // Serial << tracker.getXPosM() << " | ";
    // Serial << tracker.getYPosM() << " | ";

    // // wheel speeds
    // Serial << tracker.getRightSpeedRpS() << " | ";
    // Serial << tracker.getLeftSpeedRpS() << " | ";

    // // wheel speeds unfiltered
    // Serial << unfilteredTracker.getRightSpeedRpS() << " | ";
    // Serial << unfilteredTracker.getLeftSpeedRpS() << " | ";

    // // rho and phi speeds
    // Serial << tracker.getRhoSpeedMpS() << " | ";
    // Serial << tracker.getPhiSpeedRpS() << " | ";

    // // rho and phi speeds unfiltered
    // Serial << unfilteredTracker.getRhoSpeedMpS() << " | ";
    // Serial << unfilteredTracker.getPhiSpeedRpS() << " | ";

    Serial << endl;
    timeS += sampleTimeMs/1000.0;
  }

  tracker.update();
  unfilteredTracker.update();
}
