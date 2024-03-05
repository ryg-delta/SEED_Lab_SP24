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

Tracker tracker(&rightEnc, &leftEnc);
Tracker filteredTracker(&rightEnc, &leftEnc);

long sampleTimeMs = 100;
long lastReadMs = 0;
double timeS = 0;


void setup() {
  tracker.filterInputs(false);
  filteredTracker.filterInputs(true);

  Serial.begin(115200);
  while(!Serial);
  Serial.println("Time  rho_dot  phi_dot  rho  phi  |  rho_dot_filtered  phi_dot_filtered  rho_filtered  phi_filtered");
}

void loop() {

  if (millis() - lastReadMs >= sampleTimeMs) {
    lastReadMs = millis();
    Serial << timeS << " ";
    Serial << tracker.getRhoSpeedMpS()*1'000'000 << " " << tracker.getPhiSpeedRpS()*1000 << " " << tracker.getRhoPosM() << " " << tracker.getPhiPosRad() << " ";
    Serial << "| ";
    Serial << filteredTracker.getRhoSpeedMpS()*1'000'000 << " " << filteredTracker.getPhiSpeedRpS()*1000 << " " << filteredTracker.getRhoPosM() << " " << filteredTracker.getPhiPosRad() << " ";
    Serial << endl;
    timeS += sampleTimeMs/1000.0;
  }

  tracker.update();
  filteredTracker.update();
}
