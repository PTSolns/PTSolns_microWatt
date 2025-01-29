// EXAMPLE - Deep Sleep
// Last update: Jan 28, 2025
//
// DESCRIPTION
// This example puts the ESP32 microWatt into deep sleep for 10 seconds, and then wakes for 20 seconds.
// This example can be used to measure deep sleep current draw, or alterned for user requried applications.

#include <PTSolns_microWatt.h>

microWatt microWatt;

void setup() {
  microWatt.setFreq(80); // reduces current consumption during active mode

  delay(20000);
  microWatt.deepSleep(10); // in seconds
}

void loop() {
  // Nothing here
}