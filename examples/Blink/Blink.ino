// EXAMPLE: Blink
// Last update: Oct 25, 2023
// contact@PTSolns.com
//
// DESCRIPTION
// This example demonstrates the classic Blink example. 
// We will use the onboard LED on pin 13 for this example. 
//
// In the setup() loop the call microWatt.begin() is made. Among other things, this sets a start-up blink pattern (four fast onboard flashes.)
// Every time the ESP32 microWatt is restarted this initial blink is called.
// Then we wait 3 sec to go into the loop() loop. Not required, only to show and separate the blink that is called in loop().
// In loop() the call microWatt.blink(13, 1, 100) is made. The first argument is the pin to the LED. Since the onboard LED is used, this should remain 13.
// As an exercise, the user can add an external LED + resistor circuit to another pin and change pin 13 accordingly.
// The second argument is the number of blink ON-OFF cycles. It is set to 1 as the loop() is repeated indefinitely anyway. But the number of blink ON-OFF cycles can be set to any number.
// The last argument is the equal ON and OFF durations measured in milliseconds.
//
// HARDWARE CONFIGURATION
// Connect the ESP32 microWatt via USB to your laptop. No other hardware is required.

#include <PTSolns_microWatt.h>

microWatt microWatt;

void setup() {
  microWatt.begin(); // Among other things, this calls microWatt.blink() with default arguments (LED on pin 13, 4 ON-OFF cycles, duration of 50msec)

  delay(3000); // Not required. Only put here to visually separate the blinks made in microWatt.begin() and the ones from microWatt.blink().
}

void loop() {
  microWatt.blink(13, 1, 100); // Non-default arguments. LED on pin 13, 1 ON-oFF cycle, duration of 100msec.
}
