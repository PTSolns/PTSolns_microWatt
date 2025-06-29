// EXAMPLE: Blink
// Last update: June 28, 2025
// contact@PTSolns.com
//
// DESCRIPTION
// This example demonstrates the classic Blink example. 
// Two types of Blink routines are incorporated, one that uses delay() and one that uses millis()
// using delay -> .blinkDelay(), using millis() -> .blink()
//
// We will use the onboard LED on pin 13 (LED_BUILTIN) for this example. 
//
// In loop() the call microWatt.blink(LED_BUILTIN, 4, 100) is made. 
// The first argument is the pin to the LED.
// The second argument is the number of blink ON-OFF cycles. It is set to 4 as the loop(). After four blinks it will stop. If you want to continue blink indefinately put -1 instead.
// The last argument is the equal ON and OFF durations measured in milliseconds.
//
// HARDWARE CONFIGURATION
// Connect the ESP32 microWatt via USB to your laptop. No other hardware is required.

#include <PTSolns_microWatt.h>

microWatt microWatt;

void setup() {
  // Nothing needed for this example.
}

void loop() {
  // Comment and uncomment as desired...

  microWatt.blink(LED_BUILTIN, 4, 100); // Non-default arguments. LED on pin 13, 4 ON-oFF cycle, duration of 100msec.
  //microWatt.blink(LED_BUILTIN, -1, 200); // Non-default arguments. LED on pin 13, indefinate blink cycles, duration of 200msec.

  //microWatt.blinkDelay(LED_BUILTIN, 4, 300); // Non-default arguments. LED on pin 13, 4 ON-oFF cycle, duration of 100msec.
  //microWatt.blinkDelay(LED_BUILTIN, -1, 400); // Non-default arguments. LED on pin 13, indefinate blink cycles, duration of 200msec.
}
