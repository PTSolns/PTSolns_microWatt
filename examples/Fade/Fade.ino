// EXAMPLE: Fade
// Last update: Oct 27, 2023
// contact@PTSolns.com
//
// DESCRIPTION
// This example demonstrates the classic Fade. It is broken into several Parts to show different featrues of the .fade() function.
//
// Part A) The onboard LED is faded in and out using all the default values. Simply compile/upload this sketch as is, and observe the onboard LED.
// 
// Part B) The same onboard LED is faded, but the user is encourgaed to change the default settings. Change either fade_inc or time_step below and see the change in onbaord LED.
//
// Part C) Let's change the LED_pin to something else. First let's change it to some pin that is *NOT* allowed, e.g. LED_pin = 130. 
//         Change the LED_pin compile/upload and open Serial Monitor to 115200 baud. Observe the onboard LED pattern also.
//         Clearly something went wrong! When that happens the mSL will let you know. For reference, the allowable PWM pins are as follows:
//         PWM_pins = {G0, G1, G3, G4, G5, G12, G13, G14, G15, G16, G17, G18, G19, G21, G22, G23, G25, G26, G27, G32, G33}
//         You can also print the pinout diagram and take a look there to see which pins are PWM capable. Uncomment the microWatt.printPinout() in the setup() below and see.
//        
// Part D) Finally, change the LED_pin to one of the allowed PWM pins (but not 13, as that's the onboard pin and we want to check one of the exteranl pins).
//         Connect a LED in series with a 200 ohm to 1k ohm resistor to the external pin you set. Use a breadboard for easy connection.
// 
// Tips: To use all the default values, use:
//       microWatt.fade();
//
//       To specify a different LED_pin but still use all the other default values, use:
//       microWatt.fade(LED_pin);
//
//       To specify all different values from defaults, use:
//       microWatt.fade(LED_pin, PWM_Channel, PWM_freq, PWM_res, fade_inc, time_step)

#include <PTSolns_microWatt.h>

microWatt microWatt;

const int LED_pin     = 13;   // [Default: 13] Pin 13 is builtin pin.
const int PWM_channel = 0;    // [Default: 0, Allowed: 0-15] ESP32 has 16 channels (0 to 15) that can generate 16 independent waveforms
const int PWM_freq    = 500;  // [Default: 500] PWM frequency, in Hz.
const int PWM_res     = 8;    // [Default: 8, Allowed: 1-16] Resolution setting in bits. 8 bits = 0 to 2^8-1, 16 bits = 0 to 2^16-1. microWatt has up to 16 bit resolution. 
int fade_inc          = 5;    // [Default: 5] The amount the LED changes brightness per iteration. 
int time_step         = 20;   // [Default: 20] Time between increments, in milliseconds.

void setup() {
  microWatt.begin(); 

  // Uncomment the line below for Part C)
  microWatt.printPinout();
}

void loop() {
  // Uncomment the line below for Part A), keep the others commented out.
  microWatt.fade();

  // Uncomment the line below for Part B), keep the others commented out.
  //microWatt.fade(LED_pin, PWM_Channel, PWM_freq, PWM_res, fade_inc, time_step); 

  // Uncomment the line below for Part C) and D), keep the others commented out.
  //microWatt.fade(LED_pin);
}
