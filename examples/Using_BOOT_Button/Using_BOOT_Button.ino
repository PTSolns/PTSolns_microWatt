// Example: Using BOOT button as general inout button
// Last update: Nov 11, 2023
// contact@PTSOlns.com
//
// DESCRIPTION
// It is demonstrated how to use the BOOT button onboard the ESP32 microWatt as a general purpose input button.
// Pressing the BOOT button after uploading this example will cause the onboard LED to turn ON.
// NOTE: We don't need to define the onboard LED on pin 13 as that is already done in the mSL.
//
// HARDWARE CONFIGURATION
// Nothing is needed besides the ESP32 microWatt.

// Load the microWatt Support Library (mSL)
#include <PTSolns_microWatt.h>

microWatt microWatt;

const int BOOT_pin = 0; // The BOOT button is connected to pin G0.
int BOOT_button_state = 0; 

void setup() {
  microWatt.begin();

  pinMode(BOOT_pin, INPUT);
}

void loop() {
  BOOT_button_state = digitalRead(BOOT_pin); // Read the state of the button

  // If button is not pressed, leave LED oFF. Else turn ON LED.
  if (digitalRead(BOOT_pin) == HIGH) {
    digitalWrite(LED_BUITLIN, LOW);

  } else {
    digitalWrite(LED_BUITLIN, HIGH);
  }

  delay(100);
}
