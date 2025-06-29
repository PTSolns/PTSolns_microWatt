// Example: Using BOOT button as general input button
// Last update: June 28, 2025
// contact@PTSOlns.com
//
// DESCRIPTION
// It is demonstrated how to use the BOOT button onboard the ESP32 microWatt as a general-purpose input button.
// Pressing the BOOT button after uploading this example will cause the onboard LED to turn ON.
// NOTE: We don't need to define the onboard LED on pin 13 as that is already done in the mSL.
// NOTE: We also don't need to define the BOOT button pin as that is also already done in the mSL.
//
// HARDWARE CONFIGURATION
// Nothing is needed besides the ESP32 microWatt.

// Load the microWatt Support Library (mSL)
#include <PTSolns_microWatt.h>

microWatt microWatt;

int BOOT_button_state = 0; 

void setup() {
  microWatt.begin();
}

void loop() {
  BOOT_button_state = digitalRead(BOOT_BUTTON); // Read the state of the button

  // If the button is not pressed, leave LED oFF. Else turn ON LED.
  if (BOOT_button_state == HIGH) {
    digitalWrite(LED_BUILTIN, LOW);

  } else {
    digitalWrite(LED_BUILTIN, HIGH);
  }

  delay(100);
}
