// Last update: Oct 25, 2023

// COMMANDS
// .begin()                                          - Uses default blink settings
// .begin(LED, number_of_blink, time_between_blink); - Overwrites default blink settings
// .blink(LED, number_of_blink, time_between_blink); - Blinks as per settings
// .printPinout();                                   - Prints the pinout of the microcontroller
// .setI2Cpins(SDA_pin, SCL_pin);                    - Set I2C pins 
// .printI2Cpins();                                  - Print out the default I2C pins
// .printSPIpins();                                  - Print out the default SPI pins
// .I2Cscan();                                       - No Parameters resorts to default, which are SDA=21, SCL=22. 
// .I2Cscan(SDA_pin,SCL_pin);

// mSL CODES (mSL_code)
// An integer return value from microWatt.begin().
//   = 0 -> Everything is OK
//   = 1 -> Relating to Serial. Likely Serial.begin(baud rate) was not called BEFORE any microWatt commands, and hence it was initiated by the microWatt Support Library. Otherwise check baud rate.
//          If microWatt Support Library initiated Serial, it would be on Serial.begin(115200).
// Tip: If printing the error code to Serial does not work, try an LCD such as the 1602 over I2C. Not seeing error code over Serial might be an issue with Serial itself.

#include "PTSolns_microWatt.h"

microWatt microWatt;

int mSL; // microWatt Support Library (mSL) return code
int i;

void setup() {
  //Serial.begin(9600); // Must be called BEFORE any microWatt commands.

  //error_code = microWatt.begin(13, 4, 500);
  mSL = microWatt.begin();

  //microWatt.blink(13, 4, 1000);


  //Serial.println(error_code);
  //microWatt.printPinout();

 // microWatt.printI2Cpins();

  mSL = microWatt.setI2Cpins();

 // microWatt.printI2Cpins();
  //microWatt.printSPIpins();
}

void loop() {
 // microWatt.blink(13,1,1000);

 // microWatt.I2Cscan(21, 23);

  delay(1000);
}
