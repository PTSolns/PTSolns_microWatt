// EXAMPLE: Sensor connected via I2C
// Last update: Oct 27, 2023
//
// DESCRIPTION
// This example shows how to connect a sensor via I2C. First, the sensor's address is scanned and then the sensor is used to take a measurement.
// The sensor used in this example is the Sparkfun BME280 sensor. The focus in this example is not on this sensor. This example can easily be adapted to any other I2C sensor.
// More details about this particular sensor can be found here:
// https://github.com/sparkfun/SparkFun_BME280_Arduino_Library or here: https://www.sparkfun.com/products/14348 
//
// This example is separated into parts to show the different features of the mSL.
// Part A) Make sure all lines relating to the BME280 sensor module are commented out. We will first do an I2C scan to confirm the peripheral's address.
//         Run the sketch with basically only the microWatt.I2Cscan() enabled. Go to Serial Monitor on baud 115200 and see the output.
//         The microWatt.I2Cscan() uses the default I2C pins, but these can be changed by including them in the argument as such microWatt.I2Cscan(SDA_pin, SCL_pin);
//
// Part B) The address seen in the Serial Monitor on baud 115200 likely was 0x77. Enter this into the command below mySensor.setI2CAddress(0x77);.
//         Comment the microWatt.I2Cscan() line and uncomment all the lines labeled for Part B).
//
// NOTES - The command microWatt.I2Cscan() also sets the I2C pins using Wire.begin(). Therefore, it is not required to first call microWatt.setI2Cpins(). 
//       
//
// HARDWARE CONFIGURATION
// Connect the following pins:
// Sensor <--> ESP32 microWatt
//   3V3  <--> 3.3V
//   GND  <--> GND
//   SDA  <--> G21 (by default)
//   SCL  <--> G22 (by default)

#include <PTSolns_microWatt.h>
#include "SparkFunBME280.h"

microWatt microWatt;
BME280 mySensor; 

void setup() {
  // Serial.begin(115200); // Doesn't have to be called. If you want Serial Monitor on any other baud besides 115200, then uncomment this here. Otherwise, let microWatt.begin() below initiate Serial on baud 115200. Either way works!

  microWatt.begin(); // Not requried but nice to have. Also initiates Serial.begin(115200) so you don't need to do it above, but you can if you want to.

  microWatt.setI2Cpins(); // Call this to set the I2C pins (needed for the BME280 sensor module).
  //microWatt.setI2Cpins(SDA_pin, SCL_pin); // Use this command instead if you want to use I2C pins other than the default (SDA = G21, SCL = G22)

  // Comment the two lines below for Part A). Uncomment the two lines below for Part B)
  //mySensor.setI2CAddress(0x77); 
  //mySensor.beginI2C(); 
}

void loop() {
  // Comment the two lines below for Part A). Uncomment the two lines below for Part B)
  // Serial.print("Temp = ");
  // Serial.println(mySensor.readTempC());
  // NOTE: There are several commands related to the BME280 which are not discussed here. See the above link for full details.

  // Uncomment the line below for Part A). Comment the line below for Part B)
  microWatt.I2Cscan();

  delay(1000); // Wait a bit before taking another sensor reading
}
