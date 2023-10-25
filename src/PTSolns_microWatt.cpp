// microWatt Support Library (mSL)
// Last Update: Oct 25, 2023
// contact@PTSolns.com

#include "PTSolns_microWatt.h"

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "Arduino.h"
#include <Wire.h>

int flag_wire = 0;
int SDA_pin_global = SDA_pin_default;
int SCL_pin_global = SCL_pin_default;
int mSL_code = 0;


uint8_t microWatt::begin(int LED, int number_of_blink, int time_between_blink) {

  delay(1000);

  if (!Serial) {
    Serial.begin(115200);
    mSL_code = 1;
  }

  Serial.println("*****");
  Serial.println("Starting microWatt Support Library (mSL) ...");
  Serial.println("");
  if (mSL_code == 1) {
    Serial.println("-> microWatt Support Library (mSL) initiated Serial with baud rate 115200.");
    Serial.print("-> mSL code = ");
    Serial.println(mSL_code);
    Serial.println("");
  }
  Serial.println("Available commands:");
  Serial.println("     microWatt.begin(int LED = LED_buildIn, int number_of_blink = 4, int time_between_blink = 50)");
	Serial.println("     microWatt.blink(int LED = LED_buildIn, int number_of_blink = 4, int time_between_blink = 50)");
  Serial.println("     microWatt.setI2Cpins(int SDA_pin = SDA_pin_default, int SCL_pin = SCL_pin_default)");  
  Serial.println("     microWatt.printI2Cpins()");  
  Serial.println("     microWatt.I2Cscan(int SDA_pin_scan = SDA_pin_default, int SCL_pin_scan = SCL_pin_default)");  
  Serial.println("     microWatt.printSPIpins()"); 
  Serial.println("     microWatt.printPinout()");
  Serial.println("");

  Serial.println("For further details visit: https://github.com/PTSolns/PTSolns_microWatt");
  Serial.println("");

  // printPinout();

  blink(LED, number_of_blink, time_between_blink);

  return mSL_code;
}


void microWatt::blink(int LED, int number_of_blink, int time_between_blink) {
  pinMode(LED, OUTPUT);
  
  for (int i = 1; i <= number_of_blink; ++i) {
    digitalWrite(LED, HIGH);
    delay(time_between_blink); 
    digitalWrite(LED, LOW);
    delay(time_between_blink); 
  }
}


void microWatt::printPinout() {
  Serial.println("                      microWatt v1.2 PINOUT");
  Serial.println("                    __________________________");
  Serial.println("                   | |                      | |");
  Serial.println("               Vin | |                      | | Vin");
  Serial.println("               GND | |                      | | GND");
  Serial.println("              3.3V | |         ESP32        | | G22 -- SCL");
  Serial.println("                EN | |         Module       | | G21 -- SDA");
  Serial.println(" Input only -- SVP | |                      | | G23 -- COPI");
  Serial.println(" Input only -- SVN | |                      | | G19 -- CIPO");
  Serial.println(" Input only -- G34 | |                      | | G18 -- SCK");
  Serial.println(" Input only -- G35 | |                      | | G5 --- CS");
  Serial.println("               G32 | |______________________| | TX");
  Serial.println("               G33 |                          | RX");
  Serial.println("               GND |                          | GND");
  Serial.println("               G25 |                          | G17");
  Serial.println("               G26 |                          | G16");
  Serial.println("               G27 |                          | G4");
  Serial.println("               G14 |                          | G0");
  Serial.println("               G12 |                          | G2");
  Serial.println("Onboard LED -- G13 |                          | G15");
  Serial.println("              3.3V |         ________         | 3.3V");
  Serial.println("               GND |        |        |        | GND");
  Serial.println("              Vusb |        |  USB-C |        | Vusb");
  Serial.println("                   |________|________|________|");      
  Serial.println("");
}


void microWatt::callWire(int SDA_pin, int SCL_pin) {
  if (flag_wire == 0) { 
    flag_wire = 1;
    if ((SDA_pin != SDA_pin_default) || (SCL_pin != SCL_pin_default)) {
      Serial.println("Calling Wire.begin(SDA, SCL) with specified I2C pins:");
    } else {
      Serial.println("Calling Wire.begin(SDA, SCL) with default I2C pins:");
    }
    Serial.print("     SDA = G");
    Serial.println(SDA_pin);
    Serial.print("     SCL = G");
    Serial.println(SCL_pin);
    Serial.println("");
    Wire.begin(SDA_pin, SCL_pin);
  }
}


uint8_t microWatt::setI2Cpins(int SDA_pin, int SCL_pin) {
  SDA_pin_global = SDA_pin;
  SCL_pin_global = SCL_pin;
  callWire(SDA_pin, SCL_pin);

  mSL_code = 2;

  if (mSL_code == 2) {
    Serial.println("-> microWatt Support Library (mSL) set I2C pins.");
    Serial.print("-> mSL code = ");
    Serial.println(mSL_code);
    Serial.println("");
  }

  return mSL_code;
}


void microWatt::printI2Cpins() {
  if ((SDA_pin_global != SDA_pin_default) || (SCL_pin_global != SCL_pin_default)) {
    Serial.println("Default I2C pins were overwritten to:");
  } else {
    Serial.println("Default I2C pins:");
  }
  Serial.print("     SDA = G");
  Serial.println(SDA_pin_global);
  Serial.print("     SCL = G");
  Serial.println(SCL_pin_global);
  Serial.println("");
}


void microWatt::printSPIpins() {
  // Need to add if loop similar to printI2Cpins()
  Serial.println("Default SPI pins (VSPI):");
  Serial.println("     COPI = G23, where COPI = Controller Out Peripheral In");
  Serial.println("     CIPO = G19, where CIPO = Controller In Peripheral Out");
  Serial.println("     SCK  = G18");
  Serial.println("     CS   = G5");
  Serial.println("");
}


void microWatt::I2Cscan(int SDA_pin, int SCL_pin) {
  byte error, address;
  int nDevices;

  // call Wire.begin only if it has not already been called in setI2Cpins()
  if (flag_wire == 0) {
    callWire(SDA_pin, SCL_pin);
  }

  if ((SDA_pin_global != SDA_pin_default) || (SCL_pin_global != SCL_pin_default)) {
    Serial.println("Starting I2C scan on pins:");
    Serial.print("     SDA pin = G");
    Serial.println(SDA_pin_global);
    Serial.print("     SCL pin = G");
    Serial.println(SCL_pin_global);
    Serial.println("");
  } else {
    Serial.println("Starting I2C scan on pins:");
    Serial.print("     SDA pin = G");
    Serial.println(SDA_pin);
    Serial.print("     SCL pin = G");
    Serial.println(SCL_pin);
    Serial.println("");
  }

  nDevices = 0;

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0){
      Serial.print("     I2C device found at address 0x");
      if (address < 16) 
        Serial.print("0");
      Serial.println(address, HEX);

      nDevices++;
    }
    else if (error == 4) {
      Serial.print("     Unknown error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address, HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("     No I2C devices found\n");
  else
    Serial.println("Finished I2C scan!\n");

  delay(5000);
}
