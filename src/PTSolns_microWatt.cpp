// microWatt Support Library (mSL)
// Last Update: Oct 27, 2023
// contact@PTSolns.com

#include "PTSolns_microWatt.h"

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "Arduino.h"
#include <Wire.h>
#include <list>
#include <iostream>

int flag_wire = 0;
int flag_fade = 0;
int flag_blinkWarning = 0;
int SDA_pin_global = SDA_pin_default;
int SCL_pin_global = SCL_pin_default;
int mSL_code = 0;
int dutyCycle = 0;
int fade_inverter = 1;
unsigned long fade_timer;

uint8_t microWatt::begin(const int LED, int number_of_blink, int time_between_blink) {

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
  Serial.println("     microWatt.begin(const int LED = LED_buildIn, int number_of_blink = 4, int time_between_blink = 50)");
  Serial.println("     microWatt.blink(const int LED = LED_buildIn, int number_of_blink = 4, int time_between_blink = 50)");
  Serial.println("     microWatt.setI2Cpins(const int SDA_pin = SDA_pin_default, const int SCL_pin = SCL_pin_default)");  
  Serial.println("     microWatt.printI2Cpins()");  
  Serial.println("     microWatt.I2Cscan(const int SDA_pin_scan = SDA_pin_default, const int SCL_pin_scan = SCL_pin_default)");  
  Serial.println("     microWatt.printSPIpins()"); 
  Serial.println("     microWatt.printPinout()");
  Serial.println("     microWatt.fade(const int LED_pin = LED_buildIn, const int PWM_Channel = 0, const int PWM_freq = 500, const int PWM_res = 8, int fade_inc = 5, int time_step = 20)");
  Serial.println("");

  Serial.println("For further details visit: https://github.com/PTSolns/PTSolns_microWatt");
  Serial.println("");

  // printPinout();

  blink(LED, number_of_blink, time_between_blink);

  return mSL_code;
}


void microWatt::blink(const int LED, int number_of_blink, int time_between_blink) {
	
  pinMode(LED, OUTPUT);
  
  for (int i = 1; i <= number_of_blink; ++i) {
    digitalWrite(LED, HIGH);
    delay(time_between_blink); 
    digitalWrite(LED, LOW);
    delay(time_between_blink); 
  }
}


void microWatt::blinkWarning() {
	if (flag_blinkWarning == 0) {
		flag_blinkWarning = 1;
		pinMode(LED_buildIn, OUTPUT);
	}
  
	for (int i = 1; i <= 10; ++i) {
		digitalWrite(LED_buildIn, HIGH);
		delay(30); 
		digitalWrite(LED_buildIn, LOW);
		delay(30); 
	}
  
  delay(500);
}				


void microWatt::printPinout() {
  Serial.println("                       microWatt v1.1+ Pinout");
  Serial.println("                     __________________________");
  Serial.println("                    | |                      | |");
  Serial.println("               Vin--| |                      | |--Vin");
  Serial.println("               GND--| |                      | |--GND");
  Serial.println("              3.3V--| |         ESP32        | |~~G22 -- SCL");
  Serial.println("                EN--| |         Module       | |~~G21 -- SDA");
  Serial.println(" Input only -- SVP--| |                      | |~~G23 -- COPI");
  Serial.println(" Input only -- SVN--| |                      | |~~G19 -- CIPO");
  Serial.println(" Input only -- G34--| |                      | |~~G18 -- SCK");
  Serial.println(" Input only -- G35--| |                      | |~~G5 --- CS");
  Serial.println("               G32~~| |______________________| |~~TX");
  Serial.println("               G33~~|                          |~~RX");
  Serial.println("               GND--|                          |--GND");
  Serial.println("               G25~~|                          |~~G17");
  Serial.println("               G26~~|                          |~~G16");
  Serial.println("               G27~~|                          |~~G4");
  Serial.println("               G14~~|                          |~~G0");
  Serial.println("               G12~~|                          |~~G2");
  Serial.println("Onboard LED -- G13~~|                          |~~G15");
  Serial.println("              3.3V--|         ________         |--3.3V");
  Serial.println("               GND--|        |        |        |--GND");
  Serial.println("              Vusb--|        |  USB-C |        |--Vusb");
  Serial.println("                    |________|________|________|");      
  Serial.println("");
  Serial.println("     NOTE: Pins connected by symbol '~~' are PWM capable pins.");
  Serial.println("           PWM_pins = {G0, G1, G3, G4, G5, G12, G13, G14, G15, G16, G17, G18, G19, G21, G22, G23, G25, G26, G27, G32, G33}");
  Serial.println("");
}


void microWatt::callWire(const int SDA_pin, const int SCL_pin) {
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


uint8_t microWatt::setI2Cpins(const int SDA_pin, const int SCL_pin) {
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


void microWatt::I2Cscan(const int SDA_pin, const int SCL_pin) {
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


void microWatt::fade(const int LED_pin, const int PWM_Channel, const int PWM_freq, const int PWM_res, int fade_inc, int time_step) {
	
	const int duty_max = (int)(pow(2, PWM_res) - 1);
	bool found =  false;
	
	// Call this only once at the beginning
	if (flag_fade == 0) {
		flag_fade = 1;

		std::list<int> PWM_pins = {0, 1, 3, 4, 5, 12, 13, 14, 15, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33};
		
		for (int pin : PWM_pins) {
			if (LED_pin == pin) {
				found = true;
				break;
			}
		}
		
		// Forever while loop to print warning message and turn on warning blinkWarning pattern.
		if (!found) {
			delay(500);
			while(1) {
				Serial.println("WARNING!");
				Serial.print("     LED_pin = G");
				Serial.print(LED_pin);
				Serial.println(" input in .fade() is not allowed. Please choose a PWM pin from the following list: ");	
				Serial.println("     PWM_pins = {G0, G1, G3, G4, G5, G12, G13, G14, G15, G16, G17, G18, G19, G21, G22, G23, G25, G26, G27, G32, G33}");
				Serial.println("     Program stuck in a forever loop ...");
				Serial.println("");
				
				blinkWarning();
			}
		}
		ledcSetup(PWM_Channel, PWM_freq, PWM_res);
		ledcAttachPin(LED_pin, PWM_Channel);
	}
	
	if ((millis() - fade_timer) >= time_step) {
		fade_timer = millis();
		dutyCycle = dutyCycle + fade_inverter*fade_inc;
		ledcWrite(PWM_Channel, dutyCycle);
		
		if ((dutyCycle <= 0) || (dutyCycle >= duty_max)) {
			fade_inverter = -fade_inverter;
		}
	}
}
