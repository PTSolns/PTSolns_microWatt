#ifndef PTSolns_microWatt_h
#define PTSolns_microWatt_h
#endif
#include <Arduino.h>
#include <inttypes.h>

const int SDA_pin_default = 21; // Default value for microWatt
const int SCL_pin_default = 22; // Default value for microWatt
const int LED_buildIn = 13;

class microWatt 
{
  public:
    
  uint8_t begin(int LED = LED_buildIn, int number_of_blink = 4, int time_between_blink = 50);
	void blink(int LED = LED_buildIn, int number_of_blink = 4, int time_between_blink = 50);
  void printPinout();
  uint8_t setI2Cpins(int SDA_pin = SDA_pin_default, int SCL_pin = SCL_pin_default);
  void printI2Cpins();
  void printSPIpins();
  void I2Cscan(int SDA_pin_scan = SDA_pin_default, int SCL_pin_scan = SCL_pin_default);

  private:

  void callWire(int SDA_pin = SDA_pin_default, int SCL_pin = SCL_pin_default);

};