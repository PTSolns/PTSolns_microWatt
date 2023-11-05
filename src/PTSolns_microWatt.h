// microWatt Support Library (mSL)
// Last Update: Oct 27, 2023
// contact@PTSolns.com

#ifndef PTSolns_microWatt_h
#define PTSolns_microWatt_h
#endif
#include <Arduino.h>
#include <inttypes.h>

const int SDA_pin_default = 21; // Default value for microWatt
const int SCL_pin_default = 22; // Default value for microWatt
const int LED_buildIn = 13; // Default value for microWatt

class microWatt 
{
  public:
    
  uint8_t begin(const int LED = LED_buildIn, const int number_of_blink = 4, int time_between_blink = 50);
  void blink(const int LED = LED_buildIn, const int number_of_blink = 4, int time_between_blink = 50);
  void blinkDelay(const int LED = LED_buildIn, int number_of_blink = 4, int time_on_blink = 50, int time_off_blink = 50);
  void printPinout();
  uint8_t setI2Cpins(const int SDA_pin = SDA_pin_default, const int SCL_pin = SCL_pin_default);
  void printI2Cpins();
  void printSPIpins();
  void I2Cscan(const int SDA_pin_scan = SDA_pin_default, const int SCL_pin_scan = SCL_pin_default);
  void fade(const int LED_pin = LED_buildIn, const int PWM_Channel = 0, const int PWM_freq = 500, const int PWM_res = 8, int fade_inc = 5, int time_step = 20);
  
  private:

  void callWire(const int SDA_pin = SDA_pin_default, const int SCL_pin = SCL_pin_default);
  void blinkWarning();

};
