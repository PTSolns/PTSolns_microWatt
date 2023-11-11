// microWatt Support Library (mSL)
// Last Update: Nov 5, 2023
// contact@PTSolns.com

#ifndef PTSolns_microWatt_h
#define PTSolns_microWatt_h
#endif
#include <Arduino.h>
#include <inttypes.h>

#define BOOT_BUTTON 0 // Onbaord BOOT button for microWatt
#define LED_BUITLIN 13 // Onbaord LED for microWatt

const int SDA_pin_default = 21; // Default value for microWatt
const int SCL_pin_default = 22; // Default value for microWatt

class microWatt 
{
  public:
    
  uint8_t begin(const int LED = LED_BUITLIN, const int number_of_blink = 4, int time_on_blink = 50, int time_off_blink = 50);
  void blink(const int LED = LED_BUITLIN, int number_of_blink = 4, int time_on_blink = 50, int time_off_blink = 50);
  void blinkDelay(const int LED = LED_BUITLIN, int number_of_blink = 4, int time_on_blink = 50, int time_off_blink = 50);
  void printPinout();
  uint8_t setI2Cpins(const int SDA_pin = SDA_pin_default, const int SCL_pin = SCL_pin_default);
  void printI2Cpins();
  void printSPIpins();
  void I2Cscan(const int SDA_pin_scan = SDA_pin_default, const int SCL_pin_scan = SCL_pin_default);
  void fade(const int LED_pin = LED_BUITLIN, const int PWM_Channel = 0, const int PWM_freq = 500, const int PWM_res = 8, int fade_inc = 5, int time_step = 20);
  void deepSleep(uint32_t duration);
  void lightSleep(uint32_t duration);
  void setFreq(uint32_t CPUfreq);
  
  private:

  void callWire(const int SDA_pin = SDA_pin_default, const int SCL_pin = SCL_pin_default);
  void blinkWarning();

};
