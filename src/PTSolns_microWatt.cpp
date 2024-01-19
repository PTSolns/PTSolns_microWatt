// microWatt Support Library (mSL)
// Last Update: Jan 4, 2024
// contact@PTSolns.com

#include "PTSolns_microWatt.h"

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "Arduino.h"
#include <Wire.h>
#include <list>
#include <iostream>

#define conversion_uS_S 1000000ULL

int flag_wire = 0;
int flag_fade [34] = {};
int flag_blinkWarning = 0;
int SDA_pin_global = SDA_pin_default;
int SCL_pin_global = SCL_pin_default;
int mSL_code = 0;
int dutyCycle [34] = {};
int fade_inverter [34] = {};
unsigned long fade_timer [34] = {};
unsigned long blink_timer [34] = {};
int blink_counter [34] = {};
int blink_data [34] = {};
int blinkDelay_data [34] = {};

// #########################################################################################
// MAX17048 section
// #########################################################################################


uint8_t charger::setI2Cpins(const int SDA_pin, const int SCL_pin)
{
    microWatt *mw = new microWatt();
    mSL_code = mw->setI2Cpins(SDA_pin, SCL_pin);
    return mSL_code;
}

void charger::printI2Cpins()
{
    microWatt *mw = new microWatt();
    mw->printI2Cpins();
}

float charger::battVoltage()
{
    uint16_t vCell = read16(MAX17048_VCELL);
    return (((float)vCell) * MAX17048_MULTIPLIER);
}

float charger::battSOC()
{
    uint16_t soc = read16(MAX17048_SOC);
    float percent = (float)((soc & 0xFF00) >> 8);
    percent += ((float)(soc & 0x00FF)) / 256.0;
    return percent;
}

uint8_t charger::chargerID()
{
    uint16_t vresetID = read16(MAX17048_VRESET_ID);
    return (vresetID & 0xFF);
}

uint8_t charger::setResetVoltage(uint8_t threshold)
{
    uint16_t vreset = read16(MAX17048_VRESET_ID);
    vreset &= 0x01FF;
    vreset |= ((uint16_t)threshold << 9);
    return write16(vreset, MAX17048_VRESET_ID);
}

uint8_t charger::setResetVoltage(float threshold)
{
    uint8_t thresh = (uint8_t)(constrain(threshold, 0.0, 5.08) / 0.04);
    return setResetVoltage(thresh);
}

uint8_t charger::getResetVoltage(void)
{
    uint16_t threshold = read16(MAX17048_VRESET_ID) >> 9;
    return ((uint8_t)threshold);
}

uint8_t charger::enableComparator(void)
{
    uint16_t vresetReg = read16(MAX17048_VRESET_ID);
    vresetReg &= ~(1 << 8); // Clear bit to enable comparator
    return write16(vresetReg, MAX17048_VRESET_ID);
}

uint8_t charger::disableComparator(void)
{
    uint16_t vresetReg = read16(MAX17048_VRESET_ID);
    vresetReg |= (1 << 8); // Set bit to disable comparator
    return write16(vresetReg, MAX17048_VRESET_ID);
}

float charger::getChangeRate(void)
{
    int16_t changeRate = read16(MAX17048_CRATE);
    float changerate_f = changeRate * 0.208;
    return (changerate_f);
}

uint8_t charger::getStatus(void)
{
    uint8_t statusReg = read16(MAX17048_STATUS) >> 8;
    return (statusReg & 0x7F); // Highest bit is don't care
}

bool charger::isReset(bool clear)
{
    uint8_t status = getStatus();
    bool flag = (status & MAX17048_STATUS_RI) > 0;
    if (flag && clear) // Clear the flag if requested
    {
        // Clear the aligned bit in the status register
        clearStatusRegBits(MAX17048_STATUS_RI << 8);
    }
    return (flag);
}

bool charger::isVoltageHigh(bool clear)
{
    uint8_t status = getStatus();
    bool flag = (status & MAX17048_STATUS_VH) > 0;
    if (flag && clear) // Clear the flag if requested
    {
        // Clear the aligned bit in the status register
        clearStatusRegBits(MAX17048_STATUS_VH << 8);
    }
    return (flag);
}

bool charger::isVoltageLow(bool clear)
{
    uint8_t status = getStatus();
    bool flag = (status & MAX17048_STATUS_VL) > 0;
    if (flag && clear) // Clear the flag if requested
    {
        // Clear the aligned bit in the status register
        clearStatusRegBits(MAX17048_STATUS_VL << 8);
    }
    return (flag);
}

bool charger::isVoltageReset(bool clear)
{
    uint8_t status = getStatus();
    bool flag = (status & MAX17048_STATUS_VR) > 0;
    if (flag && clear) // Clear the flag if requested
    {
        // Clear the aligned bit in the status register
        clearStatusRegBits(MAX17048_STATUS_VR << 8);
    }
    return (flag);
}

bool charger::isLow(bool clear)
{
    uint8_t status = getStatus();
    bool flag = (status & MAX17048_STATUS_HD) > 0;
    if (flag && clear) // Clear the flag if requested
    {
        // Clear the aligned bit in the status register
        clearStatusRegBits(MAX17048_STATUS_HD << 8);
    }
    return (flag);
}

bool charger::isChange(bool clear)
{
    uint8_t status = getStatus();
    bool flag = (status & MAX17048_STATUS_SC) > 0;
    if (flag && clear) // Clear the flag if requested
    {
        // Clear the aligned bit in the status register
        clearStatusRegBits(MAX17048_STATUS_SC << 8);
    }
    return (flag);
}

uint8_t charger::clearStatusRegBits(uint16_t mask)
{
    uint16_t statusReg = read16(MAX17048_STATUS);
    statusReg &= ~mask;                           // Clear the specified bits
    return (write16(statusReg, MAX17048_STATUS)); // Write the contents back again
}

uint8_t charger::clearAlert()
{
    uint16_t configReg = read16(MAX17048_CONFIG);
    configReg &= ~MAX17048_CONFIG_ALERT; // Clear ALRT bit manually.

    return write16(configReg, MAX17048_CONFIG);
}

uint8_t charger::getAlert(bool clear)
{
    // Read config reg, so we don't modify any other values:
    uint16_t configReg = read16(MAX17048_CONFIG);
    if (configReg & MAX17048_CONFIG_ALERT)
    {
        if (clear) // If the clear flag is set
        {
            configReg &= ~MAX17048_CONFIG_ALERT; // Clear ALRT bit manually.
            write16(configReg, MAX17048_CONFIG);
        }
        return 1;
    }

    return 0;
}

bool charger::enableSOCAlert()
{
    // Read config reg, so we don't modify any other values:
    uint16_t configReg = read16(MAX17048_CONFIG);
    configReg |= MAX17048_CONFIG_ALSC; // Set the ALSC bit
    // Update the config register, return false if the write fails
    if (write16(configReg, MAX17048_CONFIG) > 0)
        return (false);
    // Re-Read the config reg
    configReg = read16(MAX17048_CONFIG);
    // Return true if the ALSC bit is set, otherwise return false
    return ((configReg & MAX17048_CONFIG_ALSC) > 0);
}

bool charger::disableSOCAlert()
{
    // Read config reg, so we don't modify any other values:
    uint16_t configReg = read16(MAX17048_CONFIG);
    configReg &= ~MAX17048_CONFIG_ALSC; // Clear the ALSC bit
    // Update the config register, return false if the write fails
    if (write16(configReg, MAX17048_CONFIG) > 0)
        return (false);
    // Re-Read the config reg
    configReg = read16(MAX17048_CONFIG);
    // Return true if the ALSC bit is clear, otherwise return false
    return ((configReg & MAX17048_CONFIG_ALSC) == 0);
}

uint8_t charger::enableAlert(void)
{
    uint16_t statusReg = read16(MAX17048_STATUS);
    statusReg |= MAX17048_STATUS_EnVR; // Set EnVR bit
    return write16(statusReg, MAX17048_STATUS);
}

uint8_t charger::disableAlert(void)
{
    uint16_t statusReg = read16(MAX17048_STATUS);
    statusReg &= ~MAX17048_STATUS_EnVR; // Clear EnVR bit
    return write16(statusReg, MAX17048_STATUS);
}

uint8_t charger::getThreshold()
{
    uint16_t configReg = read16(MAX17048_CONFIG);
    uint8_t threshold = (configReg & 0x001F);

    // It has an LSb weight of 1%, and can be programmed from 1% to 32%.
    // The value is (32 - ATHD)%, e.g.: 00000=32%, 00001=31%, 11111=1%.
    // Let's convert our percent to that first:
    threshold = 32 - threshold;
    return threshold;
}

uint8_t charger::setThreshold(uint8_t percent)
{
    // The alert threshold is a 5-bit value that sets the state of charge level
    // where an interrupt is generated on the ALRT pin.

    // It has an LSb weight of 1%, and can be programmed from 1% to 32%.
    // The value is (32 - ATHD)%, e.g.: 00000=32%, 00001=31%, 11111=1%.
    // Let's convert our percent to that first:
    percent = (uint8_t)constrain((float)percent, 0.0, 32.0);
    percent = 32 - percent;

    // Read config reg, so we don't modify any other values:
    uint16_t configReg = read16(MAX17048_CONFIG);
    configReg &= 0xFFE0;  // Mask out threshold bits
    configReg |= percent; // Add new threshold

    return write16(configReg, MAX17048_CONFIG);
}

uint8_t charger::sleep()
{
    uint8_t result = write16(MAX17048_MODE_ENSLEEP, MAX17048_MODE);
    if (result)
        return (result); // Write failed. Bail.

    // Read config reg, so we don't modify any other values:
    uint16_t configReg = read16(MAX17048_CONFIG);
    if (configReg & MAX17048_CONFIG_SLEEP)
        return MAX17048_GENERIC_ERROR;

    configReg |= MAX17048_CONFIG_SLEEP; // Set sleep bit

    return write16(configReg, MAX17048_CONFIG);
}

uint8_t charger::wake()
{
    // Read config reg, so we don't modify any other values:
    uint16_t configReg = read16(MAX17048_CONFIG);
    if (!(configReg & MAX17048_CONFIG_SLEEP)) return MAX17048_GENERIC_ERROR; // Already sleeping

    configReg &= ~MAX17048_CONFIG_SLEEP; // Clear sleep bit
    uint8_t result = write16(configReg, MAX17048_CONFIG);

    if (result) return (result); // Write failed. Bail.
    return write16(0x0000, MAX17048_MODE);
}

uint8_t charger::getCompensation()
{
    uint16_t configReg = read16(MAX17048_CONFIG);
    uint8_t compensation = (configReg & 0xFF00) >> 8;
    return compensation;
}

uint16_t charger::getConfigRegister()
{
    return read16(MAX17048_CONFIG);
}

uint8_t charger::setCompensation(uint8_t newCompensation)
{
    // The CONFIG register compensates the ModelGauge algorith. The upper 8 bits
    // of the 16-bit register control the compensation.
    // Read the original configReg, so we can leave the lower 8 bits alone:
    uint16_t configReg = read16(MAX17048_CONFIG);
    configReg &= 0x00FF; // Mask out compensation bits
    configReg |= ((uint16_t)newCompensation) << 8;
    return write16(configReg, MAX17048_CONFIG);
}

uint8_t charger::setVALRTMax(uint8_t threshold)
{
    uint16_t valrt = read16(MAX17048_CVALRT);
    valrt &= 0xFF00; // Mask off max bits
    valrt |= (uint16_t)threshold;
    return write16(valrt, MAX17048_CVALRT);
}

uint8_t charger::setVALRTMax(float threshold)
{
    uint8_t thresh = (uint8_t)(constrain(threshold, 0.0, 5.1) / 0.02);
    return setVALRTMax(thresh);
}

uint8_t charger::getVALRTMax()
{
    uint16_t valrt = read16(MAX17048_CVALRT);
    valrt &= 0x00FF; // Mask off max bits
    return ((uint8_t)valrt);
}

uint8_t charger::setVALRTMin(uint8_t threshold)
{
    uint16_t valrt = read16(MAX17048_CVALRT);
    valrt &= 0x00FF; // Mask off min bits
    valrt |= ((uint16_t)threshold) << 8;
    return write16(valrt, MAX17048_CVALRT);
}

uint8_t charger::setVALRTMin(float threshold)
{
    uint8_t thresh = (uint8_t)(constrain(threshold, 0.0, 5.1) / 0.02);
    return setVALRTMin(thresh);
}

uint8_t charger::getVALRTMin()
{
    uint16_t valrt = read16(MAX17048_CVALRT);
    valrt >>= 8; // Shift min into LSB
    return ((uint8_t)valrt);
}

bool charger::isHibernating()
{
    uint16_t mode = read16(MAX17048_MODE);
    return ((mode & MAX17048_MODE_HIBSTAT) > 0);
}

uint8_t charger::getHIBRTActThr()
{
    uint16_t hibrt = read16(MAX17048_HIBRT);
    hibrt &= 0x00FF; // Mask off Act bits
    return ((uint8_t)hibrt);
}

uint8_t charger::setHIBRTActThr(uint8_t threshold)
{
    uint16_t hibrt = read16(MAX17048_HIBRT);
    hibrt &= 0xFF00; // Mask off Act bits
    hibrt |= (uint16_t)threshold;
    return write16(hibrt, MAX17048_HIBRT);
}

uint8_t charger::setHIBRTActThr(float threshold)
{
    // LSb = 1.25mV
    uint8_t thresh = (uint8_t)(constrain(threshold, 0.0, 0.31875) / 0.00125);
    return setHIBRTActThr(thresh);
}

uint8_t charger::getHIBRTHibThr()
{
    uint16_t hibrt = read16(MAX17048_HIBRT);
    hibrt >>= 8; // Shift HibThr into LSB
    return ((uint8_t)hibrt);
}

uint8_t charger::setHIBRTHibThr(uint8_t threshold)
{
    uint16_t hibrt = read16(MAX17048_HIBRT);
    hibrt &= 0x00FF; // Mask off Hib bits
    hibrt |= ((uint16_t)threshold) << 8;
    return write16(hibrt, MAX17048_HIBRT);
}

uint8_t charger::setHIBRTHibThr(float threshold)
{
    // LSb = 0.208%/hr
    uint8_t thresh = (uint8_t)(constrain(threshold, 0.0, 53.04) / 0.208);
    return setHIBRTHibThr(thresh);
}

uint8_t charger::write16(uint16_t data, uint8_t address)
{
    uint8_t msb, lsb;
    msb = (data & 0xFF00) >> 8;
    lsb = (data & 0x00FF);
    Wire.beginTransmission(MAX17048_ADDRESS);
    Wire.write(address);
    Wire.write(msb);
    Wire.write(lsb);
    return (Wire.endTransmission());
}

uint16_t charger::read16(uint8_t address)
{
    bool success = false;
    uint8_t retries = 3;
    uint16_t result = 0;

    while ((success == false) && (retries > 0))
    {
        Wire.beginTransmission(MAX17048_ADDRESS);
        Wire.write(address);
        Wire.endTransmission(false); // Don't release the bus

        if (Wire.requestFrom(MAX17048_ADDRESS, 2) == 2)
        {
            uint8_t msb = Wire.read();
            uint8_t lsb = Wire.read();
            result = ((uint16_t)msb << 8) | lsb;
            success = true;
        }
        else
        {
            retries--;
            delay(50);
        }
    }
    return (result);
}






uint8_t microWatt::begin(const int LED, int number_of_blink, int time_on_blink, int time_off_blink) {

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
  Serial.println("     microWatt.begin(const int LED = LED_BUITLIN, int number_of_blink = 4, int time_on_blink = 50, int time_off_blink = 50)");
  Serial.println("     microWatt.blink(const int LED = LED_BUITLIN, int number_of_blink = 4, int time_on_blink = 50, int time_off_blink = 50)");
  Serial.println("     microWatt.blinkDelay(const int LED = LED_BUITLIN, int number_of_blink = 4, int time_on_blink = 50, int time_off_blink = 50)");	
  Serial.println("     microWatt.setI2Cpins(const int SDA_pin = SDA_pin_default, const int SCL_pin = SCL_pin_default)");  
  Serial.println("     microWatt.printI2Cpins()");  
  Serial.println("     microWatt.I2Cscan(const int SDA_pin_scan = SDA_pin_default, const int SCL_pin_scan = SCL_pin_default)");  
  Serial.println("     microWatt.printSPIpins()"); 
  Serial.println("     microWatt.printPinout()");
  Serial.println("     microWatt.fade(const int LED_pin = LED_BUITLIN, const int PWM_Channel = 0, const int PWM_freq = 500, const int PWM_res = 8, int fade_inc = 5, int time_step = 20)");
  Serial.println("     microWatt.deepSleep(uint32_t duration)");
  Serial.println("     microWatt.lightSleep(uint32_t duration)");
  Serial.println("     microWatt.setFreq(uint32_t CPUfreq)");
  Serial.println("");

  Serial.println("For further details visit: https://github.com/PTSolns/PTSolns_microWatt");
  Serial.println("");

  // printPinout();

  pinMode(BOOT_BUTTON, INPUT);
	
  blinkDelay(LED, number_of_blink, time_on_blink, time_off_blink);

  return mSL_code;
}


void microWatt::blink(const int LED, int number_of_blink, int time_on_blink, int time_off_blink) {
	
	if (blink_data[LED] == 0) {
		blink_data[LED] = 1;
		pinMode(LED, OUTPUT);
	}
	
	if (number_of_blink == -1)  {
		blink_counter[LED] = -2; // Set below threshold indefinitely.
	}
	
	if (blink_counter[LED] < number_of_blink) {
		if (blink_data[LED] == 1) {
			blink_data[LED] = 2;
			digitalWrite(LED, HIGH);
			blink_timer[LED] = millis(); // Set first timer
		}

		if (((millis() - blink_timer[LED]) >= time_on_blink) && (blink_data[LED] == 2)) {
			blink_data[LED] = 3;
			digitalWrite(LED, LOW);
			blink_timer[LED] = millis(); // Set second timer
		}
	
		if (((millis() - blink_timer[LED]) >= time_off_blink) && (blink_data[LED] == 3)) {
			blink_data[LED] = 1;
			blink_counter[LED] = blink_counter[LED] + 1;
		}
	}
	
	if (number_of_blink == -1)  {
		blink_counter[LED] = 0; // Reset counter
	}
}

void microWatt::blinkDelay(const int LED, int number_of_blink, int time_on_blink, int time_off_blink) {  
	if (blinkDelay_data[LED] == 0) {
		blinkDelay_data[LED] = 1;
		pinMode(LED, OUTPUT);
	}

	if (number_of_blink == -1)  {
		blink_counter[LED] = -2; // Set below threshold indefinitely.
	}

	while (blink_counter[LED] < number_of_blink) {
		digitalWrite(LED, HIGH);
		delay(time_on_blink); 
		digitalWrite(LED, LOW);
		delay(time_off_blink); 

		blink_counter[LED] = blink_counter[LED] + 1;
	}

	if (number_of_blink == -1)  {
		blink_counter[LED] = 0; // Reset counter
	}
}



void microWatt::blinkWarning() {
	if (flag_blinkWarning == 0) {
		flag_blinkWarning = 1;
		pinMode(LED_BUITLIN, OUTPUT);
	}
  
	for (int i = 1; i <= 10; ++i) {
		digitalWrite(LED_BUITLIN, HIGH);
		delay(30); 
		digitalWrite(LED_BUITLIN, LOW);
		delay(30); 
	}
  
  delay(500);
}				


void microWatt::printPinout() {
  Serial.println("                       microWatt v2.0+ Pinout");
  Serial.println("                     __________________________");
  Serial.println("                    | |                      | |");
  Serial.println("               Vin--| |                      | |--Vin");
  Serial.println("               GND--| |                      | |--GND");
  Serial.println("              3.3V--| |         ESP32        | |~~G22 -- SCL");
  Serial.println("                EN->| |         Module       | |~~G21 -- SDA");
  Serial.println("               SVP->| |                      | |~~G23 -- COPI (VSPI)");
  Serial.println("               SVN->| |                      | |~~G19 -- CIPO (VPSI)");
  Serial.println("               G34->| |                      | |~~G18 -- SCK (VSPI)");
  Serial.println("               G35->| |                      | |~~G5 --- CS (VSPI)");
  Serial.println("               G32~~| |______________________| |~~TX --- UART0");
  Serial.println("               G33~~|                          |~~RX --- UART0");
  Serial.println("               GND--|                          |--GND");
  Serial.println("               G25~~|                          |~~G17 -- UART2");
  Serial.println("               G26~~|                          |~~G16 -- UART2");
  Serial.println("  CS (HSPI) -- G15~~|                          |~~G4");
  Serial.println(" SCK (HSPI) -- G14~~|                          |~~G0");
  Serial.println("CIPO (HSPI) -- G12~~|                          |~~G2");
  Serial.println("COPI (HSPI) -- G13~~|                          |~~G27");
  Serial.println("              3.3V--|         ________         |--3.3V");
  Serial.println("               GND--|        |        |        |--GND");
  Serial.println("              Vusb--|        |  USB-C |        |--Vusb");
  Serial.println("                    |________|________|________|");      
  Serial.println("");
  Serial.println("     NOTE: Pins connected by symbol '~~' are PWM capable pins.");
  Serial.println("           PWM_pins = {G0, G1, G2, G3, G4, G5, G12, G13, G14, G15, G16, G17, G18, G19, G21, G22, G23, G25, G26, G27, G32, G33}");
  Serial.println("     NOTE: Pins connected by symbol '->' are input only pins.");	
  Serial.println("     NOTE: G13 pin controls onboard LED.");
  Serial.println("     NOTE: For complete description of pin definitions, please consult the user manual.");
  Serial.println("");
  
  delay(1000);
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
      if (address < 16) 
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
	if (flag_fade[LED_pin] == 0) {
		flag_fade[LED_pin] = 1;
		
		std::list<int> PWM_pins = {0, 1, 2, 3, 4, 5, 12, 13, 14, 15, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33};
		
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
				Serial.println("     PWM_pins = {G0, G1, G2, G3, G4, G5, G12, G13, G14, G15, G16, G17, G18, G19, G21, G22, G23, G25, G26, G27, G32, G33}");
				Serial.println("     Program stuck in a forever loop ...");
				Serial.println("");
				
				blinkWarning();
			}
		}
		ledcSetup(PWM_Channel, PWM_freq, PWM_res);
		ledcAttachPin(LED_pin, PWM_Channel);
		
		fade_inverter[LED_pin] = 1;
	}
	
	if ((millis() - fade_timer[LED_pin]) >= time_step) {
		fade_timer[LED_pin] = millis();
		dutyCycle[LED_pin] = dutyCycle[LED_pin] + fade_inverter[LED_pin]*fade_inc;
		ledcWrite(PWM_Channel, dutyCycle[LED_pin]);
		
		if ((dutyCycle[LED_pin] <= 0) || (dutyCycle[LED_pin] >= duty_max)) {
			fade_inverter[LED_pin] = -fade_inverter[LED_pin];
		}
	}
}

void microWatt::deepSleep(uint32_t duration) {
	esp_sleep_enable_timer_wakeup(conversion_uS_S * duration);
    esp_deep_sleep_start();
}


void microWatt::lightSleep(uint32_t duration) {
	esp_sleep_enable_timer_wakeup(conversion_uS_S * duration);
    esp_light_sleep_start();
}


void microWatt::setFreq(uint32_t CPUfreq) {
    setCpuFrequencyMhz(CPUfreq); // Measured in MHz
}

