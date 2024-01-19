// microWatt Support Library (mSL)
// Last Update: Jan 4, 2024
// contact@PTSolns.com

#ifndef PTSolns_microWatt_h
#define PTSolns_microWatt_h
#endif
#include <Arduino.h>
#include <inttypes.h>

#define BOOT_BUTTON 0  // Onboard BOOT button for microWatt
#define LED_BUITLIN 13 // Onboard LED for microWatt

#define MAX17048_FULL_SCALE 5.12
#define MAX17048_MULTIPLIER 7.8125E-5
#define MAX17048_VCELL 0x02             // R - 12-bit A/D measurement of battery voltage
#define MAX17048_SOC 0x04               // R - 16-bit state of charge (SOC)
#define MAX17048_MODE 0x06              // W - Sends special commands to IC
#define MAX17048_VERSION 0x08           // R - Returns IC version
#define MAX17048_HIBRT 0x0A             // R/W - Thresholds for entering hibernate
#define MAX17048_CONFIG 0x0C            // R/W - Battery compensation (default 0x971C)
#define MAX17048_CVALRT 0x14            // R/W - Configures vcell range to generate alerts (default 0x00FF)
#define MAX17048_CRATE 0x16             // R - Charge rate 0.208%/hr
#define MAX17048_VRESET_ID 0x18         // R/W - Reset voltage and ID (default 0x96__)
#define MAX17048_STATUS 0x1A            // R/W - Status of ID (default 0x01__)
#define MAX17048_COMMAND 0xFE           // W - Sends special comands to IC
#define MAX17048_CONFIG_ALERT (1 << 5)
#define MAX17048_CONFIG_ALSC 0x0040
#define MAX17048_CONFIG_SLEEP (1 << 7)
#define MAX17048_COMMAND_POR 0x5400
#define MAX17048_MODE_ENSLEEP 0x2000    // W - _Enables_ sleep mode (the SLEEP bit in the CONFIG reg engages sleep)
#define MAX17048_MODE_HIBSTAT 0x1000    // R - indicates when the IC is in hibernate mode
#define MAX17048_MODE_QUICKSTART 0x4000 // this also clears the EnSleep bit
#define MAX17048_HIBRT_ENHIB 0xFFFF     // always use hibernate mode
#define MAX17048_HIBRT_DISHIB 0x0000    // disable hibernate mode
#define MAX17048_STATUS_RI (1 << 0)     // Assumes the MSB has been shifted >> 8
#define MAX17048_STATUS_VH (1 << 1)     // Assumes the MSB has been shifted >> 8
#define MAX17048_STATUS_VL (1 << 2)     // Assumes the MSB has been shifted >> 8
#define MAX17048_STATUS_VR (1 << 3)     // Assumes the MSB has been shifted >> 8
#define MAX17048_STATUS_HD (1 << 4)     // Assumes the MSB has been shifted >> 8
#define MAX17048_STATUS_SC (1 << 5)     // Assumes the MSB has been shifted >> 8
#define MAX17048_STATUS_EnVR (1 << 14)  // ** Unshifted **
#define MAX17048_GENERIC_ERROR 5
#define MAX17048_ADDRESS 0x36

const int SDA_pin_default = 21;         // Default value for microWatt
const int SCL_pin_default = 22;         // Default value for microWatt


// MAX17048
class charger {
  public:

    // These two redirect to microWatt class
    uint8_t setI2Cpins(const int SDA_pin = SDA_pin_default, const int SCL_pin = SCL_pin_default);
    void printI2Cpins();

    void begin() { isConnected(); }

    // Returns true if device is present
    bool isConnected(void) { return ((chargerVersion() & (1 << 4)) != 0); }

    // quickStart() - Restarts the MAX17048 to allow it to re-"guess" the
    // parameters that go into its SoC algorithms. Calling this in your setup()
    // usually results in more accurate SoC readings.
    // Output: 0 on success, positive integer on fail.
    uint8_t quickStart() { return write16(MAX17048_MODE_QUICKSTART, MAX17048_MODE); }

    // battVoltage() - Get the MAX17048's voltage reading.
    // Output: floating point value between 0-5V in 1.25mV increments.
    float battVoltage();

    // battSOC() - Get the MAX17048's state-of-charge (SOC) reading, as calculated
    // by the IC's "ModelGauge" algorithm.
    // The first update is available approximately 1s after POR of the IC.
    // Output: floating point value between 0-100, representing a percentage of
    // full charge.
    float battSOC();

    // chargerVersion() - Get the MAX17048's production version number.
    // Output: 3 on success
    uint16_t chargerVersion() { return read16(MAX17048_VERSION); }

    // getThreshold() - Get the MAX17048's current percentage threshold that will
    // trigger an alert.
    // Output: An integer value between 1 and 32, representing a % that will
    // trigger an alert interrupt.
    uint8_t getThreshold();

    // setThreshold([percent]) - Set the MAX17048's percentage threshold that will
    // trigger an alert.
    // Input: [percent] - Percentage value that will trigger an alert interrupt.
    // Any value between 1 and 32 is valid. Default value is 0x1C == 4%
    // Output: 0 on success, positive integer on fail.
    uint8_t setThreshold(uint8_t percent = 4);

    // sleep() - Set the MAX17048 into sleep mode.
    // Output: 0 on success, positive integer on fail.
    // In sleep mode, the IC halts all operations, reducing current
    // consumption to below 1μA. After exiting sleep mode,
    // the IC continues normal operation. In sleep mode, the
    // IC does not detect self-discharge. If the battery changes
    // state while the IC sleeps, the IC cannot detect it, causing
    // SOC error. Wake up the IC before charging or discharging.
    uint8_t sleep();

    // wake() - Wake the MAX17048 up from sleep.
    // Output: 0 on success, positive integer on fail.
    uint8_t wake();

    // reset() - Issue a Power-on-reset command to the MAX17048. This function
    // will reset every register in the MAX17048 to its default value.
    // Output: Positive integer on success, 0 on fail.
    uint8_t reset() { return write16(MAX17048_COMMAND_POR, MAX17048_COMMAND); }

    // getConfigRegister() - Read the 16-bit value of the CONFIG Register.
    // Output: 16-bit integer value representing the msb and lsb bytes of the
    // CONFIG register.
    uint16_t getConfigRegister();

    // getCompensation() - Get the ModelGauge compensation value - an obscure
    // 8-bit value set to 0x97 by default.
    // Output: 8-bit value read from the CONFIG register's MSB.
    uint8_t getCompensation();

    // setCompensation([newCompensation]) - Set the 8-bit compensation value. This
    // is an obscure 8-bit value that has some effect on Maxim's ModelGauge
    // algorithm. The POR value of RCOMP is 0x97.
    // From the datasheet: "Contact Maxim for instructions for optimization."
    // For best performance, the host microcontroller must measure
    // battery temperature periodically, and compensate
    // the RCOMP ModelGauge parameter accordingly, at least
    // once per minute. Each custom model defines constants
    // RCOMP0 (default is 0x97), TempCoUp (default is -0.5),
    // and TempCoDown (default is -5.0). To calculate the new
    // value of CONFIG.RCOMP:
    // // T is battery temperature (degrees Celsius)
    // if (T > 20) {
    // RCOMP = RCOMP0 + (T - 20) x TempCoUp;
    // }
    // else {
    // RCOMP = RCOMP0 + (T - 20) x TempCoDown;
    // }
    // Input: [newCompensation] - Should be a value between 0-255.
    // Output: 0 on success, positive integer on fail.
    uint8_t setCompensation(uint8_t newCompensation = 0x97);

    // chargerID() - Returns 8-bit OTP bits set at factory. Can be used to
    // 'to distinguish multiple cell types in production'.
    // Writes to these bits are ignored.
    uint8_t chargerID(void);

    // setResetVoltage([threshold]) - Set the 7-bit VRESET value.
    // A 7-bit value that controls the comparator for detecting when
    // a battery is detached and re-connected. 40mV per bit. Default is 3.0V.
    // For captive batteries, set to 2.5V. For
    // removable batteries, set to at least 300mV below the
    // application’s empty voltage, according to the desired
    // reset threshold for your application.
    // Input: [threshold] - Should be a value between 0-127.
    // Output: 0 on success, positive integer on fail.
    uint8_t setResetVoltage(uint8_t threshold = (0x96 >> 1));
    uint8_t setResetVoltage(float threshold = 3.0); // Helper function: set threshold in Volts

    // getResetVoltage() - Get the 7-bit VRESET value
    // Output: 7-bit value read from the VRESET/ID register's MSB.
    // Returned value is 40mV per bit.
    uint8_t getResetVoltage(void);

    // enableComparator() - Set bit in VRESET/ID reg
    // Comparator is enabled by default. (Re)enable the analog comparator, uses 0.5uA.
    // Output: 0 on success, positive integer on fail.
    uint8_t enableComparator(void);

    // disableComparator() - Clear bit in VRESET/ID reg
    // Disable the analog comparator, saves 0.5uA in hibernate mode.
    // Output: 0 on success, positive integer on fail.
    uint8_t disableComparator(void);

    // getChangeRate() - Get rate of change per hour in %
    // Output: (signed) Float (that is the 0.208% * CRATE register value)
    // A positive rate is charging, negative is discharge.
    float getChangeRate();

    // getStatus() - Get the 7 bits of status register
    // Output: 7-bits indicating various alerts
    uint8_t getStatus();

    // Various helper functions to check bits in status register
    //  INPUT: [clear] - If [clear] is true, the alert flag will be cleared if it
    //  was set.
    bool isReset(bool clear = false);       // True after POR
    bool isVoltageHigh(bool clear = false); // True when VCELL goes above VALRTMAX (see setVALRTMax)
    bool isVoltageLow(bool clear = false);  // True when VCELL goes below VALRTMIN (see setVALRTMin)
    bool isVoltageReset(bool clear = false);
    bool isLow(bool clear = false);    // True when SOC crosses the value in ATHD (see setThreshold)
    bool isChange(bool clear = false); // True when SOC changes by at least 1% and SOCAlert is enabled

    // getAlert([clear]) - Check if the MAX1704X's ALRT alert interrupt has been
    // triggered.
    // INPUT: [clear] - If [clear] is true, the alert flag will be cleared if it
    // was set.
    // OUTPUT: Returns 1 if interrupt is/was triggered, 0 if not.
    uint8_t getAlert(bool clear = false);

    // clearAlert() - Clear the MAX1704X's ALRT alert flag.
    // Output: 0 on success, positive integer on fail.
    uint8_t clearAlert();

    // enableSOCAlert() - Enable the SOC change alert
    // Returns true if the SOC change alert was enabled successfully
    bool enableSOCAlert();

    // disableSOCAlert() - Disable the SOC change alert
    // Returns true if the SOC change alert was disabled successfully
    bool disableSOCAlert();

    // Enable or Disable MAX17048/49 VRESET Alert:
    //  EnVr (enable voltage reset alert) when set to 1 asserts
    //  the ALRT pin when a voltage-reset event occurs under
    //  the conditions described by the VRESET/ ID register.
    // enableAlert() - Set ENvR bit in STATUS register 0x1A
    // Output: 0 on success, positive integer on fail.
    uint8_t enableAlert();
    // disableAlert() - Clear the ENvR bit in STATUS register 0x1A
    // Output: 0 on success, positive integer on fail.
    uint8_t disableAlert();

    // Read and return VALRT Maximum threshold
    // LSb = 20mV
    uint8_t getVALRTMax();

    // Read and return VALRT Minimum threshold
    // LSb = 20mV
    uint8_t getVALRTMin();

    // Set VALRT Maximum threshold
    // Output: 0 on success, positive integer on fail.
    // Note: this sets the threshold voltage _per cell_ (MAX17049 monitors two cells)
    uint8_t setVALRTMax(uint8_t threshold = 0xFF); // LSb = 20mV
    uint8_t setVALRTMax(float threshold = 5.1);    // threshold is defined in Volts

    // Set VALRT Minimum threshold
    // Output: 0 on success, positive integer on fail.
    // Note: this sets the threshold voltage _per cell_ (MAX17049 monitors two cells)
    uint8_t setVALRTMin(uint8_t threshold = 0x00); // LSb = 20mV
    uint8_t setVALRTMin(float threshold = 0.0);    // threshold is defined in Volts

    // Read and return Hibernate Status flag
    bool isHibernating();

    // Read and return HIBRT Active Threshold
    // LSb = 1.25mV
    uint8_t getHIBRTActThr();

    // Set HIBRT Active Threshold
    // Output: 0 on success, positive integer on fail.
    uint8_t setHIBRTActThr(uint8_t threshold); // LSb = 1.25mV
    uint8_t setHIBRTActThr(float threshold);   // Helper function: set threshold in Volts

    // Read and return HIBRT Hibernate Threshold
    // LSb = 0.208%/hr
    uint8_t getHIBRTHibThr();

    // Set HIBRT Hibernate Threshold
    // Output: 0 on success, positive integer on fail.
    uint8_t setHIBRTHibThr(uint8_t threshold); // LSb = 0.208%/hr
    uint8_t setHIBRTHibThr(float threshold);   // Helper function: set threshold in percent

    uint8_t enableHibernate() { return write16(MAX17048_HIBRT_ENHIB, MAX17048_HIBRT); }
    uint8_t disableHibernate() { return write16(MAX17048_HIBRT_DISHIB, MAX17048_HIBRT); }

    // Lower level functions but exposed incase user wants them

    // write16([data], [address]) - Write 16 bits to the requested address. After
    // writing the address to be written, two sequential 8-bit writes will occur.
    // the msb is written first, then lsb.
    // Input: [data] - A 16-bit integer to be written to the device.
    //        [address] - An 8-bit address to be written to.
    // Output: 0 on success, positive integer on fail.
    uint8_t write16(uint16_t data, uint8_t address);

    // read16([address]) - Read 16-bits from the requested address of a device.
    // Input: [address] - An 8-bit address to be read from.
    // Output: A 16-bit value read from the device's address will be returned.
    uint16_t read16(uint8_t address);

  private:
    // Clear the specified bit(s) in status register
    // This requires the bits in mask to be correctly aligned.
    // MAX1704x_STATUS_RI etc. will need to be shifted left by 8 bits to become aligned.
    // Output: 0 on success, positive integer on fail.
    uint8_t clearStatusRegBits(uint16_t mask);
};



class microWatt {
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
