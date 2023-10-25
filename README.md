# PTSolns microWatt Support Library (mSL)
Library for PTSolns microWatt

Although it is recommended that novice users install the microWatt Support Library (mSL), this library is *NOT* a requirement, or a must, to use the microWatt microcontroller. The mSL can be used optionally to assist with various tasks or as a general help.

Various examples are included in this package. For any comments or questions please contact us at contact@PTSolns.com.


# Available Commands
- .begin(optional arguments)<br />
  -> Arguments can be left blank (uses defaults), or input (int LED = 13, int number_of_blink = 4, int time_between_blink = 50) which are passed to .blink() (see below).<br />
  -> This starts the mSL with some useful printouts.<br />
  -> Starts Serial if it was not initiated prior.<br />
  -> Sets up .blink() (see below).<br />
  -> function returns int mLS_code (see below).

- .blink(optional arguments)<br />
  -> Arguments can be left blank (uses defaults), or input (int LED = 13, int number_of_blink = 4, int time_between_blink = 50).<br />
  -> Makes LED blink ON and OFF number_of_blink times with a equal duration of time_between_blink ms.<br />

- .printPinout()<br />
  -> No arguments to pass.<br />
  -> Prints the pinout of the microWatt<br />
  
- .setI2Cpins(optional arguments)<br />
  -> Arguments can be left blank (uses defaults), or input (int SDA_pin = 21, int SCL_pin = 22).<br />
  -> Set I2C pins.<br />
  -> Initiates Wire.begin(SDA_pin, SCL_pin).<br />
  -> function returns int mLS_code (see below).
  
- .printI2Cpins()<br />
  -> No arguments to pass.<br />
  -> Print out the I2C pins (whether default or specified previously using .setI2Cpins()).<br />
  
- .printSPIpins()<br />
  -> No arguments to pass.<br />
  -> Print out the SPI pins.<br />
  
- .I2Cscan(optional arguments)<br />
  -> Arguments can be left blank (uses defaults), or input (int SDA_pin = 21, int SCL_pin = 22).<br />
  -> Performs I2C scan to check for any address of connected peripherals.<br />
  -> If no arguments are supplied, then either default I2C pins are used for the scan, or a previously specified set of pins using .setI2Cpins().<br />
  -> NOTE: if not already previously called, this function will initiate Wire.being().<br />


# mSL_code
An integer return value from various functions.
- = 0 -> Everything is OK
- = 1 -> Relating to Serial. Likely Serial.begin(baud rate) was not called BEFORE any microWatt commands, and hence it was initiated by the microWatt Support Library. Otherwise check baud rate. If microWatt Support Library initiated Serial, it would be on Serial.begin(115200).
- = 2 -> I2C pins have been set manually AND Wire.begin() has been initiated using either the default pins or specified pins using .setI2Cpins().

  
# Tips
- If Serial.begin(baud rate) has not been initiated before the BEGIN call, then the mSL will do so using baud rate Serial.begin(115200). If nothing prints to Serial monitor, ensure baud rate is set to 115200.
- The mSL uses an integer code called mSL_code to keep track on what is happening. Many functions return this value after being called. You can print out the mSL_code and check the reference as to what it means.
