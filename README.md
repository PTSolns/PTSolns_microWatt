# PTSolns ESP32 microWatt Support Library (mSL)
Library for PTSolns _ESP32 microWatt_

The mSL is designed to help the user program the _ESP32 microWatt_ easily by providing intuitive functions and reducing complexity.

Various examples are included in this package. For comments or questions please [Contact Us](https://ptsolns.com/pages/contact).


# microWatt Family
The microWatt Family has the _ESP32 microWatt_ at its core, with various add-on stacks. The product line includes:
- _[ESP32 microWatt](https://ptsolns.com/products/esp32-microwatt)_
  -- An ESP32-based development board.
- _[microWatt Proto](https://ptsolns.com/products/microwatt-proto)_
  -- A stackable fully customizable prototyping board.  
- _microWatt Charger_
  -- A stackable board to power the _ESP32 microWatt_, with LiPo battery management, PV and USB power inputs.
- _microWatt LoRa SX1276 915MHz_
  -- A stackable board to add LoRa communication to the _ESP microWatt_.



# Install the CH340 Driver
In order for the _ESP32 microWatt_ to be programmed, your computer will need the CH340 driver. If you try to program the _ESP32 microWatt_ and it doesn't work, try installing the driver first. [We've made a video on this!](https://youtu.be/UUQ84VKg3oM?si=tP0sAfqpZ2siR3AG)

# Install the ESP32 Board in Arduino IDE
Add the ESP32 Board Manager URL in Arduino IDE
- Open the Arduino IDE and navigate to File > Preferences.
- In the "Additional Board Manager URLs" field, enter the following URL:
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
- If another URL is already present, separate multiple URLs using commas.

Install the ESP32 Board Package
- In the Arduino IDE, navigate to Tools > Board > Board Manager…
-	In the search bar, type "ESP32 by Espressif Systems".
-	Select and install the ESP32 board package.

NOTE on compatbility!
- If you are using ESP32 board 2.x, then use mSL v1.1.2.
- If you are using ESP32 board 3.x, then use latest version of mSL.


# Tips
- If Serial.begin(baud rate) has not been initiated before the BEGIN call, then the mSL will do so using baud rate Serial.begin(115200). If nothing prints to Serial monitor, ensure baud rate is set to 115200.
- Make sure to install the CH340 driver (see above).
