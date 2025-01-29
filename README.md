# PTSolns ESP32 microWatt Support Library (mSL)
Library for PTSolns _ESP32 microWatt_

The mSL is designed to help the user program the _ESP32 microWatt_ easily by providing intuitive functions and reducing complexity.

Various examples are included in this package. For comments or questions please contact us at contact@PTSolns.com.


# microWatt Family
The microWatt Family has the _ESP32 microWatt_ at its core, with various add-on stacks. The product line includes:
- _ESP32 microWatt_
  -- An ESP32-based development board.
- _microWatt Charger_
  -- A stackable board to power the _ESP32 microWatt_, with LiPo battery management, PV and USB power inputs.
- _microWatt LoRa SX1276 915MHz_
  -- A stackable board to add LoRa communication to the _ESP microWatt_.
- _microWatt Proto_
  -- A stackable fully customizable prototyping board.


# Install the CH340 Driver
In order for the _ESP32 microWatt_ to be programmed, your computer will need the CH340 driver. If you try to program the _ESP32 microWatt_ and it doesn't work, try installing the driver first. [Sparkun has a great tutorial on this!](https://learn.sparkfun.com/tutorials/how-to-install-ch340-drivers/all)


# Tips
- If Serial.begin(baud rate) has not been initiated before the BEGIN call, then the mSL will do so using baud rate Serial.begin(115200). If nothing prints to Serial monitor, ensure baud rate is set to 115200.
- Make sure to install the CH340 driver (see above).
