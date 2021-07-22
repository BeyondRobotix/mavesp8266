# MavEspx2

## Current Binary

* Release Candidate Version 1.0.0
Download the current version (MAVLink V2) from here: [Firmware version 1.0.0] (https://github.com/Xelack/mavespx2/releases/tag/RC-V1.0.0)

Important Note : Please pay attention to select the correct firmware for your device. If you have any doubts don't take the risk and ask me.

 * mavespx2-esp01-x_x_x.bin : for board based on module ESP8266 with 512kb Flash (80kb ram).
 * mavespx2-esp12e-x_x_x.bin : for board based on module ESP8266 with 4mb Flash (80k ram), the major part of the market.
 * mavespx2-pw_link-x_x_x.bin : only for CUAV pw_link board (based on ESP8266EX chip).
 * mavespx2-esp32dev-x_x_x.zip : for board based on module ESP32 with 4mb Flash (zip file including partition file and firmware).
 * mavespx2-wemos_d1_mini32-1_0_0.zip : for D1 board based on module ESP32 with 4mb Flash (zip file including partition file and firmware).


## WiFi Access Point and MavLink Bridge

This is a fork of MAVEPS8266 of "dogmaphobic" port to ESP32 using a two [MINI D1 ESP32](https://www.az-delivery.de/fr/products/esp32-d1-mini) for debugging and test (on ground and on air mounted on Hexa with PIXHACK V3 (Arducopter V4.0.3)). Even though the ESP32 port showed better efficiency than the ESP8266, I decided to keep the ESP8266 code working, and finally I'm improving all of them.

For now I'don't have ESP8266 module (NodeMCU like) so I use [PW_LINK](http://doc.cuav.net/data-transmission/pw-link/en/) to test.
On this PW_LINK all it's working fine but I don't know why, I don't want boot at the first power-on, I need to unplug dc and replug it...(I'll have to get out my old cathode ray oscilloscope ^^ ).

The build enviroment is based on [PlatformIO](http://platformio.org). Follow the instructions found here: http://platformio.org/#!/get-started (only tested on Windows 10) for installing it but skip the ```platform init``` step as this has already been done, modified and it is included in this repository. In summary:

```
brew install platformio
git clone --recursive https://github.com/Xelack/mavespx2.git
cd mavesp8266
platformio run
```

When you run ```platformio run``` for the first time, it will download the toolchains and all necessary libraries automatically.

### Useful commands:
ESP32:
* ```platformio run -e esp32``` - process/build just the ESP32 target ("must common" ESP32 module with 4mo flash)

ESP8266:
* ```platformio run -e esp12e``` - process/build just the ESP12e target (the NodeMcu v2, Adafruit HUZZAH, etc.)
* ```platformio run -e pw_link``` - process/build just the CUAV PW_LINK target (based on ESP8622EX with 4mo flash)
* ```platformio run -e esp12e -t upload``` - build and upload firmware to embedded board

ALL :
* ```platformio run``` - process/build all targets
* ```platformio run -t clean``` - clean project (remove compiled files)


The resulting image(s) can be found in the directory ```.pio\build\[target]``` created during the build process.

### MavLink Submodule

The ```git clone --recursive``` above not only cloned the MavESP8266 repository but it also installed the dependent [MavLink](https://github.com/mavlink/c_library) sub-module. To upated the module (when needed), use the command:

```git submodule update --init```

### Wiring it up

User level (as well as wiring) instructions can be found [here for px4](https://docs.px4.io/en/telemetry/esp8266_wifi_module.html) and [here for ArduPilot](http://ardupilot.org/copter/docs/common-esp8266-telemetry.html)

For ESP8266:
FC (TX) -> (RX) GPIO03 (MAVESP - define by UART_MAVFC_RX in source code)
FC (RX) <- (TX) GPIO01 (MAVESP - define by UART_MAVFC_TX in source code)

For ESP32:
FC (TX) -> (RX) GPIO16 (MAVESP - define by UART_MAVFC_RX in source code)
FC (RX) <- (TX) GPIO17 (MAVESP - define by UART_MAVFC_TX in source code)

* Important Note: to ensure a RELIABLE and SECURE Communications with your WiFi bridge device, please use UART level shifter high speed (like TXB0102 (2 wires), TXB0104 (4 wires),etc...) between your FC and ESP UARTs port if your FC work in 5v. ALL ESP DEVICES ARE NOT 5v TOLERANT, SO INOPPORTUNE BURN COULD HAPPEN AT ANY TIME!

* Resetting to Defaults: In case you change the parameters and get locked out of the module, all the parameters can be reset by bringing the GPIO pin low.
    For ESP8266: Connect GPIO02 pin to GND pin.
    For ESP32: Connect GPIO22 pin to GND pin.

### Status LED

This feature is availlable for esp32 and PW_LINK with builtin led (GPIO02 is used by default).
For others boards you can add led on a availlabe GPIO but you need set this GPIO in source code and rebuild the firmware (or tell me ^^ ).

Meaning:
* blink at 1hz : On start, waiting wifi client. 
* Stay on : update in progress.
* Blink at 2hz : reboot imminant.

### Actions on button

This feature is availlable for all board (except PW_LINK) with free GPIO (default GPIO : ref to wiring). 
Just Plug a button to GPIO pin and GND pin.

* 2 presses : test (for debug, do nothing).
* 3 presses : request reboot.
* 4 presses : request restore default parameters.
* 5 presses : request factory reset.

* Sup. to 5 presses : reset button action after 2s.

Actions are commit and start only 2 seconds after the last push.
If the delay between first press and second press is superior at 5 seconds cancel the first press.

### MavLink Protocol

The MavESP8266 handles its own set of parameters and commands. Look at the [PARAMETERS](PARAMETERS.md) page for more information.

### HTTP Protocol

There are some preliminary URLs that can be used for checking the WiFi Bridge status as well as updating firmware and changing parameters. [You can find it here.](HTTP.md)
