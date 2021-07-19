# MavESP8266

## Current Binary

Download the current version (MAVLink V2) from here: [Firmware version 1.2.2](http://www.grubba.com/mavesp8266/firmware-1.2.2.bin)

Download the legacy version (MAVLink V1) from here: [Firmware version 1.1.1](http://www.grubba.com/mavesp8266/firmware-1.1.1.bin)

## ESP8266 WiFi Access Point and MavLink Bridge

[![Join the chat at https://gitter.im/dogmaphobic/mavesp8266](https://badges.gitter.im/dogmaphobic/mavesp8266.svg)](https://gitter.im/dogmaphobic/mavesp8266?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

This was developed using a [NodeMCU v2 Dev Kit](http://www.seeedstudio.com/depot/NodeMCU-v2-Lua-based-ESP8266-development-kit-p-2415.html) as it conveniently provides a secondary UART for debugging. It has been tested with the ESP-01 shipped with the [PixRacer](https://pixhawk.org/modules/pixracer) and it is stable at 921600 baud.

The build enviroment is based on [PlatformIO](http://platformio.org). Follow the instructions found here: http://platformio.org/#!/get-started (only tested on Mac OS) for installing it but skip the ```platform init``` step as this has already been done, modified and it is included in this repository. In summary:

```
brew install platformio
git clone --recursive https://github.com/dogmaphobic/mavesp8266.git
cd mavesp8266
platformio run
```

When you run ```platformio run``` for the first time, it will download the toolchains and all necessary libraries automatically.

### Useful commands:

* ```platformio run``` - process/build all targets
* ```platformio run -e esp12e``` - process/build just the ESP12e target (the NodeMcu v2, Adafruit HUZZAH, etc.)
* ```platformio run -e esp32``` - process/build just the ESP32 target ("must common" ESP32 module with 4mo flash)
* ```platformio run -e pw_link``` - process/build just the CUAV PW_LINK target (based on ESP8622EX with 4mo flash)
* ```platformio run -e esp12e -t upload``` - build and upload firmware to embedded board
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
    For ESP32: Connect GPIO36 pin to GND pin.

### MavLink Protocol

The MavESP8266 handles its own set of parameters and commands. Look at the [PARAMETERS](PARAMETERS.md) page for more information.

### HTTP Protocol

There are some preliminary URLs that can be used for checking the WiFi Bridge status as well as updating firmware and changing parameters. [You can find it here.](HTTP.md)
