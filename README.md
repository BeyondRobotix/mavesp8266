# MavESP8266

# Project Status

This project is being continued here, follow link for latest releases: [Kahuna WiFi Telemetry Unit](https://github.com/BeyondRobotix/Kahuna)




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

### Useful commands: ESP32-WROOM

* ```platformio run -e espwroom32 -t upload```

### Useful commands: ESP12e

* ```platformio run``` - process/build all targets
* ```platformio run -e esp12e``` - process/build just the ESP12e target (the NodeMcu v2, Adafruit HUZZAH, etc.)
* ```platformio run -e esp12e -t upload``` - build and upload firmware to embedded board
* ```platformio run -t clean``` - clean project (remove compiled files)

### Useful commands: ESP-01
* ```platformio run``` - process/build all targets
* ```platformio run -e esp01_1m``` - process/build just the ESP01 target with 1MB [Amazon](https://www.amazon.com/gp/product/B01EA3UJJ4)
* ```platformio run -e esp01_1m -t upload``` - build and upload firmware to embedded board
* ```platformio run -t clean``` - clean project (remove compiled files)



The resulting image(s) can be found in the directory ```.pioenvs``` created during the build process.

### MavLink Submodule

The ```git clone --recursive``` above not only cloned the MavESP8266 repository but it also installed the dependent [MavLink](https://github.com/mavlink/c_library) sub-module. To upated the module (when needed), use the command:

```git submodule update --init```

### Wiring it up

User level (as well as wiring) instructions can be found [here for px4](https://docs.px4.io/en/telemetry/esp8266_wifi_module.html) and [here for ArduPilot](http://ardupilot.org/copter/docs/common-esp8266-telemetry.html)

* Resetting to Defaults: In case you change the parameters and get locked out of the module, all the parameters can be reset by bringing the GPIO02 pin low (Connect GPIO02 pin to GND pin). 

Get the ESP-01 adapter board here on [Amazon](https://www.amazon.com/gp/product/B07Q17XJ36/); commonly called an "ESP01 programmer", this one has a little rocker switch on the side (UART side for serial TTL debugging by AT commands, PROG for firmware programming) and a yellow pin header which allows you plugin the ESP01 module without any wires. It defaults to 115200 and enumerates under Ubuntu as a ch341-uart converter. 


### MavLink Protocol

The MavESP8266 handles its own set of parameters and commands. Look at the [PARAMETERS](PARAMETERS.md) page for more information.

### HTTP Protocol

There are some preliminary URLs that can be used for checking the WiFi Bridge status as well as updating firmware and changing parameters. [You can find it here.](HTTP.md)
