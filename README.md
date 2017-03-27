# MavESP8266

## Current Binary

Download the current version from here: [Firmware version 1.1.1](http://www.grubba.com/mavesp8266/firmware-1.1.1.bin)

## ESP8266 WiFi Access Point and MavLink Bridge

[![Join the chat at https://gitter.im/dogmaphobic/mavesp8266](https://badges.gitter.im/dogmaphobic/mavesp8266.svg)](https://gitter.im/dogmaphobic/mavesp8266?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

This was developed using a [NodeMCU v2 Dev Kit](http://www.seeedstudio.com/depot/NodeMCU-v2-Lua-based-ESP8266-development-kit-p-2415.html) as it conveniently provides a secondary UART for debugging. It has been tested with the ESP-01 shipped with the [PixRacer](https://pixhawk.org/modules/pixracer) and it is stable at 921600 baud.

The build enviroment is based on [PlatformIO](http://platformio.org). Follow the instructions found here: http://platformio.org/#!/get-started (only tested on Mac OS) for installing it but skip the ```platform init``` step as this has already been done, modified and it is included in this repository. In summary:

```
sudo pip install -U pip setuptools
sudo pip install -U platformio
git clone --recursive https://github.com/dogmaphobic/mavesp8266.git
cd mavesp8266
platformio run
```

When you run ```platformio run``` for the first time, it will download the toolchains and all necessary libraries automatically.

### Useful commands:

* ```platformio run``` - process/build all targets
* ```platformio run -e esp12e``` - process/build just the ESP12e target (the NodeMcu v2)
* ```platformio run -e esp12e -t upload``` - build and upload firmware to embedded board
* ```platformio run -t clean``` - clean project (remove compiled files)

The resulting image(s) can be found in the directory ```.pioenvs``` created during the build process.

### MavLink Submodule

The ```git clone --recursive``` above not only cloned the MavESP8266 repository but it also installed the dependent [MavLink](https://github.com/mavlink/c_library) sub-module. To upated the module (when needed), use the command:

```git submodule update```

### Wiring it up

User level (as well as wiring) instructions can be found here: https://pixhawk.org/peripherals/8266

* Resetting to Defaults: In case you change the parameters and get locked out of the module, all the parameters can be reset by bringing the GPIO02 pin low (Connect GPIO02 pin to GND pin). 

### MavLink Protocol

The MavESP8266 handles its own set of parameters and commands. Look at the [PARAMETERS](PARAMETERS.md) page for more information.

### HTTP Protocol

There are some preliminary URLs that can be used for checking the WiFi Bridge status as well as updating firmware and changing parameters. [You can find it here.](HTTP.md)
