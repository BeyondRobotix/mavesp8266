# MavESP8266
## ESP8266 WiFi Access Point and MavLink Bridge

This is very preliminary work. It has been tested only on a [NodeMCU v2 Dev Kit](http://www.seeedstudio.com/depot/NodeMCU-v2-Lua-based-ESP8266-development-kit-p-2415.html) (waiting for a 3.3v FTDI so I can flash to the ESP-01 board included with the [PixRacer](https://pixhawk.org/modules/pixracer))

The build enviroment is based on [PlatformIO](http://platformio.org). Follow the instructions found here: http://platformio.org/#!/get-started (only tested on Mac OS) for installing it but skip the ```platform init``` step as this has already been done, modified and it is included in this repo. In summary:

```
sudo pip install -U pip setuptools
sudo pip install -U platformio
git clone --recursive mavesp8266.git
cd mavesp8266
platformio run
```

:point_right: I don't like installing Python modules as root so I used ```pip install -U --user platformio```. That gave me some *bad egg* issues with the SCons package, a dependency of PlatformIO. I used ```brew install scons``` instead before running ```pip install -U --user platformio```.

When you run ```platformio run``` for the first time, it will download the toolchains and all necessary libraries automatically.

### Useful commands:

```platformio run``` - process/build all targets
```platformio run -e esp12e``` - process/build just the ESP12e target (the NodeMcu v2)
```platformio run -e esp12e -t upload``` - upload firmware to embedded board
```platformio run -t clean``` - clean project (remove compiled files)

The resulting image(s) can be found in the directory ```.pioenvs``` created during the build process.

### MavLink Submodule

The ```git clone --recursive``` above not only cloned the MavESP8266 repository but it also installed the dependent [MavLink](https://github.com/mavlink/c_library) sub-module. To upated the module, use the command:

```git submodule update```

