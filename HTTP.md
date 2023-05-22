## MavESP8266
### MavESP8266 Web Interface

#### API

The idea is to have some sort of web interface so you can check the status and change parameters. While that is not done, there are however, a few URLs that can be used. Obviously, you need to be connected to the WiFi Bridge's Access Point for these URLs to work.

##### Get Parameters

http://192.168.4.1/getparameters

This will show the current parameters and their values.

##### Get Status

http://192.168.4.1/getstatus

This will show the current comm link status.

##### Set Parameters

http://192.168.4.1/setparameters?key=value&key=value

This will allow you to set any parameter to the specified value. Once set, the values are stored in non volatile memory (EEPROM) but will only take effect once you reboot it. Use this with caution as you may lock yourself out. For instance, if you change the AP password and don't remember later, currently you must either trigger the reset pin (see README) or reflash the module in order to revert the parameters. Also note that there is no validation done to the values entered. 

There are the supported parameters:

| Key  | Default Value | Description | Example |
| ------------- | -------------- | -------------- | -------------- |
| baud  | 57600 | UAS UART Link Baud Rate | http://192.168.4.1/setparameters?baud=57600 |
| channel | 11  | AP WiFi Channel | http://192.168.4.1/setparameters?channel=11 |
| cport | 14555  | Local UDP Port | http://192.168.4.1/setparameters?cport=14555 |
| debug | 0  | Enable Debug Messages | http://192.168.4.1/setparameters?debug=0 |
| hport | 14550  | GCS UDP Port | http://192.168.4.1/setparameters?hport=14550 |
| mode | 0  | Set to AP Mode (0) or Station Mode (1) | http://192.168.4.1/setparameters?mode=1 |
| pwd | robotix1  | WiFi AP Password | http://192.168.4.1/setparameters?pwd=robotix1 |
| pwdsta | robotix1  | WiFi STA Password | http://192.168.4.1/setparameters?pwdsta=robotix1 |
| reboot | 0  | Reboot the WiFi Bridge | http://192.168.4.1/setparameters?reboot=1 |
| ssid | Beyond1  | WiFi AP SSID | http://192.168.4.1/setparameters?ssid=Beyond1 |
| ssidsta | Beyond1  | WiFi STA SSID | http://192.168.4.1/setparameters?ssidsta=Beyond1 |
| ipsta | 0.0.0.0 | Wifi STA Static IP | http://192.168.4.1/setparameters?ipsta=192.168.4.2 |
| gatewaysta | 0.0.0.0 | Wifi STA Gateway | http://192.168.4.1/setparameters?gatewaysta=192.168.4.1 |
| subnetsta | 0.0.0.0 | Wifi STA Subnet | http://192.168.4.1/setparameters?subnetsta=255.255.255.0 |

You can combine any number of parameters into one request. For example:

http://192.168.4.1/setparameters?baud=57600&channel=9&reboot=1

To connect to a typical existing network with a static ip:

http://192.168.4.1/setparameters?mode=1&ssidsta=networkname&pwdsta=thepassword&ipsta=192.168.1.123&gatewaysta=192.168.1.1&subnetsta=255.255.255.0

##### Upload New Firmware

http://192.168.4.1/update

This will bring up a simple page with a button to allow you to pick the new ```firmware.bin``` file to upload and a button to upload it to the WiFi Bridge. Again, caution when using this as there is no validation yet.
