## MavESP8266
### Mavlink Paramaters and Messages supported by the MavESP8266 Firmware

#### Parameters

The MavESP8266 uses ```MAV_COMP_ID_UDP_BRIDGE``` (240) as its component id. It will process messages sent to this component id or messages sent to ```MAV_COMP_ID_ALL``` only.

##### MAVLINK_MSG_ID_PARAM_REQUEST_LIST

If this message is sent to *All Components* (```MAV_COMP_ID_ALL```), or specifically to the MavESP8266 component ID, it will return its list of parameters. Messages sent to ```MAV_COMP_ID_ALL``` will be forwarded to the UAS as well.

##### MAVLINK_MSG_ID_PARAM_REQUEST_READ

If this message is sent specifically to the MavESP8266 component ID, it will return the requested parameter (either by ID or by Index)

##### MAVLINK_MSG_ID_PARAM_SET

If this message is sent specifically to the MavESP8266 component ID, it will set the new value for the specified parameter. Note that this only sets the value for the current session. It does not write the values to EEPROM. See MAV_CMD_PREFLIGHT_STORAGE below.

#### Available Parameters

| Parameter ID  | Parameter Type | Description |
| ------------- | -------------- | ----------- |
| DEBUG_ENABLED | MAV_PARAM_TYPE_INT8  | Enable Debug Messages (1) |
| SW_VER  | MAV_PARAM_TYPE_UINT32 | Firmware Version (Read Only) |
| UART_BAUDRATE | MAV_PARAM_TYPE_UINT32 | UAS UART Link Baud Rate (default to 921600) |
| WIFI_CHANNEL  | MAV_PARAM_TYPE_UINT32 | AP WiFi Channel (default to 11) |
| WIFI_IPADDRESS | MAV_PARAM_TYPE_UINT32 | Local IP Address (default to 192.168.4.1 when in AP Mode) (Read Only) |
| WIFI_MODE | MAV_PARAM_TYPE_INT8 | WiFi Operating Mode (3) |
| WIFI_PASSWORD1 | MAV_PARAM_TYPE_UINT32 | WiFi AP Password (2) |
| WIFI_PASSWORD2 | MAV_PARAM_TYPE_UINT32 | |
| WIFI_PASSWORD3 | MAV_PARAM_TYPE_UINT32 | |
| WIFI_PASSWORD4 | MAV_PARAM_TYPE_UINT32 | |
| WIFI_PWDSTA1 | MAV_PARAM_TYPE_UINT32 | WiFi STA Password (2) |
| WIFI_PWDSTA2 | MAV_PARAM_TYPE_UINT32 | |
| WIFI_PWDSTA3 | MAV_PARAM_TYPE_UINT32 | |
| WIFI_PWDSTA4 | MAV_PARAM_TYPE_UINT32 | |
| WIFI_SSID1 | MAV_PARAM_TYPE_UINT32 | WiFi AP SSID (2) |
| WIFI_SSID2 | MAV_PARAM_TYPE_UINT32 | |
| WIFI_SSID3 | MAV_PARAM_TYPE_UINT32 | |
| WIFI_SSID4 | MAV_PARAM_TYPE_UINT32 | |
| WIFI_SSIDSTA1 | MAV_PARAM_TYPE_UINT32 | WiFi STA SSID (2) |
| WIFI_SSIDSTA2 | MAV_PARAM_TYPE_UINT32 | |
| WIFI_SSIDSTA3 | MAV_PARAM_TYPE_UINT32 | |
| WIFI_SSIDSTA4 | MAV_PARAM_TYPE_UINT32 | |
| WIFI_IPSTA | MAV_PARAM_TYPE_UINT32 | Wifi STA Static IP Address (4) |
| WIFI_GATEWAYSTA | MAV_PARAM_TYPE_UINT32 | Wifi STA Gateway Address (4) |
| WIFI_SUBNETSTA | MAV_PARAM_TYPE_UINT32 | Wifi STA Subnet Address (4) |
| WIFI_UDP_CPORT | MAV_PARAM_TYPE_UINT16 | Local UDP Port (default to 14555)  |
| WIFI_UDP_HPORT | MAV_PARAM_TYPE_UINT16 | GCS UDP Port (default to 14550) |
| WEB_ACCOUNT1 | MAV_PARAM_TYPE_UINT32 | Web authentication account(default to Pixracer) |
| WEB_ACCOUNT2 | MAV_PARAM_TYPE_UINT32 | |
| WEB_ACCOUNT3 | MAV_PARAM_TYPE_UINT32 | |
| WEB_ACCOUNT4 | MAV_PARAM_TYPE_UINT32 | |
| WEB_PASSWORD1 | MAV_PARAM_TYPE_UINT32 | Web authentication pasword(default to pixracer) |
| WEB_PASSWORD2 | MAV_PARAM_TYPE_UINT32 | |
| WEB_PASSWORD3 | MAV_PARAM_TYPE_UINT32 | |
| WEB_PASSWORD4 | MAV_PARAM_TYPE_UINT32 | |


##### Notes

* (1) If debug is enabled, debug messages are sent using ```MAVLINK_MSG_ID_STATUSTEXT``` with a proper ```MAV_SEVERITY_DEBUG``` type. Other messages types, ```MAV_SEVERITY_NOTICE``` for example, are sent regardless.
* (2) MavLink parameter messages only support a 32-Bit parameter (be it a float, an uint32_t, etc.) In other to fit a 16-character SSID and a 16-character Password, 4 paramaters are used for each. The 32-Bit storage is used to contain 4 bytes for the string.
* (3) The mode defaults to 0. Set to 0 to act as an Access Point. Set to 1 to connect to an existing WiFi network using the STA (Station Mode) SSID and password. When in *Station Mode*, the module will attempt to connect for up to one minute. If after that it cannot connect, it reverts to AP mode.
* (4) Defaults to 0 for an unset address. If either the STA IP, Gateway, or Subnet are set, then all three need to be set for it to work properly.

#### MAVLINK_MSG_ID_COMMAND_LONG

In addition to parameters, MavESP8266 also supports a few commands, which will be handled _**if addressed to its component ID**_:

##### MAV_CMD_PREFLIGHT_STORAGE

* If ```param1``` == 0 It will load all parameters from EEPROM overwriting any changes.
* If ```param1``` == 1 It will save all current parameters to EEPROM.
* If ```param1``` == 2 It will reset all parameters to the original default values. Note that it will not store them to EEPROM. You must request that separately if that's what you want to do.

##### MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN

* If ```param2``` == 1 It will cause the ESP8266 module to reboot. This is necessary if you want parameter changes to take effect. Out of the above, the only parameter that takes effect immediatly upon changing is **DEBUG_ENABLED**. All other values will only take effect at boot time.
