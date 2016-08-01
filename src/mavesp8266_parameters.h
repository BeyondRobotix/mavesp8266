/****************************************************************************
 *
 * Copyright (c) 2015, 2016 Gus Grubba. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mavesp8266_parameters.h
 * ESP8266 Wifi AP, MavLink UART/UDP Bridge
 *
 * @author Gus Grubba <mavlink@grubba.com>
 */

#ifndef MAVESP8266_PARAMETERS_H
#define MAVESP8266_PARAMETERS_H

#define WIFI_MODE_AP 0
#define WIFI_MODE_STA 1

//-- Constants
#define DEFAULT_WIFI_MODE       WIFI_MODE_AP
#define DEFAULT_UART_SPEED      921600
#define DEFAULT_WIFI_CHANNEL    11
#define DEFAULT_UDP_HPORT       14550
#define DEFAULT_UDP_CPORT       14555

struct stMavEspParameters {
    char        id[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN];
    void*       value;
    uint16_t    index;
    uint8_t     length;
    uint8_t     type;
    bool        readOnly;
};

class MavESP8266Parameters {
public:
    MavESP8266Parameters();

    enum {
        ID_FWVER = 0,
        ID_DEBUG,
        ID_MODE,
        ID_CHANNEL,
        ID_HPORT,
        ID_CPORT,
        ID_IPADDRESS,
        ID_SSID1,
        ID_SSID2,
        ID_SSID3,
        ID_SSID4,
        ID_PASS1,
        ID_PASS2,
        ID_PASS3,
        ID_PASS4,
        ID_SSIDSTA1,
        ID_SSIDSTA2,
        ID_SSIDSTA3,
        ID_SSIDSTA4,
        ID_PASSSTA1,
        ID_PASSSTA2,
        ID_PASSSTA3,
        ID_PASSSTA4,
        ID_IPSTA,
        ID_GATEWAYSTA,
        ID_SUBNETSTA,
        ID_UART,
        ID_COUNT
    };

    void        begin                       ();
    void        loadAllFromEeprom           ();
    uint32_t    paramHashCheck              ();
    void        resetToDefaults             ();
    void        saveAllToEeprom             ();

    uint32_t    getSwVersion                ();
    int8_t      getDebugEnabled             ();
    int8_t      getWifiMode                 ();
    uint32_t    getWifiChannel              ();
    uint16_t    getWifiUdpHport             ();
    uint16_t    getWifiUdpCport             ();
    char*       getWifiSsid                 ();
    char*       getWifiPassword             ();
    char*       getWifiStaSsid              ();
    char*       getWifiStaPassword          ();
    uint32_t    getWifiStaIP                ();
    uint32_t    getWifiStaGateway           ();
    uint32_t    getWifiStaSubnet            ();
    uint32_t    getUartBaudRate             ();

    void        setDebugEnabled             (int8_t enabled);
    void        setWifiMode                 (int8_t mode);
    void        setWifiChannel              (uint32_t channel);
    void        setWifiUdpHport             (uint16_t port);
    void        setWifiUdpCport             (uint16_t port);
    void        setWifiSsid                 (const char* ssid);
    void        setWifiPassword             (const char* pwd);
    void        setWifiStaSsid              (const char* ssid);
    void        setWifiStaPassword          (const char* pwd);
    void        setWifiStaIP                (uint32_t addr);
    void        setWifiStaGateway           (uint32_t addr);
    void        setWifiStaSubnet            (uint32_t addr);
    void        setUartBaudRate             (uint32_t baud);
    void        setLocalIPAddress           (uint32_t ipAddress);

    stMavEspParameters* getAt               (int index);

private:
    uint32_t    _crc32part                  (uint8_t* value, uint32_t len, uint32_t crc);
    uint32_t    _getEepromCrc               ();
    void        _initEeprom                 ();

private:

};

#endif
