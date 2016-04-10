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
 * @file main.cpp
 * ESP8266 Wifi AP, MavLink UART/UDP Bridge
 *
 * @author Gus Grubba <mavlink@grubba.com>
 */

#include "mavesp8266.h"
#include "mavesp8266_parameters.h"
#include "mavesp8266_gcs.h"
#include "mavesp8266_vehicle.h"
#include "mavesp8266_httpd.h"

#define GPIO02  2

#ifndef DEBUG_PRINT
uint8_t                 reset_state;
#endif

//---------------------------------------------------------------------------------
//-- HTTP Update Status
class MavESP8266UpdateImp : public MavESP8266Update {
public:
    MavESP8266UpdateImp ()
        : _isUpdating(false)
    {

    }
    void updateStarted  ()
    {
        _isUpdating = true;
    }
    void updateCompleted()
    {
        //-- TODO
    }
    void updateError    ()
    {
        //-- TODO
    }
    bool isUpdating     () { return _isUpdating; }
private:
    bool _isUpdating;
};



//-- Singletons
IPAddress               localIP;
MavESP8266Parameters    Parameters;
MavESP8266GCS           GCS;
MavESP8266Vehicle       Vehicle;
MavESP8266Httpd         updateServer;
MavESP8266UpdateImp     updateStatus;
MavESP8266Log           Logger;

//---------------------------------------------------------------------------------
//-- Accessors
class MavESP8266WorldImp : public MavESP8266World {
public:
    MavESP8266Parameters*   getParameters   () { return &Parameters;    }
    MavESP8266Vehicle*      getVehicle      () { return &Vehicle;       }
    MavESP8266GCS*          getGCS          () { return &GCS;           }
    MavESP8266Log*          getLogger       () { return &Logger;        }
};

MavESP8266WorldImp      World;

MavESP8266World* getWorld()
{
    return &World;
}


//---------------------------------------------------------------------------------
//-- Wait for a DHCPD client
void wait_for_client() {
    DEBUG_LOG("Waiting for a client...\n");
    int wcount = 0;
    uint8 client_count = wifi_softap_get_station_num();
    while (!client_count) {
        #ifdef ENABLE_DEBUG
        Serial1.print(".");
        if(++wcount > 80) {
            wcount = 0;
            Serial1.println();
        }
        #endif
        delay(1000);
        client_count = wifi_softap_get_station_num();
    }
    DEBUG_LOG("Got %d client(s)\n", client_count);
}

//---------------------------------------------------------------------------------
//-- Check for reset pin
void check_reset() {
#ifndef ENABLE_DEBUG
    //-- Test for "Reset To Factory"
    /* Needs testing
    int reset = digitalRead(GPIO02);
    if(reset != reset_state) {
        resetToDefaults();
        saveAllToEeprom();
        wifi_reboot();
    }
    */
#endif
}

//---------------------------------------------------------------------------------
//-- Set things up
void setup() {
    delay(1000);
    Logger.begin(2048);
#ifndef ENABLE_DEBUG
    //-- Initialized GPIO02 (Used for "Reset To Factory")
    //   We only use it for non bebug because GPIO02 is used as a serial
    //   pin (TX) when debugging.
    pinMode(GPIO02, INPUT_PULLUP);
    reset_state = digitalRead(GPIO02);
#endif
    DEBUG_LOG("\nConfiguring access point...\n");
    DEBUG_LOG("Free Sketch Space: %u\n", ESP.getFreeSketchSpace());
    Parameters.begin();
    //-- Start AP
    WiFi.mode(WIFI_AP);
    WiFi.encryptionType(AUTH_WPA2_PSK);
    WiFi.softAP(Parameters.getWifiSsid(), Parameters.getWifiPassword(), Parameters.getWifiChannel());
    localIP = WiFi.softAPIP();
    //-- I'm getting bogus IP from the DHCP server. Broadcasting for now.
    IPAddress gcs_ip(localIP);
    gcs_ip[3] = 255;
    DEBUG_LOG("Waiting for DHCPD...\n");
    dhcp_status dstat = wifi_station_dhcpc_status();
    while (dstat != DHCP_STARTED) {
        #ifdef ENABLE_DEBUG
        Serial1.print(".");
        #endif
        delay(500);
        dstat = wifi_station_dhcpc_status();
    }
    wait_for_client();
    DEBUG_LOG("Start WiFi Bridge\n");
    //-- Initialize Comm Links
    GCS.begin((MavESP8266Bridge*)&Vehicle, gcs_ip);
    Vehicle.begin((MavESP8266Bridge*)&GCS);
    //-- Initialize Update Server
    updateServer.begin(&updateStatus);
}

//---------------------------------------------------------------------------------
//-- Main Loop
void loop() {
    if(!updateStatus.isUpdating()) {
        GCS.readMessage();
        delay(0);
        Vehicle.readMessage();
        delay(0);
        //check_reset();
        //delay(0);
    }
    updateServer.checkUpdates();
}
