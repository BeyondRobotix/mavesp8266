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
#include <Arduino.h>

#include "mavesp8266.h"
#include "mavesp8266_parameters.h"
#include "mavesp8266_gcs.h"
#include "mavesp8266_vehicle.h"
#include "mavesp8266_httpd.h"
#include "mavesp8266_component.h"
#ifdef ESP32
    #include <ESPmDNS.h>
#else
    #include <ESP8266mDNS.h>
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
MavESP8266Component     Component;
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
    MavESP8266Component*    getComponent    () { return &Component;     }
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
    uint8_t state = LED_OFF;
#ifdef ENABLE_DEBUG
    int wcount = 0;
#endif
#ifdef ESP32  
    uint8_t client_count = WiFi.softAPgetStationNum();
#else
    uint8_t client_count = wifi_softap_get_station_num();
#endif
    while (!client_count) {
        state = (state == LED_ON) ? LED_OFF : LED_ON;
        SET_STATUS_LED(state);
#ifdef ENABLE_DEBUG
        Serial1.print(".");
        if(++wcount > 80) {
            wcount = 0;
            Serial1.println();
        }
#endif
        delay(1000);
#ifdef ESP32
        client_count = WiFi.softAPgetStationNum();
#else
        client_count = wifi_softap_get_station_num();
#endif
    }
    SET_STATUS_LED(LED_OFF);
    DEBUG_LOG("Got %d client(s)\n", client_count);
}

//---------------------------------------------------------------------------------
//-- Reset all parameters whenever the reset gpio pin is active
void reset_interrupt(){
    DEBUG_LOG("Reset to factory\n");
    Parameters.resetToDefaults();
    Parameters.saveAllToEeprom();
#ifndef ESP32
    ESP.reset();
#else
    delay(200); // to be sure of the Eeprom end to write 
    ESP.restart();
#endif
}

//---------------------------------------------------------------------------------
//-- Set things up
void setup() {
#ifdef ESP32
    //downgrade CPU speed to reduce power consumption
    setCpuFrequencyMhz(160);  
#endif
    delay(1000);
    Parameters.begin();
#ifdef ENABLE_DEBUG
    #ifndef ESP32
        Serial1.begin(115200, SERIAL_8N1, SERIAL_TX_ONLY, GPIO2);
    #else
        Serial1.begin(115200, SERIAL_8N1, UART_DEBUG_RX, UART_DEBUG_TX);
    #endif
#else
#ifndef PW_LINK
    //   We only use it for non debug because GPIO02 is used as a serial
    //   pin (TX) when debugging.
    //-- Initialized "Reset To Factory
    pinMode(RESTORE_BTN, INPUT_PULLUP);
    attachInterrupt(RESTORE_BTN, reset_interrupt, FALLING);
#endif
#endif
    DEBUG_LOG("\nStart...\n");
    pinMode(STATUS_LED, OUTPUT); //Used for status
    SET_STATUS_LED(LED_OFF);
    Logger.begin(2048);
    DEBUG_LOG("Free Sketch Space: %u\n", ESP.getFreeSketchSpace());

    WiFi.disconnect(true);
    if(Parameters.getWifiMode() == WIFI_MODE_STA){
        DEBUG_LOG("\nConfiguring Wifi Station...\n");
        //-- Connect to an existing network
#ifndef ESP32
        WiFi.mode(WIFI_STA);
#endif
        WiFi.config(Parameters.getWifiStaIP(), Parameters.getWifiStaGateway(), Parameters.getWifiStaSubnet(), 0U, 0U);
        WiFi.begin(Parameters.getWifiStaSsid(), Parameters.getWifiStaPassword());

        //-- Wait a minute to connect
        for(int i = 0; i < 120 && WiFi.status() != WL_CONNECTED; i++) {
            #ifdef ENABLE_DEBUG
            Serial.print(".");
            #endif
            delay(500);
        }
        if(WiFi.status() == WL_CONNECTED) {
            localIP = WiFi.localIP();
            WiFi.setAutoReconnect(true);
        } else {
            //-- Fall back to AP mode if no connection could be established
            WiFi.disconnect(true);
            Parameters.setWifiMode(WIFI_MODE_AP);
        }
    }

    if(Parameters.getWifiMode() == WIFI_MODE_AP){
        DEBUG_LOG("\nConfiguring access point...\n");
        //-- Start AP
#ifndef ESP32
        WiFi.mode(WIFI_AP);
        WiFi.encryptionType(AUTH_WPA2_PSK);
#else
        WiFi.encryptionType(WIFI_AUTH_WPA2_PSK);
#endif
        WiFi.softAP(Parameters.getWifiSsid(), Parameters.getWifiPassword(), Parameters.getWifiChannel());
        localIP = WiFi.softAPIP();
        wait_for_client();
    }

    //-- Boost power to Max
#ifndef ESP32
    WiFi.setOutputPower(20.5);
#else
    WiFi.setTxPower(WIFI_POWER_19_5dBm);
#endif
    //-- MDNS
    DEBUG_LOG("Start mDNS...\n");
    if(MDNS.begin(DNSNAME)){
        MDNS.addService("http", "tcp", 80);
        DEBUG_LOG("http://%s.local exposed...\n",DNSNAME);
    }else{
        DEBUG_LOG("Url : %s.local NOT exposed (Error)...\n", DNSNAME);
    }
    //-- Initialize Comm Links
    DEBUG_LOG("Start WiFi Bridge\n");
    DEBUG_LOG("Local IP: %s\n", localIP.toString().c_str());

    Parameters.setLocalIPAddress(localIP);
    IPAddress gcs_ip(localIP);
    //-- I'm getting bogus IP from the DHCP server. Broadcasting for now.
    gcs_ip[3] = 255;
    GCS.begin(&Vehicle, gcs_ip);
    Vehicle.begin(&GCS);
    //-- Initialize Update Server
    updateServer.begin(&updateStatus);
}

//---------------------------------------------------------------------------------
//-- Main Loop
void loop() {
    if(!updateStatus.isUpdating()) {
        if (Component.inRawMode()) {
            GCS.readMessageRaw();
            delay(0);
            Vehicle.readMessageRaw();

        } else {
            GCS.readMessage();
            delay(0);
            Vehicle.readMessage();
        }
    }
    updateServer.checkUpdates();
}
