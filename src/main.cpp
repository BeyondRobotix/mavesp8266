/****************************************************************************
 *
 * Copyright (c) 2015 Gus Grubba. All rights reserved.
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
 * Minimal, Bare-bones ESP8266 Wifi AP, MavLink UART/UDP Bridge
 *
 * @author Gus Grubba <mavlink@grubba.com>
 */

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>

extern "C" {
    // Espressif SDK
    #include "user_interface.h"
}

#include <mavlink.h>

//-- Debug sent out to Serial1 (GPIO02), which is TX only (no RX)
//#define DEBUG

//-- Forward declarations

bool read_uas_message();
bool read_gcs_message();
bool send_gcs_message();
bool send_uas_message();
void wait_for_client();

//-- WiFi AP Settings
const char*     ssid            = "PixRacer";
const char*     password        = "pixracer"; // Blank (or NULL) for an open AP.
IPAddress       localIP;
uint16_t        localPort       = 14555;
int32_t         wifiChannel     = 11;

//-- GCS Data
mavlink_message_t   gcs_message;
WiFiUDP             Udp;
uint16_t            gcs_port = 14550;
IPAddress           gcs_ip;

//-- UAS Data
mavlink_message_t   uas_message;

//---------------------------------------------------------------------------------
//-- Set things up
void setup() {

    delay(5000);

#ifdef DEBUG
    Serial1.begin(115200);
    Serial1.println();
    Serial1.println("Configuring access point...");
#endif

    //-- Start AP
    WiFi.mode(WIFI_AP);
    WiFi.encryptionType(AUTH_WPA2_PSK);
    WiFi.softAP(ssid, password, wifiChannel);
    localIP = WiFi.softAPIP();
    //-- I'm getting bogus IP from the DHCP server. Broadcasting for now.
    gcs_ip = localIP;
    gcs_ip[3] = 255;

#ifdef DEBUG
    Serial1.print("AP IP address: ");
    Serial1.println(localIP);
    Serial1.print("Broadcast IP: ");
    Serial1.println(gcs_ip);
#endif

#ifdef DEBUG
    Serial1.println("Waiting for DHCPD...");
#endif

    dhcp_status dstat = wifi_station_dhcpc_status();
    while (dstat != DHCP_STARTED) {
#ifdef DEBUG
        Serial1.print(".");
#endif
        delay(500);
        dstat = wifi_station_dhcpc_status();
    }

    wait_for_client();

#ifdef DEBUG
      Serial1.println("Start WiFi Bridge");
#endif

    //-- Start UDP
    Udp.begin(localPort);
    //-- Start UART connected to UAS
    Serial.begin(921600);
    //Serial.begin(57600);
    //-- Swap to TXD2/RXD2 (GPIO015/GPIO013) For ESP12 Only
    //Serial.swap();
    //-- Reset Message Buffers
    memset(&gcs_message, 0, sizeof(gcs_message));
    memset(&uas_message, 0, sizeof(uas_message));
}

//---------------------------------------------------------------------------------
//-- Main Loop
void loop() {
    //-- Read UART
    if(read_uas_message()) {
        send_gcs_message();
        memset(&uas_message, 0, sizeof(uas_message));
    }
    delay(0);
    //-- Read UDP
    if(read_gcs_message()) {
        send_uas_message();
        memset(&gcs_message, 0, sizeof(gcs_message));
    }
}

//---------------------------------------------------------------------------------
//-- Read MavLink message from UAS
bool read_uas_message()
{
    bool msgReceived = false;
    mavlink_status_t uas_status;
    while(Serial.available())
    {
        int result = Serial.read();
        if (result >= 0)
        {
            // Parsing
            msgReceived = mavlink_parse_char(MAVLINK_COMM_1, result, &uas_message, &uas_status);
            if(msgReceived) {
                #ifdef DEBUG
                if(uas_message.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                    Serial1.print("U");
                } else {
                    Serial1.print("u");
                }
                #endif
                break;
            }
        }
    }
    return msgReceived;
}

//---------------------------------------------------------------------------------
//-- Read MavLink message from GCS
bool read_gcs_message()
{
    bool msgReceived = false;
    int udp_count = Udp.parsePacket();
    if(udp_count > 0)
    {
        mavlink_status_t gcs_status;
        while(udp_count--)
        {
            int result = Udp.read();
            if (result >= 0)
            {
                // Parsing
                msgReceived = mavlink_parse_char(MAVLINK_COMM_2, result, &gcs_message, &gcs_status);
                if(msgReceived) {
                    //-- We no longer need to broadcast
                    if(gcs_ip[3] == 255) {
                        gcs_ip = Udp.remoteIP();
                        #ifdef DEBUG
                        Serial1.println();
                        Serial1.print("Response from GCS. Setting GCS IP to: ");
                        Serial1.println(gcs_ip);
                        #endif
                    }
                    #ifdef DEBUG
                    if(gcs_message.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                        Serial1.print("G");
                    } else {
                        Serial1.print("g");
                    }
                    #endif
                    break;
                }
            }
        }
    }
    return msgReceived;
}

//---------------------------------------------------------------------------------
//-- Forward message to the GCS
bool send_gcs_message() {
    // Translate message to buffer
    char buf[300];
    unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &uas_message);
    // Send it
    Udp.beginPacket(gcs_ip, gcs_port);
    Udp.write((uint8_t*)(void*)buf, len);
    Udp.endPacket();
}

//---------------------------------------------------------------------------------
//-- Forward message to the UAS
bool send_uas_message() {
    // Translate message to buffer
    char buf[300];
    unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &gcs_message);
    // Send it
    Serial.write((uint8_t*)(void*)buf, len);
}

//---------------------------------------------------------------------------------
//-- Wait for a DHCPD client
void wait_for_client() {
#ifdef DEBUG
    Serial1.println("Waiting for a client...");
    int wcount = 0;
#endif
    uint8 client_count = wifi_softap_get_station_num();
    while (!client_count) {
#ifdef DEBUG
        Serial1.print(".");
        if(++wcount > 80) {
            wcount = 0;
            Serial1.println();
        }
#endif
        delay(1000);
        client_count = wifi_softap_get_station_num();
    }
#ifdef DEBUG
    Serial1.println();
    Serial1.print("Got ");
    Serial1.print(client_count);
    Serial1.println(" client(s).");
#endif
}

