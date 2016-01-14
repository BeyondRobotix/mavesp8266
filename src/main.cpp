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

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <EEPROM.h>

extern "C" {
    // Espressif SDK
    #include "user_interface.h"
}

#include <mavlink.h>

//-- TODO: This needs to come from the build system
#define MAVESP8266_VERSION_MAJOR    1
#define MAVESP8266_VERSION_MINOR    0
#define MAVESP8266_VERSION_BUILD    1

//-- Debug sent out to Serial1 (GPIO02), which is TX only (no RX)
//#define DEBUG

//-- Forward declarations

bool     read_uas_message           ();
bool     read_gcs_message           ();
bool     send_gcs_message           ();
bool     send_uas_message           ();
void     wait_for_client            ();
void     send_radio_status          ();
void     send_status_message        (uint8_t type, const char* text);
void     handle_param_set           (mavlink_param_set_t* param);
void     handle_param_request_list  ();
void     handle_param_request_read  (mavlink_param_request_read_t* param);
void     send_parameter             (uint16_t index);
void     send_parameter             (const char* id, uint32_t value, uint16_t index);
void     send_single_udp_message    (mavlink_message_t* msg);
void     check_upd_errors           (mavlink_message_t* msg);
void     handle_command_long        (mavlink_command_long_t* cmd);
uint32_t param_hash_check           ();
uint32_t crc32part                  (uint8_t* value, uint32_t len, uint32_t crc);
void     init_eeprom                ();
void     set_all_defaults           ();
uint32_t get_eeprom_crc             ();
void     save_all_to_eeprom         ();
void     load_all_from_eeprom       ();
void     wifi_reboot                ();

//-- Constants

#define DEFAULT_UART_SPEED      921600
#define DEFAULT_WIFI_CHANNEL    11
#define DEFAULT_UDP_HPORT       14550
#define DEFAULT_UDP_CPORT       14555

const char* kDEFAULT_SSDI       = "PixRacer";
const char* kDEFAULT_PASSWORD   = "pixracer";
const char* kHASH_PARAM         = "_HASH_CHECK";

#define GPIO02  2

//-- Parameters
//   No string support in parameters so we stash a char[16] into 4 uint32_t
//   This is dying for a proper OO encapsulation and abstraction...

uint32_t    param_sw_version;
int8_t      param_debug_enabled;
uint32_t    param_wifi_channel;
uint16_t    param_wifi_udp_hport;
uint16_t    param_wifi_udp_cport;
char        param_wifi_ssid[16];
char        param_wifi_password[16];
uint32_t    param_uart_baud_rate;

struct stMavEspParameters {
    char        id[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN];
    void*       value;
    uint16_t    index;
    uint8_t     length;
    uint8_t     type;
};

enum {
    ID_PARAMETER_FWVER = 0,
    ID_PARAMETER_DEBUG,
    ID_PARAMETER_CHANNEL,
    ID_PARAMETER_HPORT,
    ID_PARAMETER_CPORT,
    ID_PARAMETER_SSID1,
    ID_PARAMETER_SSID2,
    ID_PARAMETER_SSID3,
    ID_PARAMETER_SSID4,
    ID_PARAMETER_PASS1,
    ID_PARAMETER_PASS2,
    ID_PARAMETER_PASS3,
    ID_PARAMETER_PASS4,
    ID_PARAMETER_UART,
    ID_PARAMETER_COUNT
};

struct stMavEspParameters mavParameters[] = {
//   ID                 Value (pointer)             Index                   Length                          Type
    {"SW_VER",          &param_sw_version,          ID_PARAMETER_FWVER,     sizeof(uint32_t),               MAV_PARAM_TYPE_UINT32},
    {"DEBUG_ENABLED",   &param_debug_enabled,       ID_PARAMETER_DEBUG,     sizeof(int8_t),                 MAV_PARAM_TYPE_INT8},
    {"WIFI_CHANNEL",    &param_wifi_channel,        ID_PARAMETER_CHANNEL,   sizeof(uint32_t),               MAV_PARAM_TYPE_UINT32},
    {"WIFI_UDP_HPORT",  &param_wifi_udp_hport,      ID_PARAMETER_HPORT,     sizeof(uint16_t),               MAV_PARAM_TYPE_UINT16},
    {"WIFI_UDP_CPORT",  &param_wifi_udp_cport,      ID_PARAMETER_CPORT,     sizeof(uint16_t),               MAV_PARAM_TYPE_UINT16},
    {"WIFI_SSID1",      &param_wifi_ssid[0],        ID_PARAMETER_SSID1,     sizeof(uint32_t),               MAV_PARAM_TYPE_UINT32},
    {"WIFI_SSID2",      &param_wifi_ssid[4],        ID_PARAMETER_SSID2,     sizeof(uint32_t),               MAV_PARAM_TYPE_UINT32},
    {"WIFI_SSID3",      &param_wifi_ssid[8],        ID_PARAMETER_SSID3,     sizeof(uint32_t),               MAV_PARAM_TYPE_UINT32},
    {"WIFI_SSID4",      &param_wifi_ssid[12],       ID_PARAMETER_SSID4,     sizeof(uint32_t),               MAV_PARAM_TYPE_UINT32},
    {"WIFI_PASSWORD1",  &param_wifi_password[0],    ID_PARAMETER_PASS1,     sizeof(uint32_t),               MAV_PARAM_TYPE_UINT32},
    {"WIFI_PASSWORD2",  &param_wifi_password[4],    ID_PARAMETER_PASS2,     sizeof(uint32_t),               MAV_PARAM_TYPE_UINT32},
    {"WIFI_PASSWORD3",  &param_wifi_password[8],    ID_PARAMETER_PASS3,     sizeof(uint32_t),               MAV_PARAM_TYPE_UINT32},
    {"WIFI_PASSWORD4",  &param_wifi_password[12],   ID_PARAMETER_PASS4,     sizeof(uint32_t),               MAV_PARAM_TYPE_UINT32},
    {"UART_BAUDRATE",   &param_uart_baud_rate,      ID_PARAMETER_UART,      sizeof(uint32_t),               MAV_PARAM_TYPE_UINT32}
};

//-- Reserved space for EEPROM persistence. A change in this will cause all values to reset to defaults.
#define EEPROM_SPACE            32 * sizeof(uint32_t)
#define EEPROM_CRC_ADD          EEPROM_SPACE - (sizeof(uint32_t) << 1)

#ifndef DEBUG
uint8_t                 reset_state;
#endif

//-- WiFi AP Settings
IPAddress               localIP;

//-- GCS Data
mavlink_message_t       gcs_message;
WiFiUDP                 Udp;
IPAddress               gcs_ip;
uint8_t                 gcs_system_id       = 0;
uint8_t                 gcs_component_id    = 0;
bool                    gcs_heard_from      = false;
uint16_t                gcs_udp_port        = DEFAULT_UDP_HPORT;

uint8_t                 udp_seq_expected    = 0;
uint16_t                udp_packets_lost    = 0;
uint32_t                udp_packets_count   = 0;

//-- UAS Data
bool                    uas_heard_from      = false;
uint8_t                 system_id           = 0;

//-- UDP Outgoing Packet Queue
#define UAS_QUEUE_SIZE          5
#define UAS_QUEUE_TIMEOUT       5 // 5ms
int                     uas_queue_count     = 0;
unsigned long           uas_queue_time      = 0;
mavlink_message_t       uas_message[UAS_QUEUE_SIZE];

//-- Radio Status
unsigned long           last_status_time    = 0;

//-- 16-Entry CRC Lookup Table
static uint32_t crc_table[] = {
    0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f, 0xe963a535, 0x9e6495a3,
    0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988, 0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91,
    0x1db71064, 0x6ab020f2, 0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
    0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9, 0xfa0f3d63, 0x8d080df5,
    0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172, 0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b,
    0x35b5a8fa, 0x42b2986c, 0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
    0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423, 0xcfba9599, 0xb8bda50f,
    0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924, 0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d,
    0x76dc4190, 0x01db7106, 0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
    0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d, 0x91646c97, 0xe6635c01,
    0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e, 0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457,
    0x65b0d9c6, 0x12b7e950, 0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
    0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7, 0xa4d1c46d, 0xd3d6f4fb,
    0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0, 0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9,
    0x5005713c, 0x270241aa, 0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
    0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81, 0xb7bd5c3b, 0xc0ba6cad,
    0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a, 0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683,
    0xe3630b12, 0x94643b84, 0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
    0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb, 0x196c3671, 0x6e6b06e7,
    0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc, 0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5,
    0xd6d6a3e8, 0xa1d1937e, 0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
    0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55, 0x316e8eef, 0x4669be79,
    0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236, 0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f,
    0xc5ba3bbe, 0xb2bd0b28, 0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
    0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f, 0x72076785, 0x05005713,
    0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38, 0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21,
    0x86d3d2d4, 0xf1d4e242, 0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
    0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69, 0x616bffd3, 0x166ccf45,
    0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2, 0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db,
    0xaed16a4a, 0xd9d65adc, 0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
    0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693, 0x54de5729, 0x23d967bf,
    0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94, 0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d
};

//---------------------------------------------------------------------------------
//-- Set things up
void setup() {

    delay(1000);

#ifndef DEBUG
    //-- Initialized GPIO02 (Used for "Reset To Factory")
    //   We only use it for non bebug because GPIO02 is used as a serial
    //   pin (TX) when debugging.
    pinMode(GPIO02, INPUT_PULLUP);
    reset_state = digitalRead(GPIO02);
#endif

#ifdef DEBUG
    Serial1.begin(115200);
    Serial1.println();
    Serial1.println("Configuring access point...");
    Serial1.print("Free Sketch Space: ");
    Serial1.println(ESP.getFreeSketchSpace());
    Serial1.print("Message size: ");
    Serial1.println(sizeof(mavlink_message_t));
    Serial1.print("Message buffer size: ");
    Serial1.println(sizeof(uas_message));
#endif

    //-- TODO: User begin()/end() where it's used
    EEPROM.begin(EEPROM_SPACE);
    init_eeprom();

    //-- Init variables that shouldn't change unless we reboot
    gcs_udp_port = param_wifi_udp_hport;

    //-- Start AP
    WiFi.mode(WIFI_AP);
    WiFi.encryptionType(AUTH_WPA2_PSK);
    WiFi.softAP(param_wifi_ssid, param_wifi_password, param_wifi_channel);
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
    Udp.begin(param_wifi_udp_cport);
    //-- Start UART connected to UAS
    Serial.begin(param_uart_baud_rate);
    //-- Swap to TXD2/RXD2 (GPIO015/GPIO013) For ESP12 Only
#ifdef DEBUG
    Serial.swap();
#endif
    //-- Reset Message Buffers
    memset(&gcs_message, 0, sizeof(gcs_message));
    memset(&uas_message, 0, sizeof(uas_message));
    uas_queue_time      = millis();
    uas_queue_count     = 0;
    udp_packets_lost    = 0;
}

//---------------------------------------------------------------------------------
//-- Main Loop
void loop() {
#ifndef DEBUG
    //-- Test for "Reset To Factory"
    /* Needs testing
    int reset = digitalRead(GPIO02);
    if(reset != reset_state) {
        set_all_defaults();
        save_all_to_eeprom();
        wifi_reboot();
    }
    */
#endif
    //-- Read UART
    if(read_uas_message()) {
        uas_queue_count++;
    }
    delay(0);
    //-- Read UDP
    if(read_gcs_message()) {
        send_uas_message();
        memset(&gcs_message, 0, sizeof(gcs_message));
    }
    //-- Do we have a message to send and is it time to send data to GCS?
    if(uas_queue_count && (uas_queue_count == UAS_QUEUE_SIZE || (millis() - uas_queue_time) > UAS_QUEUE_TIMEOUT)) {
        send_gcs_message();
        memset(&uas_message, 0, sizeof(uas_message));
        //#ifdef DEBUG
        //Serial1.print(uas_queue_count);
        //#endif
        uas_queue_count = 0;
        uas_queue_time  = millis();
    }
    //-- Update radio status (1Hz)
    if(uas_heard_from && (millis() - last_status_time > 1000)) {
        delay(0);
        send_radio_status();
        last_status_time = millis();
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
            msgReceived = mavlink_parse_char(MAVLINK_COMM_1, result, &uas_message[uas_queue_count], &uas_status);
            if(msgReceived) {
                system_id = uas_message[uas_queue_count].sysid;
                //#ifdef DEBUG
                //if(uas_message[uas_queue_count].msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                //    Serial1.print("U");
                //}
                //#endif
                //-- Is this the first packet we got?
                if(!uas_heard_from) {
                    uas_heard_from = true;
                }
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
                    udp_packets_count++;
                    if(gcs_ip[3] == 255) {
                        gcs_ip = Udp.remoteIP();
                        #ifdef DEBUG
                        Serial1.println();
                        Serial1.print("Response from GCS. Setting GCS IP to: ");
                        Serial1.println(gcs_ip);
                        #endif
                    }
                    //#ifdef DEBUG
                    //    if(gcs_message.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                    //        Serial1.print("G");
                    //    }
                    //#endif
                    //-- First packets
                    if(!gcs_heard_from) {
                        if(gcs_message.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                            gcs_heard_from      = true;
                            gcs_system_id       = gcs_message.sysid;
                            gcs_component_id    = gcs_message.compid;
                            udp_seq_expected    = gcs_message.seq + 2; //-- For whatver reason, QGC increments this by 2
                        }
                    } else {
                        check_upd_errors(&gcs_message);
                    }
                    //-- Check for message we might be interested
                    //
                    //   TODO: These response messages need to be queued up and sent as part of the main loop and not all
                    //   at once from here.
                    //
                    #ifdef DEBUG
                        //if(gcs_message.msgid != MAVLINK_MSG_ID_HEARTBEAT) {
                        //    Serial1.println("");
                        //    Serial1.print("New Message from QGC: ");
                        //    Serial1.println(gcs_message.msgid);
                        //}
                    #endif
                    //-----------------------------------------------
                    //-- MAVLINK_MSG_ID_PARAM_SET
                    if(gcs_message.msgid == MAVLINK_MSG_ID_PARAM_SET) {
                        mavlink_param_set_t param;
                        mavlink_msg_param_set_decode(&gcs_message, &param);
                        #ifdef DEBUG
                            Serial1.println("");
                            Serial1.print("MAVLINK_MSG_ID_PARAM_SET: ");
                            Serial1.print(param.target_component);
                            Serial1.print(" ");
                            Serial1.println(param.param_id);
                        #endif
                        if(param.target_component == MAV_COMP_ID_UDP_BRIDGE) {
                            handle_param_set(&param);
                            //-- Eat message (don't send it to FC)
                            memset(&gcs_message, 0, sizeof(gcs_message));
                            msgReceived = false;
                            continue;
                        }
                    //-----------------------------------------------
                    //-- MAVLINK_MSG_ID_COMMAND_LONG
                    } else if(gcs_message.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
                        mavlink_command_long_t cmd;
                        mavlink_msg_command_long_decode(&gcs_message, &cmd);
                        if(cmd.target_component == MAV_COMP_ID_ALL || cmd.target_component == MAV_COMP_ID_UDP_BRIDGE) {
                            handle_command_long(&cmd);
                            //-- If it was directed to us, eat it and loop
                            if(cmd.target_component == MAV_COMP_ID_UDP_BRIDGE) {
                                //-- Eat message (don't send it to FC)
                                memset(&gcs_message, 0, sizeof(gcs_message));
                                msgReceived = false;
                                continue;
                            }
                        }
                    //-----------------------------------------------
                    //-- MAVLINK_MSG_ID_PARAM_REQUEST_LIST
                    } else if(gcs_message.msgid == MAVLINK_MSG_ID_PARAM_REQUEST_LIST) {
                        mavlink_param_request_list_t param;
                        mavlink_msg_param_request_list_decode(&gcs_message, &param);
                        #ifdef DEBUG
                            Serial1.println("");
                            Serial1.print("MAVLINK_MSG_ID_PARAM_REQUEST_LIST: ");
                            Serial1.println(param.target_component);
                        #endif
                        if(param.target_component == MAV_COMP_ID_ALL || param.target_component == MAV_COMP_ID_UDP_BRIDGE) {
                            handle_param_request_list();
                        }
                    //-----------------------------------------------
                    //-- MAVLINK_MSG_ID_PARAM_REQUEST_READ
                    } else if(gcs_message.msgid == MAVLINK_MSG_ID_PARAM_REQUEST_READ) {
                        mavlink_param_request_read_t param;
                        mavlink_msg_param_request_read_decode(&gcs_message, &param);
                        #ifdef DEBUG
                            //Serial1.println("");
                            //Serial1.print("MAVLINK_MSG_ID_PARAM_REQUEST_READ: ");
                            //Serial1.print(param.target_component);
                            //Serial1.print(" ");
                            //Serial1.println(param.param_id);
                        #endif
                        //-- This component or all components?
                        if(param.target_component == MAV_COMP_ID_ALL || param.target_component == MAV_COMP_ID_UDP_BRIDGE) {
                            //-- If asking for hash, respond and pass through to the UAS
                            if(strncmp(param.param_id, kHASH_PARAM, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN) == 0) {
                                send_parameter(kHASH_PARAM, param_hash_check(), 0xFFFF);
                            } else {
                                handle_param_request_read(&param);
                                //-- If this was addressed to me only eat message
                                if(param.target_component == MAV_COMP_ID_UDP_BRIDGE) {
                                    //-- Eat message (don't send it to FC)
                                    memset(&gcs_message, 0, sizeof(gcs_message));
                                    msgReceived = false;
                                    continue;
                                }
                            }
                        }
                    }
                    //-- Got message, leave
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
    Udp.beginPacket(gcs_ip, gcs_udp_port);
    for(int i = 0; i < uas_queue_count; i++) {
        // Translate message to buffer
        char buf[300];
        unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &uas_message[i]);
        // Send it
        Udp.write((uint8_t*)(void*)buf, len);
    }
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

//---------------------------------------------------------------------------------
//-- Send Radio Status
void send_radio_status()
{
    float buffer_size = (float)UAS_QUEUE_SIZE;
    float buffer_left = (float)(UAS_QUEUE_SIZE - uas_queue_count);
    //-- Build message    
    mavlink_message_t msg;
    mavlink_msg_radio_status_pack(
        system_id,
        MAV_COMP_ID_UDP_BRIDGE,
        &msg,
        0xff,   // We don't have access to RSSI
        0xff,   // We don't have access to Remote RSSI
        (uint8_t)((buffer_left / buffer_size) * 100.0f),
        0,      // We don't have access to noise data
        0,      // We don't have access to remote noise data
        udp_packets_lost,
        0       // We don't fix anything
    );
    send_single_udp_message(&msg);
}

//---------------------------------------------------------------------------------
//-- Send Debug Message
void send_status_message(uint8_t type, const char* text)
{
    if(!param_debug_enabled && type == MAV_SEVERITY_DEBUG) {
        return;
    }
    //-- Build message    
    mavlink_message_t msg;
    mavlink_msg_statustext_pack(
        system_id,
        MAV_COMP_ID_UDP_BRIDGE,
        &msg,
        type,
        text
    );
    send_single_udp_message(&msg);
}

//---------------------------------------------------------------------------------
//-- Set parameter
void handle_param_set(mavlink_param_set_t* param)
{
    for(int i = 0; i < ID_PARAMETER_COUNT; i++) {
        //-- Find parameter
        if(strncmp(param->param_id, mavParameters[i].id, strlen(mavParameters[i].id)) == 0) {
            //-- Version is Read Only
            if(i != ID_PARAMETER_FWVER) {
                //-- Set new value
                memcpy(mavParameters[i].value, &param->param_value, mavParameters[i].length);
            }
            //-- "Ack" it
            send_parameter(mavParameters[i].index);
            return;
        }
    }
}

//---------------------------------------------------------------------------------
//-- Handle Parameter Request List
void handle_param_request_list()
{
    for(int i = 0; i < ID_PARAMETER_COUNT; i++) {
        send_parameter(mavParameters[i].index);
        delay(0);
    }
}

//---------------------------------------------------------------------------------
//-- Handle Parameter Request Read
void handle_param_request_read(mavlink_param_request_read_t* param)
{
    for(int i = 0; i < ID_PARAMETER_COUNT; i++) {
        //-- Find parameter
        if(param->param_index == mavParameters[i].index || strncmp(param->param_id, mavParameters[i].id, strlen(mavParameters[i].id)) == 0) {
            send_parameter(mavParameters[i].index);
            return;
        }
    }
}

//---------------------------------------------------------------------------------
//-- Send Parameter (Index Based)
void send_parameter(uint16_t index)
{
    #ifdef DEBUG
        Serial1.print("Sending Parameter: ");
        Serial1.print(mavParameters[index].id);
        Serial1.print(" Value: ");
        if(mavParameters[index].type == MAV_PARAM_TYPE_UINT32)
            Serial1.println(*((uint32_t*)mavParameters[index].value));
        else if(mavParameters[index].type == MAV_PARAM_TYPE_UINT16)
            Serial1.println(*((uint16_t*)mavParameters[index].value));
        else
            Serial1.println(*((int8_t*)mavParameters[index].value));
    #endif
    //-- Build message
    mavlink_param_value_t msg;
    msg.param_count = ID_PARAMETER_COUNT;
    msg.param_index = index;
    strncpy(msg.param_id, mavParameters[index].id, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
    uint32_t val = 0;
    memcpy(&val, mavParameters[index].value, mavParameters[index].length);
    memcpy(&msg.param_value, &val, sizeof(uint32_t));
    msg.param_type = mavParameters[index].type;
    mavlink_message_t mmsg;
    mavlink_msg_param_value_encode(
        system_id,
        MAV_COMP_ID_UDP_BRIDGE,
        &mmsg,
        &msg
    );
    send_single_udp_message(&mmsg);
}

//---------------------------------------------------------------------------------
//-- Send Parameter (Raw)
void send_parameter(const char* id, uint32_t value, uint16_t index)
{
    #ifdef DEBUG
        Serial1.print("Sending Parameter: ");
        Serial1.print(id);
        Serial1.print(" Value: ");
        Serial1.println(value);
    #endif
    //-- Build message
    mavlink_param_value_t msg;
    msg.param_count = ID_PARAMETER_COUNT;
    msg.param_index = index;
    strncpy(msg.param_id, id, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
    memcpy(&msg.param_value, &value, sizeof(uint32_t));
    msg.param_type = MAV_PARAM_TYPE_UINT32;
    mavlink_message_t mmsg;
    mavlink_msg_param_value_encode(
        system_id,
        MAV_COMP_ID_UDP_BRIDGE,
        &mmsg,
        &msg
    );
    send_single_udp_message(&mmsg);
}

//---------------------------------------------------------------------------------
//-- Send UDP Single Message
void send_single_udp_message(mavlink_message_t* msg)
{
    // Translate message to buffer
    char buf[300];
    unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, msg);
    // Send it
    Udp.beginPacket(gcs_ip, gcs_udp_port);
    Udp.write((uint8_t*)(void*)buf, len);
    Udp.endPacket();
}

//---------------------------------------------------------------------------------
//-- Compute parameters hash
uint32_t param_hash_check()
{
    uint32_t crc = 0;
    for(int i = 0; i < ID_PARAMETER_COUNT; i++) {
        crc = crc32part((uint8_t *)(void*)mavParameters[i].id, strlen(mavParameters[i].id), crc);
        //-- TODO: When parameters get to support strings, this needs to be updated (strlen() instead of .length)
        crc = crc32part((uint8_t *)(void*)mavParameters[i].value, mavParameters[i].length, crc);
    }
    delay(0);
    return crc;
}

//---------------------------------------------------------------------------------
//-- Compute CRC32 for a buffer
uint32_t crc32part(uint8_t* src, uint32_t len, uint32_t crc)
{
    for (int i = 0;  i < len;  i++) {
        crc = crc_table[(crc ^ src[i]) & 0xff] ^ (crc >> 8);
    }
    return crc;
}

//---------------------------------------------------------------------------------
//-- Keep track of packet loss
void check_upd_errors(mavlink_message_t* msg)
{
#if 0

    This is not working. The sequence is completely out of whack and I cannot count on
    it to infer packet loss.  It seems QGC sends multiple sets of sequence numbers and
    even those are not normal. Some are incremented by 2 others by 1.

    //-- Don't bother if we have not heard from the GCS (and it's the proper sys/comp ids)
    if(!gcs_heard_from || msg->sysid != gcs_system_id || msg->compid != gcs_component_id) {
        return;
    }
    uint16_t seq_received = (uint16_t)msg->seq;
    uint16_t packet_lost_count = 0;
    //-- Account for overflow during packet loss
    if(seq_received < udp_seq_expected) {
        packet_lost_count = (seq_received + 255) - udp_seq_expected;
    } else {
        packet_lost_count = seq_received - udp_seq_expected;
    }
    if(param_debug_enabled) {
        char debug_message[52];
        snprintf(debug_message, 51, "Total: %u Seq: %u Expctd: %u Lost: %u",
            udp_packets_count, msg->seq, udp_seq_expected, packet_lost_count);
        send_status_message(MAV_SEVERITY_DEBUG, debug_message);
    }
    udp_packets_lost += packet_lost_count;
    udp_seq_expected = msg->seq + 2; //-- For whatver reason, QGC increments this by 2
#endif
}

//---------------------------------------------------------------------------------
//-- Handle Commands
void handle_command_long(mavlink_command_long_t* cmd)
{
    bool reboot = false;
    uint8_t result = MAV_RESULT_UNSUPPORTED;
    if(cmd->command == MAV_CMD_PREFLIGHT_STORAGE) {
        //-- Read from EEPROM
        if((uint8_t)cmd->param1 == 0) {
            result = MAV_RESULT_ACCEPTED;
            load_all_from_eeprom();
        //-- Write to EEPROM
        } else if((uint8_t)cmd->param1 == 1) {
            result = MAV_RESULT_ACCEPTED;
            save_all_to_eeprom();
            delay(0);
        //-- Restore defaults
        } else if((uint8_t)cmd->param1 == 2) {
            result = MAV_RESULT_ACCEPTED;
            set_all_defaults();
        }
    } else if(cmd->command == MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN) {
        //-- Reset "Companion Computer"
        if((uint8_t)cmd->param2 == 1) {
            result = MAV_RESULT_ACCEPTED;
            reboot = true;
        }
    }
    //-- Response
    mavlink_message_t msg;
    mavlink_msg_command_ack_pack(
        system_id,
        MAV_COMP_ID_UDP_BRIDGE,
        &msg,
        cmd->command,
        result
    );
    send_single_udp_message(&msg);
    delay(0);
    if(reboot) {
        wifi_reboot();
    }
}

//---------------------------------------------------------------------------------
//-- Initializes EEPROM. If not initialized, set to defaults and save it.
void init_eeprom()
{
    load_all_from_eeprom();
    //-- Is it uninitialized?
    uint32_t saved_crc = 0;
    EEPROM.get(EEPROM_CRC_ADD, saved_crc);
    uint32_t current_crc = get_eeprom_crc();
    if(saved_crc != current_crc) {
        #ifdef DEBUG
            Serial1.print("Initializing EEPROM. Saved: ");
            Serial1.print(saved_crc);
            Serial1.print(" Current: ");
            Serial1.println(current_crc);
        #endif
        //-- Set all defaults
        set_all_defaults();
        //-- Save it all and store CRC
        save_all_to_eeprom();
    } else {
        //-- Load all parameters from EEPROM
        load_all_from_eeprom();
    }
}

//---------------------------------------------------------------------------------
//-- Computes EEPROM CRC
void set_all_defaults()
{
    param_sw_version        = (((MAVESP8266_VERSION_MAJOR << 24) & 0xF000) | ((MAVESP8266_VERSION_MINOR << 16) & 0x0F00) | (MAVESP8266_VERSION_BUILD & 0x00FF));
    param_debug_enabled     = 0;
    param_wifi_channel      = DEFAULT_WIFI_CHANNEL;
    param_wifi_udp_hport    = DEFAULT_UDP_HPORT;
    param_wifi_udp_cport    = DEFAULT_UDP_CPORT;
    param_uart_baud_rate    = DEFAULT_UART_SPEED;
    strncpy(param_wifi_ssid, kDEFAULT_SSDI, sizeof(param_wifi_ssid));
    strncpy(param_wifi_password, kDEFAULT_PASSWORD, sizeof(param_wifi_password));
}

//---------------------------------------------------------------------------------
//-- Computes EEPROM CRC
uint32_t get_eeprom_crc()
{
    uint32_t crc  = 0;
    uint32_t size = 0;
    //-- Get size of all parameter data
    for(int i = 0; i < ID_PARAMETER_COUNT; i++) {
        size += mavParameters[i].length;
    }
    //-- Computer CRC
    for (int i = 0 ; i < size; i++) {
        crc = crc_table[(crc ^ EEPROM.read(i)) & 0xff] ^ (crc >> 8);
    }
    return crc;
}

//---------------------------------------------------------------------------------
//-- Saves all parameters to EEPROM
void save_all_to_eeprom()
{
    //-- Init flash space
    uint8_t* ptr = EEPROM.getDataPtr();
    memset(ptr, 0, EEPROM_SPACE);
    //-- Write all paramaters to flash
    uint32_t address = 0;
    for(int i = 0; i < ID_PARAMETER_COUNT; i++) {
        ptr = (uint8_t*)mavParameters[i].value;
        #ifdef DEBUG
            Serial1.print("Saving to EEPROM: ");
            Serial1.print(mavParameters[i].id);
            Serial1.print(" Value: ");
            if(mavParameters[i].type == MAV_PARAM_TYPE_UINT32)
                Serial1.println(*((uint32_t*)mavParameters[i].value));
            else if(mavParameters[i].type == MAV_PARAM_TYPE_UINT16)
                Serial1.println(*((uint16_t*)mavParameters[i].value));
            else
                Serial1.println(*((int8_t*)mavParameters[i].value));
        #endif
        for(int j = 0; j < mavParameters[i].length; j++, address++, ptr++) {
            EEPROM.write(address, *ptr);
        }
    }
    uint32_t saved_crc = get_eeprom_crc();
    EEPROM.put(EEPROM_CRC_ADD, saved_crc);
    EEPROM.commit();
    #ifdef DEBUG
        Serial1.print("Saved CRC: ");
        Serial1.print(saved_crc);
        Serial1.println("");
    #endif
}

//---------------------------------------------------------------------------------
//-- Saves all parameters to EEPROM
void load_all_from_eeprom()
{
    uint32_t address = 0;
    for(int i = 0; i < ID_PARAMETER_COUNT; i++) {
        uint8_t* ptr = (uint8_t*)mavParameters[i].value;
        for(int j = 0; j < mavParameters[i].length; j++, address++, ptr++) {
            *ptr = EEPROM.read(address);
        }
        #ifdef DEBUG
            Serial1.print("Loading from EEPROM: ");
            Serial1.print(mavParameters[i].id);
            Serial1.print(" Value: ");
            if(mavParameters[i].type == MAV_PARAM_TYPE_UINT32)
                Serial1.println(*((uint32_t*)mavParameters[i].value));
            else if(mavParameters[i].type == MAV_PARAM_TYPE_UINT16)
                Serial1.println(*((uint16_t*)mavParameters[i].value));
            else
                Serial1.println(*((int8_t*)mavParameters[i].value));
        #endif
    }
    #ifdef DEBUG
        Serial1.println("");
    #endif
}

//---------------------------------------------------------------------------------
//-- Reboot
void wifi_reboot()
{
    send_status_message(MAV_SEVERITY_NOTICE, "Rebooting WiFi Bridge.");
    ESP.reset();
}
