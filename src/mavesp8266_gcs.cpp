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
 * @file mavesp8266_gcs.cpp
 * ESP8266 Wifi AP, MavLink UART/UDP Bridge
 *
 * @author Gus Grubba <mavlink@grubba.com>
 */

#include "mavesp8266.h"
#include "mavesp8266_gcs.h"
#include "mavesp8266_parameters.h"

const char* kHASH_PARAM = "_HASH_CHECK";

//---------------------------------------------------------------------------------
MavESP8266GCS::MavESP8266GCS()
    : _udp_port(DEFAULT_UDP_HPORT)
    , _last_status_time(0)
{
    memset(&_message, 0, sizeof(_message));
}

//---------------------------------------------------------------------------------
//-- Initialize
void
MavESP8266GCS::begin(MavESP8266Bridge* forwardTo, IPAddress gcsIP)
{
    MavESP8266Bridge::begin(forwardTo);
    _ip = gcsIP;
    //-- Init variables that shouldn't change unless we reboot
    _udp_port = getWorld()->getParameters()->getWifiUdpHport();
    //-- Start UDP
    _udp.begin(getWorld()->getParameters()->getWifiUdpCport());
}

//---------------------------------------------------------------------------------
//-- Read MavLink message from GCS
void
MavESP8266GCS::readMessage()
{
    //-- Read UDP
    if(_readMessage()) {
        //-- If we have a message, forward it
        _forwardTo->sendMessage(&_message);
        memset(&_message, 0, sizeof(_message));
    }
    //-- Update radio status (1Hz)
    if(_heard_from && (millis() - _last_status_time > 1000)) {
        delay(0);
        _sendRadioStatus();
        _last_status_time = millis();
    }
}

//---------------------------------------------------------------------------------
//-- Read MavLink message from GCS
bool
MavESP8266GCS::_readMessage()
{
    bool msgReceived = false;
    int udp_count = _udp.parsePacket();
    if(udp_count > 0)
    {
        mavlink_status_t gcs_status;
        while(udp_count--)
        {
            int result = _udp.read();
            if (result >= 0)
            {
                // Parsing
                msgReceived = mavlink_parse_char(MAVLINK_COMM_2, result, &_message, &gcs_status);
                if(msgReceived) {
                    //-- We no longer need to broadcast
                    _status.packets_received++;
                    if(_ip[3] == 255) {
                        _ip = _udp.remoteIP();
                        getWorld()->getLogger()->log("Response from GCS. Setting GCS IP to: %s\n", _ip.toString().c_str());
                    }
                    //-- First packets
                    if(!_heard_from) {
                        if(_message.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                            _heard_from      = true;
                            _system_id       = _message.sysid;
                            _component_id    = _message.compid;
                            _seq_expected    = _message.seq + 1;
                            _last_heartbeat  = millis();
                        }
                    } else {
                        if(_message.msgid == MAVLINK_MSG_ID_HEARTBEAT)
                            _last_heartbeat = millis();
                        _checkLinkErrors(&_message);
                    }
                    //-- Check for message we might be interested
                    //
                    //   TODO: These response messages need to be queued up and sent as part of the main loop and not all
                    //   at once from here.
                    //
                    //-----------------------------------------------
                    //-- MAVLINK_MSG_ID_PARAM_SET
                    if(_message.msgid == MAVLINK_MSG_ID_PARAM_SET) {
                        mavlink_param_set_t param;
                        mavlink_msg_param_set_decode(&_message, &param);
                        DEBUG_LOG("MAVLINK_MSG_ID_PARAM_SET: %u %s\n", param.target_component, param.param_id);
                        if(param.target_component == MAV_COMP_ID_UDP_BRIDGE) {
                            _handleParamSet(&param);
                            //-- Eat message (don't send it to FC)
                            memset(&_message, 0, sizeof(_message));
                            msgReceived = false;
                            continue;
                        }
                    //-----------------------------------------------
                    //-- MAVLINK_MSG_ID_COMMAND_LONG
                    } else if(_message.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
                        mavlink_command_long_t cmd;
                        mavlink_msg_command_long_decode(&_message, &cmd);
                        if(cmd.target_component == MAV_COMP_ID_ALL || cmd.target_component == MAV_COMP_ID_UDP_BRIDGE) {
                            _handleCmdLong(&cmd);
                            //-- If it was directed to us, eat it and loop
                            if(cmd.target_component == MAV_COMP_ID_UDP_BRIDGE) {
                                //-- Eat message (don't send it to FC)
                                memset(&_message, 0, sizeof(_message));
                                msgReceived = false;
                                continue;
                            }
                        }
                    //-----------------------------------------------
                    //-- MAVLINK_MSG_ID_PARAM_REQUEST_LIST
                    } else if(_message.msgid == MAVLINK_MSG_ID_PARAM_REQUEST_LIST) {
                        mavlink_param_request_list_t param;
                        mavlink_msg_param_request_list_decode(&_message, &param);
                        DEBUG_LOG("MAVLINK_MSG_ID_PARAM_REQUEST_LIST: %u\n", param.target_component);
                        if(param.target_component == MAV_COMP_ID_ALL || param.target_component == MAV_COMP_ID_UDP_BRIDGE) {
                            _handleParamRequestList();
                        }
                    //-----------------------------------------------
                    //-- MAVLINK_MSG_ID_PARAM_REQUEST_READ
                    } else if(_message.msgid == MAVLINK_MSG_ID_PARAM_REQUEST_READ) {
                        mavlink_param_request_read_t param;
                        mavlink_msg_param_request_read_decode(&_message, &param);
                        //-- This component or all components?
                        if(param.target_component == MAV_COMP_ID_ALL || param.target_component == MAV_COMP_ID_UDP_BRIDGE) {
                            //-- If asking for hash, respond and pass through to the UAS
                            if(strncmp(param.param_id, kHASH_PARAM, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN) == 0) {
                                _sendParameter(kHASH_PARAM, getWorld()->getParameters()->paramHashCheck(), 0xFFFF);
                            } else {
                                _handleParamRequestRead(&param);
                                //-- If this was addressed to me only eat message
                                if(param.target_component == MAV_COMP_ID_UDP_BRIDGE) {
                                    //-- Eat message (don't send it to FC)
                                    memset(&_message, 0, sizeof(_message));
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
    if(!msgReceived) {
        if(_heard_from && (millis() - _last_heartbeat) > HEARTBEAT_TIMEOUT) {
            _heard_from = false;
            //-- Start broadcasting again
            _ip[3] = 255;
            getWorld()->getLogger()->log("Heartbeat timeout from GCS\n");
        }
    }
    return msgReceived;
}

//---------------------------------------------------------------------------------
//-- Forward message(s) to the GCS
void
MavESP8266GCS::sendMessage(mavlink_message_t* message, int count) {
    _udp.beginPacket(_ip, _udp_port);
    for(int i = 0; i < count; i++) {
        // Translate message to buffer
        char buf[300];
        unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message[i]);
        // Send it
        _udp.write((uint8_t*)(void*)buf, len);
        _status.packets_sent++;
    }
    _udp.endPacket();
}

//---------------------------------------------------------------------------------
//-- Forward message to the GCS
void
MavESP8266GCS::sendMessage(mavlink_message_t* message) {
    _sendSingleUdpMessage(message);
}

//---------------------------------------------------------------------------------
//-- Send Radio Status
void
MavESP8266GCS::_sendRadioStatus()
{
    linkStatus* st = _forwardTo->getStatus();
    //-- Build message    
    mavlink_message_t msg;
    mavlink_msg_radio_status_pack(
        _forwardTo->systemID(),
        MAV_COMP_ID_UDP_BRIDGE,
        &msg,
        0xff,   // We don't have access to RSSI
        0xff,   // We don't have access to Remote RSSI
        st->queue_status, // Outgoing queue status
        0,      // We don't have access to noise data
        0,      // We don't have access to remote noise data
        (uint16_t)(_status.packets_lost / 10),
        0       // We don't fix anything
    );
    _sendSingleUdpMessage(&msg);
    _status.radio_status_sent++;
}

//---------------------------------------------------------------------------------
//-- Send Debug Message
void
MavESP8266GCS::_sendStatusMessage(uint8_t type, const char* text)
{
    if(!getWorld()->getParameters()->getDebugEnabled() && type == MAV_SEVERITY_DEBUG) {
        return;
    }
    //-- Build message    
    mavlink_message_t msg;
    mavlink_msg_statustext_pack(
        _forwardTo->systemID(),
        MAV_COMP_ID_UDP_BRIDGE,
        &msg,
        type,
        text
    );
    _sendSingleUdpMessage(&msg);
}

//---------------------------------------------------------------------------------
//-- Set parameter
void
MavESP8266GCS::_handleParamSet(mavlink_param_set_t* param)
{
    for(int i = 0; i < MavESP8266Parameters::ID_COUNT; i++) {
        //-- Find parameter
        if(strncmp(param->param_id, getWorld()->getParameters()->getAt(i)->id, strlen(getWorld()->getParameters()->getAt(i)->id)) == 0) {
            //-- Skip Read Only
            if(!getWorld()->getParameters()->getAt(i)->readOnly) {
                //-- Set new value
                memcpy(getWorld()->getParameters()->getAt(i)->value, &param->param_value, getWorld()->getParameters()->getAt(i)->length);
            }
            //-- "Ack" it
            _sendParameter(getWorld()->getParameters()->getAt(i)->index);
            return;
        }
    }
}

//---------------------------------------------------------------------------------
//-- Handle Parameter Request List
void
MavESP8266GCS::_handleParamRequestList()
{
    for(int i = 0; i < MavESP8266Parameters::ID_COUNT; i++) {
        _sendParameter(getWorld()->getParameters()->getAt(i)->index);
        delay(0);
    }
}

//---------------------------------------------------------------------------------
//-- Handle Parameter Request Read
void
MavESP8266GCS::_handleParamRequestRead(mavlink_param_request_read_t* param)
{
    for(int i = 0; i < MavESP8266Parameters::ID_COUNT; i++) {
        //-- Find parameter
        if(param->param_index == getWorld()->getParameters()->getAt(i)->index || strncmp(param->param_id, getWorld()->getParameters()->getAt(i)->id, strlen(getWorld()->getParameters()->getAt(i)->id)) == 0) {
            _sendParameter(getWorld()->getParameters()->getAt(i)->index);
            return;
        }
    }
}

//---------------------------------------------------------------------------------
//-- Send Parameter (Index Based)
void
MavESP8266GCS::_sendParameter(uint16_t index)
{
    //-- Build message
    mavlink_param_value_t msg;
    msg.param_count = MavESP8266Parameters::ID_COUNT;
    msg.param_index = index;
    strncpy(msg.param_id, getWorld()->getParameters()->getAt(index)->id, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
    uint32_t val = 0;
    memcpy(&val, getWorld()->getParameters()->getAt(index)->value, getWorld()->getParameters()->getAt(index)->length);
    memcpy(&msg.param_value, &val, sizeof(uint32_t));
    msg.param_type = getWorld()->getParameters()->getAt(index)->type;
    mavlink_message_t mmsg;
    mavlink_msg_param_value_encode(
        _forwardTo->systemID(),
        MAV_COMP_ID_UDP_BRIDGE,
        &mmsg,
        &msg
    );
    _sendSingleUdpMessage(&mmsg);
}

//---------------------------------------------------------------------------------
//-- Send Parameter (Raw)
void
MavESP8266GCS::_sendParameter(const char* id, uint32_t value, uint16_t index)
{
    //-- Build message
    mavlink_param_value_t msg;
    msg.param_count = MavESP8266Parameters::ID_COUNT;
    msg.param_index = index;
    strncpy(msg.param_id, id, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
    memcpy(&msg.param_value, &value, sizeof(uint32_t));
    msg.param_type = MAV_PARAM_TYPE_UINT32;
    mavlink_message_t mmsg;
    mavlink_msg_param_value_encode(
        _forwardTo->systemID(),
        MAV_COMP_ID_UDP_BRIDGE,
        &mmsg,
        &msg
    );
    _sendSingleUdpMessage(&mmsg);
}

//---------------------------------------------------------------------------------
//-- Send UDP Single Message
void
MavESP8266GCS::_sendSingleUdpMessage(mavlink_message_t* msg)
{
    // Translate message to buffer
    char buf[300];
    unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, msg);
    // Send it
    _udp.beginPacket(_ip, _udp_port);
    _udp.write((uint8_t*)(void*)buf, len);
    _udp.endPacket();
    _status.packets_sent++;
}

//---------------------------------------------------------------------------------
//-- Handle Commands
void
MavESP8266GCS::_handleCmdLong(mavlink_command_long_t* cmd)
{
    bool reboot = false;
    uint8_t result = MAV_RESULT_UNSUPPORTED;
    if(cmd->command == MAV_CMD_PREFLIGHT_STORAGE) {
        //-- Read from EEPROM
        if((uint8_t)cmd->param1 == 0) {
            result = MAV_RESULT_ACCEPTED;
            getWorld()->getParameters()->loadAllFromEeprom();
        //-- Write to EEPROM
        } else if((uint8_t)cmd->param1 == 1) {
            result = MAV_RESULT_ACCEPTED;
            getWorld()->getParameters()->saveAllToEeprom();
            delay(0);
        //-- Restore defaults
        } else if((uint8_t)cmd->param1 == 2) {
            result = MAV_RESULT_ACCEPTED;
            getWorld()->getParameters()->resetToDefaults();
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
        _forwardTo->systemID(),
        MAV_COMP_ID_UDP_BRIDGE,
        &msg,
        cmd->command,
        result
    );
    _sendSingleUdpMessage(&msg);
    delay(0);
    if(reboot) {
        _wifiReboot();
    }
}

//---------------------------------------------------------------------------------
//-- Reboot
void
MavESP8266GCS::_wifiReboot()
{
    _sendStatusMessage(MAV_SEVERITY_NOTICE, "Rebooting WiFi Bridge.");
    delay(50);
    ESP.reset();
}
