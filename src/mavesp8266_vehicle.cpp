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
 * @file mavesp8266_vehicle.cpp
 * ESP8266 Wifi AP, MavLink UART/UDP Bridge
 *
 * @author Gus Grubba <mavlink@grubba.com>
 */

#include "mavesp8266.h"
#include "mavesp8266_vehicle.h"
#include "mavesp8266_parameters.h"
#include "mavesp8266_component.h"

//---------------------------------------------------------------------------------
MavESP8266Vehicle::MavESP8266Vehicle()
    : _queue_count(0)
    , _queue_time(0)
    , _buffer_status(50.0)
{
    _recv_chan = MAVLINK_COMM_0;
    _send_chan = MAVLINK_COMM_1;
    memset(_message, 0 , sizeof(_message));
}

//---------------------------------------------------------------------------------
//-- Initialize
void
MavESP8266Vehicle::begin(MavESP8266Bridge* forwardTo)
{
    MavESP8266Bridge::begin(forwardTo);
    //-- Start UART connected to UAS
    Serial.begin(getWorld()->getParameters()->getUartBaudRate());
    //-- Swap to TXD2/RXD2 (GPIO015/GPIO013) For ESP12 Only
#ifdef ENABLE_DEBUG
#ifdef ARDUINO_ESP8266_ESP12
    Serial.swap();
#endif
#endif
    // raise serial buffer size (default is 256)
    Serial.setRxBufferSize(1024);
}

//---------------------------------------------------------------------------------
//-- Read MavLink message from UAS
void
MavESP8266Vehicle::readMessage()
{
    if(_queue_count < UAS_QUEUE_SIZE) {
        if(_readMessage()) {
            _queue_count++;
        }
    }
    //-- Do we have a message to send and is it time to forward data?
    if(_queue_count && (_queue_count >= UAS_QUEUE_THRESHOLD || (millis() - _queue_time) > UAS_QUEUE_TIMEOUT)) {
        int sent = _forwardTo->sendMessage(_message, _queue_count);
        //-- Sent it all?
        if(sent == _queue_count) {
            memset(_message, 0, sizeof(_message));
            _queue_count = 0;
            _queue_time  = millis();
        //-- Sent at least some?
        } else if(sent) {
            //-- Move the pending ones up the queue
            int left = _queue_count - sent;
            for(int i = 0; i < left; i++) {
                //memcpy(&_message[sent+i], &_message[i], sizeof(mavlink_message_t));
		// 0 <--- sent+1
		// 1 <--- sent+2
		// 2 <--- sent+3
                memcpy(&_message[i], &_message[sent+i], sizeof(mavlink_message_t));
            }
            _queue_count = left;
        }
        //-- Maintain buffer status
        float cur_status  = 0.0;
        float buffer_size = (float)UAS_QUEUE_THRESHOLD;
        float buffer_left = (float)(UAS_QUEUE_THRESHOLD - _queue_count);
        if(buffer_left > 0.0)
            cur_status = ((buffer_left / buffer_size) * 100.0f);
        _buffer_status = (_buffer_status * 0.05f) + (cur_status * 0.95);
    }
    //-- Update radio status (1Hz)
    if(_heard_from && (millis() - _last_status_time > 1000)) {
        delay(0);
        _sendRadioStatus();
        _last_status_time = millis();
    }
}

void
MavESP8266Vehicle::readMessageRaw() {
    char buf[1024];
    int buf_index = 0;

    while(Serial.available() && buf_index < 300)
    {
        int result = Serial.read();
        if (result >= 0)
        {
            buf[buf_index] = (char)result;
            buf_index++;
        }
    }

    _forwardTo->sendMessageRaw((uint8_t*)buf, buf_index);
}

//---------------------------------------------------------------------------------
//-- Send MavLink message to UAS
int
MavESP8266Vehicle::sendMessage(mavlink_message_t* message, int count) {
    for(int i = 0; i < count; i++) {
        sendMessage(&message[i]);
    }
    return count;
}

//---------------------------------------------------------------------------------
//-- Send MavLink message to UAS
int
MavESP8266Vehicle::sendMessage(mavlink_message_t* message) {
    // Translate message to buffer
    char buf[300];
    unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, message);
    // Send it
    Serial.write((uint8_t*)(void*)buf, len);
    _status.packets_sent++;
    return 1;
}

int
MavESP8266Vehicle::sendMessageRaw(uint8_t *buffer, int len) {
    Serial.write(buffer, len);
    //Serial.flush();
    return len;
}

//---------------------------------------------------------------------------------
//-- We have some special status to capture when asked for
linkStatus*
MavESP8266Vehicle::getStatus()
{
    _status.queue_status = (uint8_t)_buffer_status;
    return &_status;
}

//---------------------------------------------------------------------------------
//-- Read MavLink message from UAS
bool
MavESP8266Vehicle::_readMessage()
{
    bool msgReceived = false;
    while(Serial.available())
    {
        int result = Serial.read();
        if (result >= 0)
        {
            // Parsing
            msgReceived = mavlink_frame_char_buffer(&_rxmsg,
                                                    &_rxstatus,
                                                    result,
                                                    &_message[_queue_count],
                                                    &_mav_status);
            if(msgReceived) {
                _status.packets_received++;
                //-- Is this the first packet we got?
                if(!_heard_from) {
                    if(_message[_queue_count].msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                        _heard_from     = true;
                        _component_id   = _message[_queue_count].compid;
                        _system_id      = _message[_queue_count].sysid;
                        _seq_expected   = _message[_queue_count].seq + 1;
                        _last_heartbeat = millis();
                    }
                } else {
                    if(_message[_queue_count].msgid == MAVLINK_MSG_ID_HEARTBEAT)
                        _last_heartbeat = millis();
                    _checkLinkErrors(&_message[_queue_count]);
                }

                if (msgReceived == MAVLINK_FRAMING_BAD_CRC ||
                    msgReceived == MAVLINK_FRAMING_BAD_SIGNATURE) {
                    // we don't process messages locally with bad CRC,
                    // but we do forward them, so when new messages
                    // are added we can bridge them
                    break;
                }

                //-- Check for message we might be interested
                if(getWorld()->getComponent()->handleMessage(this, &_message[_queue_count])){
                    //-- Eat message (don't send it to GCS)
                    memset(&_message[_queue_count], 0, sizeof(mavlink_message_t));
                    msgReceived = false;
                    continue;
                }

                break;
            }
        }
    }
    if(!msgReceived) {
        if(_heard_from && (millis() - _last_heartbeat) > HEARTBEAT_TIMEOUT) {
            _heard_from = false;
            getWorld()->getLogger()->log("Heartbeat timeout from Vehicle\n");
        }
    }
    return msgReceived;
}

//---------------------------------------------------------------------------------
//-- Send Radio Status
void
MavESP8266Vehicle::_sendRadioStatus()
{
    getStatus();
    //-- Build message
    mavlink_message_t msg {};
    mavlink_msg_radio_status_pack_chan(
        _forwardTo->systemID(),
        MAV_COMP_ID_UDP_BRIDGE,
        _send_chan,
        &msg,
        0,      // We don't have access to RSSI
        0,      // We don't have access to Remote RSSI
        _status.queue_status, // UDP queue status
        0,      // We don't have access to noise data
        0,      // We don't have access to remote noise data
        (uint16_t)(_status.packets_lost / 10),
        0       // We don't fix anything
    );
    sendMessage(&msg);
    _status.radio_status_sent++;
}
