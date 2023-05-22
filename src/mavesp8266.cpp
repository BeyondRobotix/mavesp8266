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
 * @file mavesp8266.cpp
 * ESP8266 Wifi AP, MavLink UART/UDP Bridge
 *
 * @author Gus Grubba <mavlink@grubba.com>
 */

#include "mavesp8266.h"
#include "mavesp8266_parameters.h"

//---------------------------------------------------------------------------------
//-- Base Comm Link
MavESP8266Bridge::MavESP8266Bridge()
    : _heard_from(false)
    , _system_id(0)
    , _component_id(0)
    , _seq_expected(0)
    , _last_heartbeat(0)
    , _last_status_time(0)
    , _forwardTo(NULL)
{
    memset(&_status, 0, sizeof(_status));
}

//---------------------------------------------------------------------------------
//-- Initialize
void
MavESP8266Bridge::begin(MavESP8266Bridge* forwardTo)
{
    _forwardTo  = forwardTo;
}

//---------------------------------------------------------------------------------
//-- Check for link errors
void
MavESP8266Bridge::_checkLinkErrors(mavlink_message_t* msg)
{
    //-- Don't bother if we have not heard from the link (and it's the proper sys/comp ids)
    if(!_heard_from || msg->sysid != _system_id || msg->compid != _component_id) {
        return;
    }
    uint16_t seq_received = (uint16_t)msg->seq;
    uint16_t packet_lost_count = 0;
    //-- Account for overflow during packet loss
    if(seq_received < _seq_expected) {
        packet_lost_count = (seq_received + 255) - _seq_expected;
    } else {
        packet_lost_count = seq_received - _seq_expected;
    }
    _seq_expected = msg->seq + 1;
    _status.packets_lost += packet_lost_count;
}

//---------------------------------------------------------------------------------
MavESP8266Log:: MavESP8266Log()
    : _buffer(NULL)
    , _buffer_size(0)
    , _log_offset(0)
    , _log_position(0)
{

}

//---------------------------------------------------------------------------------
void
MavESP8266Log::begin(size_t bufferSize)
{
    _buffer_size = bufferSize & 0xFFFE;
    _buffer = (char*)malloc(_buffer_size);
}

//---------------------------------------------------------------------------------
size_t
MavESP8266Log::log(const char *format, ...) {
    va_list arg;
    va_start(arg, format);
    char temp[1024];
    size_t len = ets_vsnprintf(temp, 1024, format, arg);
#ifdef ENABLE_DEBUG
    Serial1.print(temp);
#endif

    if(_buffer) {
        for(int i = 0; i < (int)len; i++) {
            _buffer[_log_offset] = temp[i];
            _log_offset = (_log_offset + 1) % _buffer_size;
            _log_position++;
        }
    }
    va_end(arg);
    return len;
}

//---------------------------------------------------------------------------------
String
MavESP8266Log::getLog(uint32_t* pStart, uint32_t* pLen) {
    String buffer;

    uint32_t position = *pStart, len = getLogSize();
    if (position < _log_position - len) { //-- Can't read data that was overriden
        position = _log_position - len;
    } else if (position > _log_position) { //-- Can't read data from the future
        position = _log_position;
    }
    len = _log_position - position;
    *pStart = position;
    *pLen = len;

    uint32_t r = (_log_offset + (_buffer_size - len)) % _buffer_size;
    while(len > 0) {
        char c = _buffer[r];
        //-- Copy as JSON encoded characters
        if (c == '\\' || c == '"') {
            buffer += '\\';
            buffer += c;
        } else if (c < ' ') {
            char tmp[12];
            snprintf(tmp, 12, "\\u%04x", c);
            buffer += tmp;
        } else {
            buffer += c;
        }
        r = (r + 1) % _buffer_size;
        len--;
    }
    return buffer;
}

//---------------------------------------------------------------------------------
uint32_t
MavESP8266Log::getLogSize()
{
    return min(_log_position, _buffer_size);
}

//---------------------------------------------------------------------------------
uint32_t
MavESP8266Log::getPosition()
{
    return _log_position;
}
