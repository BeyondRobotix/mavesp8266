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
 * @file mavesp8266.h
 * ESP8266 Wifi AP, MavLink UART/UDP Bridge
 *
 * @author Gus Grubba <mavlink@grubba.com>
 */

#ifndef MAVESP8266_H
#define MAVESP8266_H

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <mavlink.h>

 extern "C" {
    // Espressif SDK
    #include "user_interface.h"
}

class MavESP8266Parameters;
class MavESP8266Vehicle;
class MavESP8266GCS;

#define DEFAULT_UART_SPEED          921600
#define DEFAULT_WIFI_CHANNEL        11
#define DEFAULT_UDP_HPORT           14550
#define DEFAULT_UDP_CPORT           14555

#define HEARTBEAT_TIMEOUT           10 * 1000

//-- TODO: This needs to come from the build system
#define MAVESP8266_VERSION_MAJOR    1
#define MAVESP8266_VERSION_MINOR    0
#define MAVESP8266_VERSION_BUILD    6
#define MAVESP8266_VERSION          ((MAVESP8266_VERSION_MAJOR << 24) & 0xFF00000) | ((MAVESP8266_VERSION_MINOR << 16) & 0x00FF0000) | (MAVESP8266_VERSION_BUILD & 0xFFFF)

//-- Debug sent out to Serial1 (GPIO02), which is TX only (no RX).
//#define ENABLE_DEBUG

#ifdef ENABLE_DEBUG
#define DEBUG_LOG(format, ...) do { getWorld()->getLogger()->log(format, ## __VA_ARGS__); } while(0)
#else
#define DEBUG_LOG(format, ...) do { } while(0)
#endif

//---------------------------------------------------------------------------------
//-- Link Status
struct linkStatus {
    uint32_t    packets_received;
    uint32_t    packets_lost;
    uint32_t    packets_sent;
    uint32_t    radio_status_sent;
    uint8_t     queue_status;
};

//---------------------------------------------------------------------------------
//-- Base Comm Link
class MavESP8266Bridge {
public:
    MavESP8266Bridge();
    virtual ~MavESP8266Bridge(){;}
    virtual void    begin           (MavESP8266Bridge* forwardTo);
    virtual void    readMessage     () = 0;
    virtual void    sendMessage     (mavlink_message_t* message, int count) = 0;
    virtual void    sendMessage     (mavlink_message_t* message) = 0;
    virtual bool    heardFrom       () { return _heard_from;    }
    virtual uint8_t systemID        () { return _system_id;     }
    virtual uint8_t componentID     () { return _component_id;  }
    virtual linkStatus* getStatus   () { return &_status;       }
protected:
    virtual void    _checkLinkErrors(mavlink_message_t* msg);
protected:
    bool                    _heard_from;
    uint8_t                 _system_id;
    uint8_t                 _component_id;
    uint8_t                 _seq_expected;
    uint32_t                _last_heartbeat;
    linkStatus              _status;
    MavESP8266Bridge*       _forwardTo;
};

//---------------------------------------------------------------------------------
//-- Logger
class MavESP8266Log {
public:
    MavESP8266Log   ();
    void            begin           (size_t bufferSize); // Allocate a buffer for the log
    size_t          log             (const char *format, ...); // Add to the log
    String          getLog          (uint32_t position); // Get the log starting at a position
    uint32_t        getLogSize      (); // Number of bytes available at the current log position
private:
    char*           _buffer; // Raw memory
    size_t          _buffer_size; // Size of the above memory
    uint32_t        _log_offset; // Position in the buffer
    uint32_t        _log_position; // Absolute position in the log since boot
};

//---------------------------------------------------------------------------------
//-- Accessors
class MavESP8266World {
public:
    virtual ~MavESP8266World(){;}
    virtual MavESP8266Parameters*   getParameters   () = 0;
    virtual MavESP8266Vehicle*      getVehicle      () = 0;
    virtual MavESP8266GCS*          getGCS          () = 0;
    virtual MavESP8266Log*          getLogger       () = 0;
};

//---------------------------------------------------------------------------------
//-- HTTP Update Status
class MavESP8266Update {
public:
    virtual ~MavESP8266Update(){;}
    virtual void updateStarted  () = 0;
    virtual void updateCompleted() = 0;
    virtual void updateError    () = 0;
};

extern MavESP8266World* getWorld();

#endif
