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
  * @file mavesp8266_status.cpp
  * ESP8266 Wifi AP, MavLink UART/UDP Bridge
  *
  * @author Sam Knox <samjcknox@outlook.com>
  */


#include "mavesp8266_status.h"
#define GPIO4 4

MavESP8266Status::MavESP8266Status()
{
}

void
MavESP8266Status::begin() {
    pinMode(GPIO4, OUTPUT);
}

  //---------------------------------------------------------------------------------
  //-- Statues Lights Update
  // LED on GPIO 4 will:
  // * Solid - If wifi is connected
  // * Flashing - If heartbeat has arrived within timout (_heard_from logic)

void MavESP8266Status::statusUpdate() {

    // Wifi status update
    if (WiFi.status() == WL_CONNECTED) {
        _wifi_status = 1;
        // If low, set high
        if (!_led_state) {
            digitalWrite(GPIO4, HIGH);
        }
    }
    else {
        _wifi_status = 0;
        // If high, set low
        if (_led_state) {
            digitalWrite(GPIO4, LOW);
        }
    }
    if (_heard_from) {
        _wifi_status = 2;
        if (millis() - _time_next_blink <= 0) {
            if (_led_state) {
                digitalWrite(GPIO4, LOW);
            }
            else {
                digitalWrite(GPIO4, HIGH);
            }
            _time_next_blink = millis() + 1000;

        }
    }
}
