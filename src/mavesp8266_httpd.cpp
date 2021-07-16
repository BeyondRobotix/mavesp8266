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
 * @file mavesp8266_httpd.cpp
 * ESP8266 Wifi AP, MavLink UART/UDP Bridge
 *
 * @author Gus Grubba <mavlink@grubba.com>
 */
#ifndef ESP32
    #include <ESP8266WebServer.h>
#endif
#include "mavesp8266.h"
#include "mavesp8266_httpd.h"
#include "mavesp8266_parameters.h"
#include "mavesp8266_gcs.h"
#include "mavesp8266_vehicle.h"

#define WIFI_RX_VERY_LOW 0
#define WIFI_RX_UNRELIABLE 1
#define WIFI_RX_WEAK 2
#define WIFI_RX_RELIABLE 3
#define WIFI_RX_GOOD 4
#define WIFI_RX_EXCELLENT 5
#define WIFI_RX_MAX 6

#ifndef FW_NAME
    #define FW_NAME "unknown"
#endif

//Strength Wifi Signal based on https://eyesaas.com/wi-fi-signal-strength/
const char* kWifiStrength[7] = {
    "Very Low",                //(<= -90dBm)
    "Unreliable",              //(<= -80dBm)
    "Weak",                    //(<= -70dBm)
    "Reliable",                //(<= -67dBm)
    "Good",                    //(<= -60dBm)
    "Excellent",               //(<=-50dBm)
    "Maximum"                  //(<= -30dBm)
};

const char PROGMEM kTEXTPLAIN[]  = "text/plain";
const char PROGMEM kTEXTHTML[]   = "text/html";
const char PROGMEM kACCESSCTL[]  = "Access-Control-Allow-Origin";
const char PROGMEM kUPLOADFORM[] = "<h1><a href='/'>MAVLink WiFi Bridge</a></h1><form method='POST' action='/upload' enctype='multipart/form-data'><input type='file' name='update'><br><input type='submit' value='Update'></form>";
#ifndef ENABLE_DEBUG
    const char PROGMEM kHEADER[]     = "<!doctype html><html><head><title>MavLink Bridge</title></head><body><h1><a href='/'>MAVLink WiFi Bridge</a></h1>";
#else
    const char PROGMEM kHEADER[]     = "<!doctype html><html><head><title>MavLink Bridge (DEBUG)</title></head><body><h1><a href='/'>MAVLink WiFi Bridge (DEBUG)</a></h1>";
#endif
const char PROGMEM kBADARG[]     = "BAD ARGS";
const char PROGMEM kAPPJSON[]    = "application/json";

const char* kBAUD       = "baud";
const char* kPWD        = "pwd";
const char* kSSID       = "ssid";
const char* kPWDSTA     = "pwdsta";
const char* kSSIDSTA    = "ssidsta";
const char* kIPSTA      = "ipsta";
const char* kGATESTA    = "gatewaysta";
const char* kSUBSTA     = "subnetsta";
const char* kCPORT      = "cport";
const char* kHPORT      = "hport";
const char* kCHANNEL    = "channel";
const char* kDEBUG      = "debug";
const char* kREBOOT     = "reboot";
const char* kPOSITION   = "position";
const char* kMODE       = "mode";

const char* kFlashMaps[7] = {
    "512KB (256/256)",
    "256KB",
    "1MB (512/512)",
    "2MB (512/512)",
    "4MB (512/512)",
    "2MB (1024/1024)",
    "4MB (1024/1024)"
};

static uint32_t flash = 0;
static char paramCRC[12] = {""};
#ifndef ESP32
    ESP8266WebServer    webServer(80);
#else
    WebServer     webServer(80);
    WiFiUDP       Update;
#endif
MavESP8266Update*   updateCB    = NULL;
bool                started     = false;




//---------------------------------------------------------------------------------
void setNoCacheHeaders() {
    webServer.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    webServer.sendHeader("Pragma", "no-cache");
    webServer.sendHeader("Expires", "0");
}

//---------------------------------------------------------------------------------
void returnFail(String msg) {
    webServer.send(500, FPSTR(kTEXTPLAIN), msg + "\r\n");
}

//---------------------------------------------------------------------------------
void respondOK() {
    webServer.send(200, FPSTR(kTEXTPLAIN), "OK");
}

//---------------------------------------------------------------------------------
void handle_update() {
    webServer.sendHeader("Connection", "close");
    webServer.sendHeader(FPSTR(kACCESSCTL), "*");
    webServer.send(200, FPSTR(kTEXTHTML), FPSTR(kUPLOADFORM));
}

//---------------------------------------------------------------------------------
void handle_upload() {
    webServer.sendHeader("Connection", "close");
    webServer.sendHeader(FPSTR(kACCESSCTL), "*");
#ifndef ESP32
    webServer.send(200, FPSTR(kTEXTPLAIN), (Update.hasError()) ? "FAIL" : "OK");
#else
    webServer.send(200, FPSTR(kTEXTPLAIN), "OK");
#endif
    if(updateCB) {
        updateCB->updateCompleted();
    }
    ESP.restart();
}

//---------------------------------------------------------------------------------
void handle_upload_status() {
    bool success  = true;
    if(!started) {
        started = true;
        if(updateCB) {
            updateCB->updateStarted();
        }
    }
    HTTPUpload& upload = webServer.upload();
    if(upload.status == UPLOAD_FILE_START) {
        #ifdef DEBUG_SERIAL
            DEBUG_SERIAL.setDebugOutput(true);
        #endif
#ifndef ESP32
        WiFiUDP::stopAll();
#else
        Update.stop();
#endif
        #ifdef DEBUG_SERIAL
            DEBUG_SERIAL.printf("Update: %s\n", upload.filename.c_str());
        #endif
        uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
        if(!Update.begin(maxSketchSpace)) {
            #ifdef DEBUG_SERIAL
                Update.printError(DEBUG_SERIAL);
            #endif
            success = false;
        }
    } else if(upload.status == UPLOAD_FILE_WRITE) {
        if(Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
            #ifdef DEBUG_SERIAL
                Update.printError(DEBUG_SERIAL);
            #endif
            success = false;
        }
    } else if (upload.status == UPLOAD_FILE_END) {
#ifndef ESP32
        if (Update.end(true)) {
#else
        if (Update.endPacket()) {
#endif
            #ifdef DEBUG_SERIAL
                DEBUG_SERIAL.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
            #endif
        } else {
            #ifdef DEBUG_SERIAL
                Update.printError(DEBUG_SERIAL);
            #endif
            success = false;
        }
        #ifdef DEBUG_SERIAL
            DEBUG_SERIAL.setDebugOutput(false);
        #endif
    }
    yield();
    if(!success) {
        if(updateCB) {
            updateCB->updateError();
        }
    }
}

//---------------------------------------------------------------------------------
void handle_getParameters()
{
    String message = FPSTR(kHEADER);
    message += "<p>Parameters</p><table><tr><td width=\"240\">Name</td><td>Value</td></tr>";
    for(int i = 0; i < MavESP8266Parameters::ID_COUNT; i++) {
        message += "<tr><td>";
        message += getWorld()->getParameters()->getAt(i)->id;
        message += "</td>";
        unsigned long val = 0;
        if(getWorld()->getParameters()->getAt(i)->type == MAV_PARAM_TYPE_UINT32)
            val = (unsigned long)*((uint32_t*)getWorld()->getParameters()->getAt(i)->value);
        else if(getWorld()->getParameters()->getAt(i)->type == MAV_PARAM_TYPE_UINT16)
            val = (unsigned long)*((uint16_t*)getWorld()->getParameters()->getAt(i)->value);
        else
            val = (unsigned long)*((int8_t*)getWorld()->getParameters()->getAt(i)->value);
        message += "<td>";
        message += val;
        message += "</td></tr>";
    }
    message += "</table>";
    message += "</body>";
    webServer.send(200, FPSTR(kTEXTHTML), message);
}

//---------------------------------------------------------------------------------
static void handle_root()
{
    String message = FPSTR(kHEADER);
    message += "Version: ";
    char vstr[30];
    snprintf(vstr, sizeof(vstr), "%u.%u.%u", MAVESP8266_VERSION_MAJOR, MAVESP8266_VERSION_MINOR, MAVESP8266_VERSION_BUILD);
    message += vstr;
    message += "<p>\n";
    message += "<ul>\n";
    message += "<li><a href='/getstatus'>Get Status</a>\n";
    message += "<li><a href='/setup'>Setup</a>\n";
    message += "<li><a href='/getparameters'>Get Parameters</a>\n";
    message += "<li><a href='/update'>Update Firmware</a>\n";
    message += "<li><a href='/reboot'>Reboot</a>\n";
    message += "</ul></body>";
    setNoCacheHeaders();
    webServer.send(200, FPSTR(kTEXTHTML), message);
}

//---------------------------------------------------------------------------------
static void handle_setup()
{
    String message = FPSTR(kHEADER);
    message += "<h1>Setup</h1>\n";
    message += "<form action='/setparameters' method='post'>\n";

    message += "WiFi Mode:&nbsp;";
    message += "<input type='radio' name='mode' value='0'";
    if (getWorld()->getParameters()->getWifiMode() == WIFI_MODE_AP) {
        message += " checked";
    }
    message += ">AccessPoint\n";
    message += "<input type='radio' name='mode' value='1'";
    if (getWorld()->getParameters()->getWifiMode() == WIFI_MODE_STA) {
        message += " checked";
    }
    message += ">Station<br>\n";
    
    message += "AP SSID:&nbsp;";
    message += "<input type='text' name='ssid' value='";
    message += getWorld()->getParameters()->getWifiSsid();
    message += "'><br>";

    message += "AP Password (min len 8):&nbsp;";
    message += "<input type='text' name='pwd' value='";
    message += getWorld()->getParameters()->getWifiPassword();
    message += "'><br>";

    message += "WiFi Channel:&nbsp;";
    message += "<input type='text' name='channel' value='";
    message += getWorld()->getParameters()->getWifiChannel();
    message += "'><br>";

    message += "Station SSID:&nbsp;";
    message += "<input type='text' name='ssidsta' value='";
    message += getWorld()->getParameters()->getWifiStaSsid();
    message += "'><br>";

    message += "Station Password:&nbsp;";
    message += "<input type='text' name='pwdsta' value='";
    message += getWorld()->getParameters()->getWifiStaPassword();
    message += "'><br>";

    IPAddress IP;    
    message += "Station IP:&nbsp;";
    message += "<input type='text' name='ipsta' value='";
    IP = getWorld()->getParameters()->getWifiStaIP();
    message += IP.toString();
    message += "'><br>";

    message += "Station Gateway:&nbsp;";
    message += "<input type='text' name='gatewaysta' value='";
    IP = getWorld()->getParameters()->getWifiStaGateway();
    message += IP.toString();
    message += "'><br>";

    message += "Station Subnet:&nbsp;";
    message += "<input type='text' name='subnetsta' value='";
    IP = getWorld()->getParameters()->getWifiStaSubnet();
    message += IP.toString();
    message += "'><br>";

    message += "Host Port:&nbsp;";
    message += "<input type='text' name='hport' value='";
    message += getWorld()->getParameters()->getWifiUdpHport();
    message += "'><br>";

    message += "Client Port:&nbsp;";
    message += "<input type='text' name='cport' value='";
    message += getWorld()->getParameters()->getWifiUdpCport();
    message += "'><br>";
    
    message += "Baudrate:&nbsp;";
    message += "<input type='text' name='baud' value='";
    message += getWorld()->getParameters()->getUartBaudRate();
    message += "'><br>";
    
    message += "<input type='submit' value='Save'>";
    message += "</form>";
    setNoCacheHeaders();
    webServer.send(200, FPSTR(kTEXTHTML), message);
}
//---------------------------------------------------------------------------------
static void handle_getStatus()
{
    int8_t iRssi = 0;
    int8_t iWifiThreshold = 0; 
    if(!flash)
        flash = ESP.getFreeSketchSpace();
    if(!paramCRC[0]) {
        snprintf(paramCRC, sizeof(paramCRC), "%08X", getWorld()->getParameters()->paramHashCheck());
    }
    linkStatus* gcsStatus = getWorld()->getGCS()->getStatus();
    linkStatus* vehicleStatus = getWorld()->getVehicle()->getStatus();
    if(getWorld()->getParameters()->getWifiMode() == WIFI_MODE_STA){
        iRssi = WiFi.RSSI();
        iWifiThreshold = 0; 
        if(iRssi == 0) {
            iWifiThreshold = WIFI_RX_MAX;
        }else{
            iWifiThreshold = (iRssi <= -90)? WIFI_RX_VERY_LOW : iWifiThreshold;
            iWifiThreshold = ((-90 < iRssi) && (iRssi <= -80)) ? WIFI_RX_VERY_LOW : iWifiThreshold;
            iWifiThreshold = ((-80 < iRssi) && (iRssi <= 70)) ? WIFI_RX_UNRELIABLE : iWifiThreshold;
            iWifiThreshold = ((-70 < iRssi) && (iRssi <= 67)) ? WIFI_RX_WEAK : iWifiThreshold;
            iWifiThreshold = ((-67 < iRssi) && (iRssi <= 60)) ? WIFI_RX_RELIABLE : iWifiThreshold;
            iWifiThreshold = ((-60 < iRssi) && (iRssi <= 50)) ? WIFI_RX_GOOD : iWifiThreshold;
            iWifiThreshold = ((-50 < iRssi) && (iRssi <= 30)) ? WIFI_RX_EXCELLENT : iWifiThreshold;
            iWifiThreshold = (-30 <= iRssi) ? WIFI_RX_MAX : iWifiThreshold;
        }
    }
    String message = FPSTR(kHEADER);
    message += "<p>Comm Status</p><table><tr><td width=\"240\">Packets Received from GCS</td><td>";
    message += gcsStatus->packets_received;
    message += "</td></tr><tr><td>Packets Sent to GCS</td><td>";
    message += gcsStatus->packets_sent;
    message += "</td></tr><tr><td>GCS Packets Lost</td><td>";
    message += gcsStatus->packets_lost;
    message += "</td></tr><tr><td>Packets Received from Vehicle</td><td>";
    message += vehicleStatus->packets_received;
    message += "</td></tr><tr><td>Packets Sent to Vehicle</td><td>";
    message += vehicleStatus->packets_sent;
    message += "</td></tr><tr><td>Vehicle Packets Lost</td><td>";
    message += vehicleStatus->packets_lost;
    message += "</td></tr><tr><td>Radio Messages</td><td>";
    message += gcsStatus->radio_status_sent;
    message += "</td></tr></table>";
    message += "<p>System Status</p><table>\n";
    if(getWorld()->getParameters()->getWifiMode() == WIFI_MODE_STA){
        message += "<tr><td width=\"240\">Wifi Signal Strength (RSSI)</td><td>";
        message += iRssi; 
        message += " dBm ";
        message += kWifiStrength[iWifiThreshold];
        message += "</td></tr>\n";
    }
    message += "<tr><td width=\"240\">Flash Size</td><td>";
#ifndef ESP32
    message += ESP.getFlashChipRealSize();
#else
    message += ESP.getFlashChipSize();
#endif
    message += "</td></tr>\n";
    message += "<tr><td width=\"240\">Flash Available</td><td>";
    message += flash;
    message += "</td></tr>\n";
    message += "<tr><td>RAM Left</td><td>";
    message += String(ESP.getFreeHeap());
    message += "</td></tr>\n";
    message += "<tr><td>Parameters CRC</td><td>";
    message += paramCRC;
    message += "</td></tr>\n";
    message += "</table>";
    message += "</body>";
    setNoCacheHeaders();
    webServer.send(200, FPSTR(kTEXTHTML), message);
}

//---------------------------------------------------------------------------------
void handle_getJLog()
{
    uint32_t position = 0, len;
    if(webServer.hasArg(kPOSITION)) {
        position = webServer.arg(kPOSITION).toInt();
    }
    String logText = getWorld()->getLogger()->getLog(&position, &len);
    char jStart[128];
    snprintf(jStart, 128, "{\"len\":%d, \"start\":%d, \"text\": \"", len, position);
    String payLoad = jStart;
    payLoad += logText;
    payLoad += "\"}";
    webServer.send(200, FPSTR(kAPPJSON), payLoad);
}

//---------------------------------------------------------------------------------
void handle_getJSysInfo()
{
    if(!flash){
        flash = ESP.getFreeSketchSpace();
    }
    if(!paramCRC[0]) {
        snprintf(paramCRC, sizeof(paramCRC), "%08X", getWorld()->getParameters()->paramHashCheck());
    }
#ifndef ESP32
    uint32_t fid = spi_flash_get_id();
#else
    uint32_t fid = 0;
    for(int i=0; i<17; i=i+8) {
	  fid |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
	}
#endif
    char message[512];
    snprintf(message, 512,
        "{ "
        "\"firmware\": \"%s\", "
#ifndef ESP32
        "\"size\": \"%s\", "
        "\"id\": \"0x%02lX 0x%04lX\", "
#else
        "\"size\": \"%u\", "
        "\"id\": \"%u\", "
#endif
        "\"flashfree\": \"%u\", "
        "\"heapfree\": \"%u\", "
        "\"logsize\": \"%u\", "
        "\"paramcrc\": \"%s\""
        " }",
        FW_NAME,
#ifndef ESP32
        kFlashMaps[system_get_flash_size_map()],
        (long unsigned int)(fid & 0xff), (long unsigned int)((fid & 0xff00) | ((fid >> 16) & 0xff)),
#else
        ESP.getFlashChipSize(),
        fid,
#endif
        flash,
        ESP.getFreeHeap(),
        getWorld()->getLogger()->getPosition(),
        paramCRC
    );
    webServer.send(200, "application/json", message);
}

//---------------------------------------------------------------------------------
void handle_getJSysStatus()
{
    bool reset = false;
    if(webServer.hasArg("r")) {
        reset = webServer.arg("r").toInt() != 0;
    }
    linkStatus* gcsStatus = getWorld()->getGCS()->getStatus();
    linkStatus* vehicleStatus = getWorld()->getVehicle()->getStatus();
    if(reset) {
        memset(gcsStatus,     0, sizeof(linkStatus));
        memset(vehicleStatus, 0, sizeof(linkStatus));
    }
    char message[512];
    snprintf(message, 512,
        "{ "
        "\"gpackets\": \"%u\", "
        "\"gsent\": \"%u\", "
        "\"glost\": \"%u\", "
        "\"vpackets\": \"%u\", "
        "\"vsent\": \"%u\", "
        "\"vlost\": \"%u\", "
        "\"radio\": \"%u\", "
        "\"buffer\": \"%u\""
        " }",
        gcsStatus->packets_received,
        gcsStatus->packets_sent,
        gcsStatus->packets_lost,
        vehicleStatus->packets_received,
        vehicleStatus->packets_sent,
        vehicleStatus->packets_lost,
        gcsStatus->radio_status_sent,
        vehicleStatus->queue_status
    );
    webServer.send(200, "application/json", message);
}

//---------------------------------------------------------------------------------
void handle_setParameters()
{
    if(webServer.args() == 0) {
        returnFail(kBADARG);
        return;
    }
    bool ok = false;
    bool reboot = false;
    if(webServer.hasArg(kBAUD)) {
        ok = true;
        getWorld()->getParameters()->setUartBaudRate(webServer.arg(kBAUD).toInt());
    }
    if(webServer.hasArg(kPWD)) {
        ok = true;
        getWorld()->getParameters()->setWifiPassword(webServer.arg(kPWD).c_str());
    }
    if(webServer.hasArg(kSSID)) {
        ok = true;
        getWorld()->getParameters()->setWifiSsid(webServer.arg(kSSID).c_str());
    }
    if(webServer.hasArg(kPWDSTA)) {
        ok = true;
        getWorld()->getParameters()->setWifiStaPassword(webServer.arg(kPWDSTA).c_str());
    }
    if(webServer.hasArg(kSSIDSTA)) {
        ok = true;
        getWorld()->getParameters()->setWifiStaSsid(webServer.arg(kSSIDSTA).c_str());
    }
    if(webServer.hasArg(kIPSTA)) {
        IPAddress ip;
        ip.fromString(webServer.arg(kIPSTA).c_str());
        getWorld()->getParameters()->setWifiStaIP(ip);
    }
    if(webServer.hasArg(kGATESTA)) {
        IPAddress ip;
        ip.fromString(webServer.arg(kGATESTA).c_str());
        getWorld()->getParameters()->setWifiStaGateway(ip);
    }
    if(webServer.hasArg(kSUBSTA)) {
        IPAddress ip;
        ip.fromString(webServer.arg(kSUBSTA).c_str());
        getWorld()->getParameters()->setWifiStaSubnet(ip);
    }
    if(webServer.hasArg(kCPORT)) {
        ok = true;
        getWorld()->getParameters()->setWifiUdpCport(webServer.arg(kCPORT).toInt());
    }
    if(webServer.hasArg(kHPORT)) {
        ok = true;
        getWorld()->getParameters()->setWifiUdpHport(webServer.arg(kHPORT).toInt());
    }
    if(webServer.hasArg(kCHANNEL)) {
        ok = true;
        getWorld()->getParameters()->setWifiChannel(webServer.arg(kCHANNEL).toInt());
    }
    if(webServer.hasArg(kDEBUG)) {
        ok = true;
        getWorld()->getParameters()->setDebugEnabled(webServer.arg(kDEBUG).toInt());
    }
    if(webServer.hasArg(kMODE)) {
        ok = true;
        getWorld()->getParameters()->setWifiMode(webServer.arg(kMODE).toInt());
    }
    if(webServer.hasArg(kREBOOT)) {
        ok = true;
        reboot = webServer.arg(kREBOOT) == "1";
    }
    if(ok) {
        getWorld()->getParameters()->saveAllToEeprom();
        //-- Send new parameters back
        handle_getParameters();
        if(reboot) {
            delay(100);
            ESP.restart();
        }
    } else
        returnFail(kBADARG);
}

//---------------------------------------------------------------------------------
static void handle_reboot()
{
    String message = FPSTR(kHEADER);
    message += "rebooting ...</body>\n";
    setNoCacheHeaders();
    webServer.send(200, FPSTR(kTEXTHTML), message);
    delay(500);
    ESP.restart();    
}

//---------------------------------------------------------------------------------
//-- 404
void handle_notFound(){
    String message = "File Not Found\n\n";
    message += "URI: ";
    message += webServer.uri();
    message += "\nMethod: ";
    message += (webServer.method() == HTTP_GET) ? "GET" : "POST";
    message += "\nArguments: ";
    message += webServer.args();
    message += "\n";
    for (uint8_t i = 0; i < webServer.args(); i++){
        message += " " + webServer.argName(i) + ": " + webServer.arg(i) + "\n";
    }
    webServer.send(404, FPSTR(kTEXTPLAIN), message);
}

//---------------------------------------------------------------------------------
MavESP8266Httpd::MavESP8266Httpd()
{

}

//---------------------------------------------------------------------------------
//-- Initialize
void
MavESP8266Httpd::begin(MavESP8266Update* updateCB_)
{
    updateCB = updateCB_;
    webServer.on("/",               handle_root);
    webServer.on("/getparameters",  handle_getParameters);
    webServer.on("/setparameters",  handle_setParameters);
    webServer.on("/getstatus",      handle_getStatus);
    webServer.on("/reboot",         handle_reboot);
    webServer.on("/setup",          handle_setup);
    webServer.on("/info.json",      handle_getJSysInfo);
    webServer.on("/status.json",    handle_getJSysStatus);
    webServer.on("/log.json",       handle_getJLog);
    webServer.on("/update",         handle_update);
    webServer.on("/upload",         HTTP_POST, handle_upload, handle_upload_status);
    webServer.onNotFound(           handle_notFound);
    webServer.begin();
}

//---------------------------------------------------------------------------------
//-- Initialize
void
MavESP8266Httpd::checkUpdates()
{
    webServer.handleClient();
}
