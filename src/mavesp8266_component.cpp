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
 * @file mavesp8266_component.cpp
 * ESP8266 Wifi AP, MavLink UART/UDP Bridge
 *
 * @author Gus Grubba <mavlink@grubba.com>
 */

#include "mavesp8266.h"
#include "mavesp8266_component.h"
#include "mavesp8266_parameters.h"
#include "mavesp8266_vehicle.h"

const char* kHASH_PARAM = "_HASH_CHECK";


MavESP8266Component::MavESP8266Component() {


}

bool
MavESP8266Component::inRawMode() {
  // switch out of raw mode when not needed anymore
  if (_in_raw_mode_time > 0 && millis() > _in_raw_mode_time + 5000) {
    _in_raw_mode = false;
    _in_raw_mode_time = 0;
    getWorld()->getLogger()->log("Raw mode disabled\n");
  }

  return _in_raw_mode;
}

bool
MavESP8266Component::handleMessage(MavESP8266Bridge* sender, mavlink_message_t* message) {

  //
  //   TODO: These response messages need to be queued up and sent as part of the main loop and not all
  //   at once from here.
  //
  //-----------------------------------------------

  //-- MAVLINK_MSG_ID_PARAM_SET
  if(message->msgid == MAVLINK_MSG_ID_PARAM_SET) {
      mavlink_param_set_t param;
      mavlink_msg_param_set_decode(message, &param);
      DEBUG_LOG("MAVLINK_MSG_ID_PARAM_SET: %u %s\n", param.target_component, param.param_id);
      if(param.target_component == MAV_COMP_ID_UDP_BRIDGE) {
          _handleParamSet(sender, &param);
          return true;
      }
  //-----------------------------------------------
  //-- MAVLINK_MSG_ID_COMMAND_LONG
  } else if(message->msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
      mavlink_command_long_t cmd;
      mavlink_msg_command_long_decode(message, &cmd);
      if(cmd.target_component == MAV_COMP_ID_ALL || cmd.target_component == MAV_COMP_ID_UDP_BRIDGE) {
          _handleCmdLong(sender, &cmd, cmd.target_component);
          //-- If it was directed to us, eat it and loop
          if(cmd.target_component == MAV_COMP_ID_UDP_BRIDGE) {
              return true;
          }
      }
  //-----------------------------------------------
  //-- MAVLINK_MSG_ID_PARAM_REQUEST_LIST
  } else if(message->msgid == MAVLINK_MSG_ID_PARAM_REQUEST_LIST) {
      mavlink_param_request_list_t param;
      mavlink_msg_param_request_list_decode(message, &param);
      DEBUG_LOG("MAVLINK_MSG_ID_PARAM_REQUEST_LIST: %u\n", param.target_component);
      if(param.target_component == MAV_COMP_ID_ALL || param.target_component == MAV_COMP_ID_UDP_BRIDGE) {
          _handleParamRequestList(sender);
      }
      if(param.target_component == MAV_COMP_ID_UDP_BRIDGE) {
          // don't pass it along
          return true;
      }
  //-----------------------------------------------
  //-- MAVLINK_MSG_ID_PARAM_REQUEST_READ
  } else if(message->msgid == MAVLINK_MSG_ID_PARAM_REQUEST_READ) {
      mavlink_param_request_read_t param;
      mavlink_msg_param_request_read_decode(message, &param);
      //-- This component or all components?
      if(param.target_component == MAV_COMP_ID_ALL || param.target_component == MAV_COMP_ID_UDP_BRIDGE) {
          //-- If asking for hash, respond and pass through
          if(strncmp(param.param_id, kHASH_PARAM, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN) == 0) {
              _sendParameter(sender, kHASH_PARAM, getWorld()->getParameters()->paramHashCheck(), 0xFFFF);
          } else {
              _handleParamRequestRead(sender, &param);
              //-- If this was addressed to me only eat message
          }
          if(param.target_component == MAV_COMP_ID_UDP_BRIDGE) {
              //-- Eat message (don't send it to FC)
              return true;
          }
      }
  }

  //-- Couldn't handle the message, pass on
  return false;
}





//---------------------------------------------------------------------------------
//-- Send Debug Message
void
MavESP8266Component::_sendStatusMessage(MavESP8266Bridge* sender, uint8_t type, const char* text)
{
    if(!getWorld()->getParameters()->getDebugEnabled() && type == MAV_SEVERITY_DEBUG) {
        return;
    }
    //-- Build message
    mavlink_message_t msg;
    mavlink_msg_statustext_pack_chan(
        getWorld()->getVehicle()->systemID(),
        MAV_COMP_ID_UDP_BRIDGE,
        sender->_send_chan,
        &msg,
        type,
        text
    );
    sender->sendMessage(&msg);
}




//---------------------------------------------------------------------------------
//-- Set parameter
void
MavESP8266Component::_handleParamSet(MavESP8266Bridge* sender, mavlink_param_set_t* param)
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
            _sendParameter(sender, getWorld()->getParameters()->getAt(i)->index);
            return;
        }
    }
}

//---------------------------------------------------------------------------------
//-- Handle Parameter Request List
void
MavESP8266Component::_handleParamRequestList(MavESP8266Bridge* sender)
{
    for(int i = 0; i < MavESP8266Parameters::ID_COUNT; i++) {
        _sendParameter(sender, getWorld()->getParameters()->getAt(i)->index);
        delay(0);
    }
}

//---------------------------------------------------------------------------------
//-- Handle Parameter Request Read
void
MavESP8266Component::_handleParamRequestRead(MavESP8266Bridge* sender, mavlink_param_request_read_t* param)
{
    for(int i = 0; i < MavESP8266Parameters::ID_COUNT; i++) {
        //-- Find parameter
        if(param->param_index == getWorld()->getParameters()->getAt(i)->index || strncmp(param->param_id, getWorld()->getParameters()->getAt(i)->id, strlen(getWorld()->getParameters()->getAt(i)->id)) == 0) {
            _sendParameter(sender, getWorld()->getParameters()->getAt(i)->index);
            return;
        }
    }
}

//---------------------------------------------------------------------------------
//-- Send Parameter (Index Based)
void
MavESP8266Component::_sendParameter(MavESP8266Bridge* sender, uint16_t index)
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
    mavlink_msg_param_value_encode_chan(
        getWorld()->getVehicle()->systemID(),
        MAV_COMP_ID_UDP_BRIDGE,
        sender->_send_chan,
        &mmsg,
        &msg
    );

    sender->sendMessage(&mmsg);
}

//---------------------------------------------------------------------------------
//-- Send Parameter (Raw)
void
MavESP8266Component::_sendParameter(MavESP8266Bridge* sender, const char* id, uint32_t value, uint16_t index)
{
    //-- Build message
    mavlink_param_value_t msg;
    msg.param_count = MavESP8266Parameters::ID_COUNT;
    msg.param_index = index;
    strncpy(msg.param_id, id, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
    memcpy(&msg.param_value, &value, sizeof(uint32_t));
    msg.param_type = MAV_PARAM_TYPE_UINT32;
    mavlink_message_t mmsg;
    mavlink_msg_param_value_encode_chan(
        getWorld()->getVehicle()->systemID(),
        MAV_COMP_ID_UDP_BRIDGE,
        sender->_send_chan,
        &mmsg,
        &msg
    );
    sender->sendMessage(&mmsg);
}








//---------------------------------------------------------------------------------
//-- Handle Commands
void
MavESP8266Component::_handleCmdLong(MavESP8266Bridge* sender, mavlink_command_long_t* cmd, uint8_t compID)
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
    } else if(getWorld()->getParameters()->getRawEnable() && cmd->command == MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN) {
        //-- Reset "Companion Computer"
        if((uint8_t)cmd->param2 == 1) {
            result = MAV_RESULT_ACCEPTED;
            reboot = true;
        }

        // recognize FC reboot command and switch to raw mode for bootloader protocol to work
        if(compID == MAV_COMP_ID_ALL && (uint8_t)cmd->param1 > 0) {
          getWorld()->getLogger()->log("Raw mode enabled (cmd %d %d)\n", cmd->command, compID);
          _in_raw_mode = true;
          _in_raw_mode_time = 0;
        }
    }
    //-- Response
    if(compID == MAV_COMP_ID_UDP_BRIDGE) {
        mavlink_message_t msg;
        mavlink_msg_command_ack_pack_chan(
            getWorld()->getVehicle()->systemID(),
            //_forwardTo->systemID(),
            MAV_COMP_ID_UDP_BRIDGE,
            sender->_send_chan,
            &msg,
            cmd->command,
            result,
            0,0,0,0
        );
        sender->sendMessage(&msg);
    }
    if(reboot) {
        _wifiReboot(sender);
    }
}


//---------------------------------------------------------------------------------
//-- Reboot
void
MavESP8266Component::_wifiReboot(MavESP8266Bridge* sender)
{
    _sendStatusMessage(sender, MAV_SEVERITY_NOTICE, "Rebooting WiFi Bridge.");
    delay(50);
    ESP.reset();
}
