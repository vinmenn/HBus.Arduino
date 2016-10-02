/*
  =======================================================
  HBus - Home automation/IoT application protocol 
  
  Shutter device
  
  Copyright (C) 2014 Vincenzo Mennella (see license.txt)

  History
    1.0.0 10/08/2014:   First revision of separated code
    1.0.1 01/09/2015:   Review code for github publication
  =======================================================
*/
#ifndef HBUS_SHUTTER_DEVICE_H
#define HBUS_SHUTTER_DEVICE_H
#define VERSION_HBUS_SHUTTER_DEVICE_H   "1.0.1"

//-----------------------------------------------------------------------------
// External libraries
//-----------------------------------------------------------------------------
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
#include <Streaming.h> 
#endif
//-----------------------------------------------------------------------------
//HBus libraries
//-----------------------------------------------------------------------------
#include <HBus.h>
#include <devices/device.h>
#include <SimpleStack.h>

static const char SHUTTER_STOP[] PROGMEM = "stop"; 
static const char SHUTTER_OPEN[] PROGMEM = "open"; 
static const char SHUTTER_CLOSE[] PROGMEM = "close"; 

//-----------------------------------------------------------------------------
// Shutter device implementation
// Utilizes 2 pins for Up/down directions
// Recognized commands:
//  open: activate <openPin> for <delay> seconds
//  close: activate <closePin> for <delay> seconds
//  stop: stop both motors
//-----------------------------------------------------------------------------
class ShutterDevice : public Device {
	private:
    uint8_t _open;
    uint8_t _close;
    uint32_t _delay;
    hb_device_event_t _evt;
    char _status[MAX_STRING];
    
    //Action stop is the only action scheduled
    hb_device_action_t _action_stop;
        
    public:
        ShutterDevice(char* name, uint8_t openPin, uint8_t closePin, uint32_t delay);
        void begin(); 
        char* status();
        uint8_t executeAction(hb_device_action_t *action);
        void reset();
        uint8_t isActive();
        void deserializeConfig(SimpleStack *ss);
        void serializeConfig(SimpleStack *ss);
};
//-----------------------------------------------------------
// Default constructor
// Parameters:
//  name: device name
//  openPin: pin number for open motor (up direction)
//  closePin: pin number for close motor (down direction)
//  delay: activation time in seconds
//-----------------------------------------------------------
ShutterDevice::ShutterDevice(char* name, uint8_t openPin, uint8_t closePin, uint32_t delay) {
  _open = openPin;
  _close = closePin;  
  _delay = delay;
  strcpy(info.name, name);
  info.actions = 3;
  strcpy_P(info.action[0], SHUTTER_STOP); //Stop
  strcpy_P(info.action[1], SHUTTER_OPEN); //Open
  strcpy_P(info.action[2], SHUTTER_CLOSE); //Close
  //Event info  
  strcpy(_evt.device, name);
  strcpy_P(_evt.event, PSTR(""));
  _evt.values = 0;
  //Default action
  strcpy(_action_stop.action, info.action[0]);   
  _action_stop.values = 0;
}
//-----------------------------------------------------------
// Initialize shutter device
// Parameters:  none
// Returns: none
//-----------------------------------------------------------
void ShutterDevice::begin() 
{
  pinMode(_open, OUTPUT);   
  pinMode(_close, OUTPUT);   
  digitalWrite(_open, LOW);
  digitalWrite(_close, LOW);
  //Status
  strcpy_P(_status, SHUTTER_STOP); //Stop
}
//-----------------------------------------------------------
// Return device status
// Parameters:  none
// Returns: device status string
//-----------------------------------------------------------
char* ShutterDevice::status()
{
  return _status;
}
//-----------------------------------------------------------
// Execute specific device action
// Parameters:  Device action
// Returns: true if action executed
//-----------------------------------------------------------
uint8_t ShutterDevice::executeAction(hb_device_action_t *action)
{
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  //Serial << "action " << action->action << endl;
#endif
    //Read curent status
    uint8_t o = digitalRead(_open);
    uint8_t c = digitalRead(_close);
    //char* action_name = action.action;
    
    //Purge old device scheduled actions 
    scheduler->purge(info.name);
    //Turn off motors
    digitalWrite(_open, LOW);
    digitalWrite(_close, LOW);
    //Open
    if (strcmp(action->action, info.action[1]) == 0) {
        if (o != HIGH) {
            digitalWrite(_open, HIGH);
            scheduler->add(new DeviceSchedule(this, &_action_stop, _delay, false));
        }
        strcpy_P(_status, SHUTTER_OPEN); //Open
    }
    //Close
    else if (strcmp(action->action, info.action[2]) == 0){
        if (c != HIGH) {
            digitalWrite(_close, HIGH);
            scheduler->add(new DeviceSchedule(this, &_action_stop, _delay, false));
        }
        strcpy_P(_status, SHUTTER_CLOSE); //Close
    }
    //Stop
    else if (strcmp(action->action, info.action[0]) == 0){
      //Motors are already stopped
      strcpy_P(_status, SHUTTER_STOP); //Stop
    }
    else {
      //Action not recognized
      return false;
    }
    
    strcpy(_evt.event, action->action);
    _evt.time = 0; //TODO: add Scheduler time
    
    //Call HNode controller
    if (controller != 0)
      controller->onDeviceEvent(&_evt);
      
    return true;
}

//-----------------------------------------------------------
// Reset device status
//-----------------------------------------------------------
void ShutterDevice::reset() 
{
    //Turn off motors
    digitalWrite(_open, LOW);
    digitalWrite(_close, LOW);
    strcpy_P(_status, SHUTTER_STOP); //Emergency stop
}
//-----------------------------------------------------------
// Check if device is active
// Parameters:  none
// Returns: true if device is active (motors activated)
//-----------------------------------------------------------
uint8_t ShutterDevice::isActive()
{
    uint8_t o = digitalRead(_open);
    uint8_t c = digitalRead(_close);
    
    return ((o == HIGH) || (c == HIGH));
}
//-----------------------------------------------------------
// de-serialize configuration from stack
//-----------------------------------------------------------
void ShutterDevice::deserializeConfig(SimpleStack *ss) {

  //Device name
  ss->popName(info.name);
  //Shutter pins
  _open      = ss->popByte();
  _close      = ss->popByte();
  //Delay
  _delay      = ss->popLong();
}
//-----------------------------------------------------------
// serialize device configuration into stack
//-----------------------------------------------------------
void ShutterDevice::serializeConfig(SimpleStack *ss) {
    //Device name
    ss->pushName(info.name);
    //Shutter pins
    ss->pushByte(_open);
    ss->pushByte(_close);
    //Delay
    ss->pushLong(_delay);
}
//-----------------------------------------------------------

#endif