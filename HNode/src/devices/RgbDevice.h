/*
  =======================================================
  HBus - Home automation/IoT application protocol 
  
  RGB device
  
  Copyright (C) 2014 Vincenzo Mennella (see license.txt)

  History
    0.0.1 10/08/2014:   First revision of separated code
    0.0.2 26/08/2014:   Revision after integration with HbusNode
    0.0.3 22/10/2014:   Revision before git publication
  =======================================================
*/
#ifndef RGB_DEVICE_H
#define RGB_DEVICE_H

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
#include <devices/device.h>

static const char RGB_OFF[] PROGMEM = "off"; 
static const char RGB_ON[] PROGMEM = "on"; 
static const char RGB_FULL[] PROGMEM = "full"; 
static const char RGB_COLOR[] PROGMEM = "set"; 

PGM_P _actions[4] = {RGB_OFF, RGB_ON, RGB_FULL, RGB_COLOR}; 
PGM_P _statuses[3] = {RGB_OFF, RGB_ON, RGB_FULL}; 


//-----------------------------------------------------------------------------
// Rgb device implementation
// Utilizes 3 pins for all colors
// Recognized commands:
//  on: Turn on light with default color
//  off: Turn off light
//  full: set all RGB colors to max value
//  set:  set specific rgb color
//-----------------------------------------------------------------------------
//Rgb device implementation
class RgbDevice : public Device {
	private:
        //Pins
        uint8_t _r_pin;
        uint8_t _g_pin;
        uint8_t _b_pin;
        //Default value
        uint16_t _def_r;
        uint16_t _def_g;
        uint16_t _def_b;
        //Last value set
        uint8_t _r;
        uint8_t _g;
        uint8_t _b;
        uint8_t _invert;
        hb_device_event_t _event; //Device event
        
        char _status[NAME_LENGTH+1];
        
        void setColor(uint16_t r, uint16_t g, uint16_t b);
    public:
    
        RgbDevice(SimpleStack *ss);
        RgbDevice(char* name, uint8_t redPin, uint8_t greenPin, uint8_t bluePin, uint16_t red, uint16_t green, uint16_t blue, uint8_t invert);
        void begin();
        char* status();
        uint8_t executeAction(hb_device_action_t *action);
        void reset();
        uint8_t isActive();
        void deserializeConfig(SimpleStack *ss);
        void serializeConfig(SimpleStack *ss);
        //-----------------------------------------------------------
        //Properties
        //-----------------------------------------------------------
        inline uint8_t redPin() { return _r_pin; }
        inline uint8_t greenPin() { return _g_pin; }
        inline uint8_t bluePin() { return _b_pin; }
        inline uint8_t invert() { return _invert; }
        
};
//-----------------------------------------------------------
// Default constructor with parameters
// Parameters:
//  name: device name
//  redPin: pin for color red
//  greenPin: pin for color green
//  bluePin: pin for color blue
//  red: default red value
//  green: default green value
//  blue: default blue value
//  invert: invert values for colors
//-----------------------------------------------------------
RgbDevice::RgbDevice(char* name, uint8_t redPin, uint8_t greenPin, uint8_t bluePin, 
    uint16_t red, uint16_t green, uint16_t blue, uint8_t invert ) {

    info.actions = 4;
    for(uint8_t i=0;i<info.actions;i++)
        strcpy_P(info.action[i], _actions[i]);
    //Device name
    strcpy(info.name, name);
    //Rgb pins
    _r_pin = redPin;
    _g_pin = greenPin;  
    _b_pin = bluePin;  
    //Default color
    _def_r = red;
    _def_g = green;  
    _def_b = blue;  
    //Invert values
    _invert = invert;
    //Device event fixed parameters
    strcpy(_event.device, info.name);
}
//-----------------------------------------------------------
// Constructor with read configuration from stack
//-----------------------------------------------------------
RgbDevice::RgbDevice(SimpleStack *ss) {
  //Device actions
  info.actions = 4;
  for(uint8_t i=0;i<info.actions;i++)
      strcpy_P(info.action[i], _actions[i]);
  //Read configuration
  deserializeConfig(ss);
}
//-----------------------------------------------------------
// Begin (activate) Rgb device
//-----------------------------------------------------------
void RgbDevice::begin() {
    //Set pins
    pinMode(_r_pin, OUTPUT);  
    pinMode(_g_pin, OUTPUT);  
    pinMode(_b_pin, OUTPUT);
    //Turn off RGB light
    setColor(0, 0, 0);
}
//-----------------------------------------------------------
// Return device status
//-----------------------------------------------------------
char* RgbDevice::status()
{
    //Off
    if (_r == 0 && _g == 0 && _b == 0)
    {
        strcpy_P(_status, _statuses[0]);
        return _status; //off
    }
    //Full
    if (_r == 0xff && _g == 0xff && _b == 0xff)
    {
        strcpy_P(_status, _statuses[2]);
        return _status; //full
    }
    strcpy_P(_status, _statuses[1]);
    return _status; //on
}
//-----------------------------------------------------------
// Execute device action
//-----------------------------------------------------------
uint8_t RgbDevice::executeAction(hb_device_action_t *action)
{
  uint8_t ok = false;

  //Off
  if (strcmp_P(action->action, RGB_OFF) == 0) {
      setColor(0, 0, 0);
      ok = true;
  }
  //On
  if (strcmp_P(action->action, RGB_ON) == 0) {
      setColor(_def_r, _def_g, _def_b);
      ok = true;
  }
  //Full
  if (strcmp_P(action->action, RGB_FULL) == 0) {
      setColor(0x1ff, 0x1ff, 0x1ff);
      ok = true;
  }
  //Set
  if (strcmp_P(action->action, RGB_COLOR) == 0) {
      uint16_t r = action->value[0] << 8 | action->value[1];
      uint16_t g = action->value[2] << 8 | action->value[3];
      uint16_t b = action->value[4] << 8 | action->value[5];
      setColor(r, g, b);
      ok = true;
  }
  if (ok) {
    //Event action
    strcpy(_event.event, action->action);
    _event.time = 0; //TODO: add Scheduler time
    //Current color
    _event.values = 6;
    _event.value[0] = _r >> 8; _event.value[1] = _r & 0xff;
    _event.value[2] = _g >> 8; _event.value[3] = _g & 0xff;
    _event.value[4] = _b >> 8; _event.value[5] = _b & 0xff;
  }
  
  //Call HNode controller
  if (controller != 0)
    controller->onDeviceEvent(&_event);

  return ok;
}
//-----------------------------------------------------------
// Reset device status
//-----------------------------------------------------------
void RgbDevice::reset() 
{
    setColor(0, 0, 0);
}
//-----------------------------------------------------------
//-----------------------------------------------------------
uint8_t RgbDevice::isActive()
{
    return (( _r + _g + _b ) > 0);
}
//-----------------------------------------------------------
// de-serialize configuration from stack
//-----------------------------------------------------------
void RgbDevice::deserializeConfig(SimpleStack *ss) {

    //Device name
    ss->popName(info.name);
    //Rgb pins
_r_pin      = ss->popByte();
_g_pin      = ss->popByte();
_b_pin      = ss->popByte();
    //Default color
_def_r      = ss->popInteger();
_def_g      = ss->popInteger();
_def_b      = ss->popInteger();
    //Invert values
_invert     = ss->popByte();
}
//-----------------------------------------------------------
// serialize device configuration into stack
//-----------------------------------------------------------
void RgbDevice::serializeConfig(SimpleStack *ss) {
    //Device name
    ss->pushName(info.name);
    //Rgb pins
    ss->pushByte(_r_pin);
    ss->pushByte(_g_pin);
    ss->pushByte(_b_pin);
    //Default color
    ss->pushInteger(_def_r);
    ss->pushInteger(_def_g);
    ss->pushInteger(_def_b);
    //Invert values
    ss->pushByte(_invert);
}
//-----------------------------------------------------------
// Set specific color with PWM pins
//-----------------------------------------------------------
void RgbDevice::setColor(uint16_t r, uint16_t g, uint16_t b) {

    _r = r >> 1;
    _g = g >> 1;
    _b = b >> 1;
    if (_invert) {
        analogWrite(_r_pin, 0xff - r);
        analogWrite(_g_pin, 0xff - g);
        analogWrite(_b_pin, 0xff - b);
    }
    else {
        analogWrite(_r_pin, r);
        analogWrite(_g_pin, g);
        analogWrite(_b_pin, b);
    }
}

#endif