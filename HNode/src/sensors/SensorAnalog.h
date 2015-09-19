/*
  =======================================================
  HBus - Home automation/IoT application protocol 
  
  Sensors classes
  
  Copyright (C) 2014 Vincenzo Mennella (see license.txt)

  History
    1.0.0 30/08/2014:   First revision of code
    1.0.1 20/08/2015:   New revision of code
  =======================================================
*/
#ifndef HBUS_SENSOR_ANALOG_H
#define HBUS_SENSOR_ANALOG_H
#define VERSION_HBUS_SENSOR_ANALOG_H   "1.0.1"

//-----------------------------------------------------------------------------
// External libraries
//-----------------------------------------------------------------------------
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
//-----------------------------------------------------------------------------
//HBus libraries
//-----------------------------------------------------------------------------
#include <sensors/sensor.h> 
#include <scheduler.h>
#include <simpleStack.h>

class SensorAnalog : public Sensor {
  private:
    uint8_t _pin;
    float _factor;
    
  public:
    //========================================================
    //Constructors
    //========================================================
    SensorAnalog(SimpleStack *ss);
    SensorAnalog(char* name, uint8_t pin, char* hardware, char* unit, float minRange, float maxRange, float scale);
    //========================================================
    //Methods
    //========================================================
    void begin();
    char* status();
    void read(hb_sensor_read_t* value);
    void deserializeConfig(SimpleStack *ss);
    void serializeConfig(SimpleStack *ss);
};
// -------------------------------------------------------
// Default constructor with stack data
// -------------------------------------------------------
SensorAnalog::SensorAnalog(SimpleStack *ss) {
    //Read variable configuration
    deserializeConfig(ss);
}

// -------------------------------------------------------
//Default constructor with parameters
// -------------------------------------------------------
SensorAnalog::SensorAnalog(char* name, uint8_t pin, char* hardware, char* unit, float minRange, float maxRange, float scale) {

  //Set variable configuration
  strcpy(info.name, name);
  _pin = pin;
  itoa(pin, info.source, 10);
  strcpy(info.hardware, hardware);
  strcpy(info.unit, unit);
  info.minRange = minRange;
  info.maxRange = maxRange;
  info.scale = scale;
  _factor = (info.maxRange - info.minRange)/ (MAX_ANALOG - MIN_ANALOG) * info.scale;
}
// -------------------------------------------------------
// Init sensor
// -------------------------------------------------------
void SensorAnalog::begin() {
  pinMode(_pin, INPUT);  
}

// -------------------------------------------------------
// Return sensor status
// -------------------------------------------------------
char* SensorAnalog::status() { return ""; }
// -------------------------------------------------------
// Read sensor value
// -------------------------------------------------------
void SensorAnalog::read(hb_sensor_read_t* value) {
  (*value).value = analogRead(_pin) * _factor + info.minRange;
  (*value).time = scheduler->getTime();
  strcpy((*value).name, info.name);
  
  if (controller!=0)
    controller->onSensorRead(*value);
}
// -------------------------------------------------------
// de-serialize configuration
// from stack
// -------------------------------------------------------
void SensorAnalog::deserializeConfig(SimpleStack *ss) {
  ss->popName(info.name);
  ss->popName(info.source);
  _pin = (uint8_t) atoi(info.source);
  ss->popName(info.hardware);
  ss->popName(info.unit);
  info.minRange = ss->popFloat();
  info.maxRange = ss->popFloat();
  info.scale = ss->popFloat();
  _factor = (info.maxRange - info.minRange)/ (MAX_ANALOG - MIN_ANALOG) * info.scale;
}
// -------------------------------------------------------
// serialize configuration
// into stack
// -------------------------------------------------------
void SensorAnalog::serializeConfig(SimpleStack *ss) {
  //Push data
  ss->pushName(info.name);
  ss->pushName(info.source);
  ss->pushName(info.hardware);
  ss->pushName(info.unit);
  ss->pushFloat(info.minRange);
  ss->pushFloat(info.maxRange);
  ss->pushFloat(info.scale);
}

#endif