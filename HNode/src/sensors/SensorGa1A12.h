/*
  =======================================================
  HBus - Home automation/IoT application protocol 
  
  Sensors classes
  
  Copyright (C) 2014-15-16 Vincenzo Mennella (see license.txt)

  History
    1.0.0 10/01/2015:   First code based on these articles: 
      https://learn.adafruit.com/downloads/pdf/adafruit-ga1a12s202-log-scale-analog-light-sensor.pdf
      https://learn.adafruit.com/adafruit-ga1a12s202-log-scale-analog-light-sensor/use-it
    1.0.1 20/08/2015:   New revision of code
    1.1.0 31/10/2015:   Added sensor read struct and interval parameter
  =======================================================
*/
#ifndef HBUS_SENSOR_GA1A12_H
#define HBUS_SENSOR_GA1A12_H
#define VERSION_HBUS_SENSOR_GA1A12_H   "1.1.0"

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


class SensorGa1A12 : public Sensor {
  private:
    const float GA1A12_RAW = 1024.0f;		// ADC max value
    const float GA1A12_LOG = 5.0f;		// 10^5 value
    uint8_t _pin;
        
    //===========================================================
    //Methods
    //===========================================================
    void init();
      
  public:
    //===========================================================
    //Constructors
    //===========================================================
    SensorGa1A12(SimpleStack *ss);
    SensorGa1A12(char* name, uint8_t pin, uint16_t interval);
    //===========================================================
    //Methods
    //===========================================================
    char* status();
    void read(hb_sensor_read_t* value);
    void deserializeConfig(SimpleStack *ss);
    void serializeConfig(SimpleStack *ss);
};

// -------------------------------------------------------
// Init sensor with stack configuration
// -------------------------------------------------------
SensorGa1A12::SensorGa1A12(SimpleStack *ss) {

  //Read variable configuration
  deserializeConfig(ss);
  
  //Set sensor fixed information
  init();
}

// -------------------------------------------------------
// Init sensor with fixed configuration
// -------------------------------------------------------
SensorGa1A12::SensorGa1A12(char* name, uint8_t pin, uint16_t interval) {

  //Set variable configuration
  strcpy(info.name, name);
  _pin = pin;
  
  //Set sensor fixed information
  init();
  info.interval = interval;
}
// -------------------------------------------------------
// Return status (empry)
// -------------------------------------------------------
char* SensorGa1A12::status() { return ""; }

// -------------------------------------------------------
// Read GA1A12 raw value and convert to lux
// -------------------------------------------------------
void SensorGa1A12::read(hb_sensor_read_t* value) {

  float loglux = analogRead(_pin) * GA1A12_LOG / GA1A12_RAW;
  value->value = pow(10, loglux);
    
  value->time = scheduler->getTime();
  strcpy(value->name, info.name);
  
  if (controller!=0)
    controller->onSensorRead(value);
}
// -------------------------------------------------------
// de-serialize configuration
// from stack
// -------------------------------------------------------
void SensorGa1A12::deserializeConfig(SimpleStack *ss) {

  ss->popName(info.name);
  _pin = ss->popByte();
}
// -------------------------------------------------------
// serialize configuration
// into stack
// -------------------------------------------------------
void SensorGa1A12::serializeConfig(SimpleStack *ss) {
  //Push data
  ss->pushName(info.name);
  ss->pushByte(_pin);
}
// -------------------------------------------------------
// Initialize sensor
// -------------------------------------------------------
void SensorGa1A12::init() {
  strcpy_P(info.source, PSTR(""));
  strcpy_P(info.hardware, PSTR("GA1A12"));
  strcpy_P(info.unit, PSTR("lux"));
  info.minRange = 0.0f;
  info.maxRange = 55000.0f;
  info.scale = 1.0f;
  info.function = Logarithmic;
}
#endif