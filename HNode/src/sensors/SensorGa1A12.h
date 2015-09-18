/*
  =======================================================
  HBus - Home automation/IoT application protocol 
  
  Sensors classes
  
  Copyright (C) 2014 Vincenzo Mennella (see license.txt)

  History
    1.0.0 10/01/2015:   First code based on this article: 
      https://learn.adafruit.com/downloads/pdf/adafruit-ga1a12s202-log-scale-analog-light-sensor.pdf
    1.0.1 20/08/2015:   New revision of code
  =======================================================
*/
#ifndef HBUS_SENSOR_GA1A12_H
#define HBUS_SENSOR_GA1A12_H
#define VERSION_SENSOR_GA1A12_H   "1.0.1"

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


const float GA1A12_RAW = 1024.0f;		// ADC max value
const float GA1A12_LOG = 5.0f;		// 10^5 value
class SensorGa1A12 : public Sensor {
    private:
      uint8_t _pin;
        
        // -------------------------------------------------------
        // -------------------------------------------------------
        void init() {
          strcpy(info.source, "");
          strcpy(info.hardware, "GA1A12");
          strcpy(info.unit, "lux");
          info.minRange = 0.0f;
          info.maxRange = 55000.0f;
          info.scale = 1.0f;
        }
      
    public:
        //===========================================================
        //Constructors
        //===========================================================
        // -------------------------------------------------------
        // -------------------------------------------------------
        SensorGa1A12(SimpleStack *ss) {

          //Read variable configuration
          deserializeConfig(ss);
          
          //Set sensor fixed information
          init();
        }
        
        // -------------------------------------------------------
        // -------------------------------------------------------
        SensorGa1A12(char* name, uint8_t pin) {
        
          //Set variable configuration
          strcpy(info.name, name);
          _pin = pin;
          
          //Set sensor fixed information
          init();
        }
        
        // -------------------------------------------------------
        // -------------------------------------------------------
        ~SensorGa1A12() {
            dispose();
        }
        //===========================================================
        //Methods
        //===========================================================
        // -------------------------------------------------------
        // -------------------------------------------------------
        void begin() {
            //NTD
        }
        // -------------------------------------------------------
        // -------------------------------------------------------
        void dispose() {
            //NTD
        }
        
        // -------------------------------------------------------
        // -------------------------------------------------------
        char* status() { return ""; }
        
        // -------------------------------------------------------
        // -------------------------------------------------------
        void reset() {
            //NTD
        }
        // -------------------------------------------------------
        // -------------------------------------------------------
        void update() {
            //NTD
        }
        // -------------------------------------------------------
        // -------------------------------------------------------
        void read(hb_sensor_read_t* value) {
        
          (*value).value = pow(10, analogRead(_pin) * GA1A12_LOG / GA1A12_RAW);
            
          (*value).time = scheduler->getTime();
          strcpy((*value).name, info.name);
        }
        // -------------------------------------------------------
        // deserialize configuration
        // from stack
        // -------------------------------------------------------
        void deserializeConfig(SimpleStack *ss) {
        
          ss->popName(info.name);
          _pin = ss->popByte();
        }
        // -------------------------------------------------------
        // serialize configuration
        // into stack
        // -------------------------------------------------------
        void serializeConfig(SimpleStack *ss) {
          //Push data
          ss->pushName(info.name);
          ss->pushByte(_pin);
        }
};
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
}

public:
//===========================================================
//Constructors
//===========================================================
// -------------------------------------------------------
// -------------------------------------------------------
SensorGa1A12::SensorGa1A12(SimpleStack *ss) {

  //Read variable configuration
  deserializeConfig(ss);
  
  //Set sensor fixed information
  init();
}

// -------------------------------------------------------
// -------------------------------------------------------
SensorGa1A12::SensorGa1A12(char* name, uint8_t pin) {

  //Set variable configuration
  strcpy(info.name, name);
  _pin = pin;
  
  //Set sensor fixed information
  init();
}

//===========================================================
//Methods
//===========================================================

// -------------------------------------------------------
// -------------------------------------------------------
char* SensorGa1A12::status() { return ""; }

// -------------------------------------------------------
// -------------------------------------------------------
void SensorGa1A12::read(hb_sensor_read_t* value) {

  (*value).value = pow(10, analogRead(_pin) * GA1A12_LOG / GA1A12_RAW);
    
  (*value).time = scheduler->getTime();
  strcpy((*value).name, info.name);
  
  if (controller!=0)
    controller->onSensorRead(*value);
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
#endif