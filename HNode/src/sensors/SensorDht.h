/*
  =======================================================
  HBus - Home automation/IoT application protocol 
  
  SensorDht: temperature/humidity sensor
  
  Copyright (C) 2014-2015 Vincenzo Mennella (see license.txt)

  History
    1.0.0 08/12/2014: First code based on dht22 class
    1.1.0 02/01/2015: Renamed from SensorDht11 to SensorDht
                      Added Library version
                      Added parameter to switch sensor dht11/21/22
    1.1.1 12/08/2015: Library review       
  =======================================================
*/
#ifndef HBUS_SENSOR_DHT_H
#define HBUS_SENSOR_DHT_H
#define VERSION_HBUS_SENSOR_DHT_H   "1.1.1"

//-----------------------------------------------------------------------------
// External libraries
//-----------------------------------------------------------------------------
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include <dht.h>
//-----------------------------------------------------------------------------
//HBus libraries
//-----------------------------------------------------------------------------
#include <sensors/sensor.h> 
#include <scheduler.h>
#include <simpleStack.h>

enum dht_types {
    DHT11_TYPE = 0x00,
    DHT21_TYPE,
    DHT22_TYPE
};

class SensorDht : public Sensor {
  private:
    uint8_t _pin;               //Dht1122 data pin
    uint8_t _temperature;       //1 : read temperature, 0: read humidity
    uint8_t _model;             //0: DHT11
    char _status[MAX_STRING];
    dht _dht;
    
    void init();
  public:
    //========================================================
      //Constructors
    //========================================================
    SensorDht(SimpleStack *ss);
    SensorDht(char* name, uint8_t pin, uint8_t model, uint8_t temperature);
    //========================================================
    //Methods
    //========================================================
    char* status();
    void read(hb_sensor_read_t* value);
    void deserializeConfig(SimpleStack *ss);
    void serializeConfig(SimpleStack *ss);
};

void SensorDht::init() {
  strcpy_P(info.source, PSTR(""));
  switch(_model) 
  {
    case DHT11_TYPE:
      strcpy_P(info.hardware, PSTR("DHT11"));
      break;
    case DHT21_TYPE:
      strcpy_P(info.hardware, PSTR("DHT21"));
      break;
    case DHT22_TYPE:
      strcpy_P(info.hardware, PSTR("DHT22"));
      break;
  }

  if (_temperature) {
    strcpy_P(info.unit, PSTR("C"));
    //From datasheet
    switch(_model) 
    {
      case DHT11_TYPE:
        info.minRange = 0.0;
        info.maxRange = 50.0;
        break;
      case DHT21_TYPE:
        info.minRange = -40.0;
        info.maxRange = 80.0;
        break;
      case DHT22_TYPE:
        info.minRange = -40.0;
        info.maxRange = 80.0;
        break;
    }
  }
  else {
    strcpy_P(info.unit, PSTR("%"));
    //From datasheet
    switch(_model) 
    {
      case 0: //DHT11
        info.minRange = 20.0;
        info.maxRange = 90.0;
        break;
      case 1: //DHT21 
        info.minRange = 0.0;
        info.maxRange = 100.0;
        break;
      case 2: //DHT22
        info.minRange = 0.0;
        info.maxRange = 100.0;
        break;
    }
  }
  info.scale = 1.0;
  strcpy_P(_status, PSTR("ready"));
}
//========================================================
//Constructors
//========================================================
SensorDht::SensorDht(SimpleStack *ss) {
    //Read variable configuration
    deserializeConfig(ss);
    //Set sensor fixed information
    init();
}
SensorDht::SensorDht(char* name, uint8_t pin, uint8_t model, uint8_t temperature) {
    //Set variable configuration
    strcpy(info.name, name);
    _pin = pin;
    _model = model;
    _temperature = temperature;
    //Set sensor fixed information
    init();
}
//========================================================
//Methods
//========================================================
// -------------------------------------------------------
// Return sensor status
// -------------------------------------------------------
char* SensorDht::status() { return _status; }
// -------------------------------------------------------
// Read sensor value
// -------------------------------------------------------
void SensorDht::read(hb_sensor_read_t* value) {
  
  strcpy_P((*value).name, PSTR(""));
  (*value).value = info.minRange;
  (*value).time = 0;
    
  int chk = 0;
  for(uint8_t i = 0; i < 5; i++) {
    switch(_model) 
    {
      case DHT11_TYPE:
        chk = _dht.read11(_pin);
        break;
      case DHT21_TYPE:
        chk = _dht.read21(_pin);
        break;
      case DHT22_TYPE:
        chk = _dht.read22(_pin);
        break;
    }
    if (chk == DHTLIB_OK) {
      if (_temperature)
          (*value).value = _dht.temperature;
      else
          (*value).value = _dht.humidity;
    
      strcpy((*value).name, info.name);
      (*value).time = scheduler->getTime();
      strcpy_P(_status, PSTR("ready"));

      if (controller!=0)
        controller->onSensorRead(*value);
 
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  Serial << "read " << _dht.temperature
       << " " << _dht.humidity << endl;
#endif

      return;
    }
    delay(50);
  }
  //No read from sensor dht
  strcpy_P(_status, PSTR("read error"));
}
// -------------------------------------------------------
// de-serialize configuration
// from stack
// -------------------------------------------------------
void SensorDht::deserializeConfig(SimpleStack *ss) {

  ss->popName(info.name);
  _model = ss->popByte();
  _pin = ss->popByte();
  _temperature = ss->popByte();
}
// -------------------------------------------------------
// serialize configuration
// into stack
// -------------------------------------------------------
void SensorDht::serializeConfig(SimpleStack *ss) {
    //Push data
    ss->pushName(info.name);
    ss->pushByte(_model);
    ss->pushByte(_pin);
    ss->pushByte(_temperature);
}
#endif