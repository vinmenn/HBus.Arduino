/*
  =======================================================
  HBus - Home automation/IoT application protocol 
  
  Sensor ds18b20 temperature class
  
  Copyright (C) 2014 Vincenzo Mennella (see license.txt)

  History
    1.0.0 28/08/2014:   First revision of code
    1.0.1 20/08/2015:   New revision of code
  =======================================================
*/
#ifndef SENSOR_DS18B20_H
#define SENSOR_DS18B20_H
#define VERSION_SENSOR_DS18B20_H   "1.0.1"

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

#include <OneWire.h>
#include <DallasTemperature.h> 
//-----------------------------------------------------------------------------
//HBus libraries
//-----------------------------------------------------------------------------
#include <sensors/sensor.h> 
#include <scheduler.h>
#include <simpleStack.h>

class SensorDs18b20 : public Sensor {
  private:
    uint8_t _pin;
    uint8_t _tempC; // 1 = Celsius , 0 = Farheneit
    char _status[MAX_STRING];
    OneWire *oneWire;
		//Pass our oneWire reference to Dallas Temperature. 
		DallasTemperature *_sensor;
		//Array to hold device address
		DeviceAddress _addr; 		
        
    void init();
    void findSensor();
        
  public:
    //========================================================
    //Constructors
    //========================================================
    SensorDs18b20(SimpleStack *ss);
    SensorDs18b20(char* name, uint8_t pin, uint8_t tempC, DeviceAddress address);
    //========================================================
    //Methods
    //========================================================
    void begin();
    char* status();
    void update();
    void read(hb_sensor_read_t* value);
    void deserializeConfig(SimpleStack *ss);
    void serializeConfig(SimpleStack *ss);
};
//-----------------------------------------------------------
//-----------------------------------------------------------
void SensorDs18b20::init() {
  oneWire = new OneWire(_pin);
  _sensor = new DallasTemperature(oneWire);
    
  strcpy_P(info.source, PSTR(""));
  // info.hardware = (char*) malloc(8);
  strcpy_P(info.hardware, PSTR("DS18B20"));
  // info.unit = (char*) malloc(3);
  
  if (_tempC) {
      strcpy_P(info.unit, PSTR("C"));
      //From datasheet
      info.minRange = -55.0;
      info.maxRange = 125.0;
  }
  else {
      strcpy_P(info.unit, PSTR("F"));
      //From datasheet
      info.minRange = -67.0;
      info.maxRange = 257.0;
  }
  info.scale = 1.0;
}
//-----------------------------------------------------------
//-----------------------------------------------------------
void SensorDs18b20::findSensor() {
  // Method 1:
  // search for devices on the bus and assign based on an index.  ideally,
  // you would do this to initially discover addresses on the bus and then 
  // use those addresses and manually assign them (see above) once you know 
  // the devices on your bus (and assuming they don't change).
  if (!_sensor->getAddress(_addr, 0)) {
            strcpy_P(_status, PSTR("NotFound"));
            for (uint8_t i = 0; i < 8; i++)
                _addr[i] = 0;
  }
        
  // set the resolution to 9 bit
  _sensor->setResolution(_addr, 9);
  strcpy_P(_status, PSTR("Connected"));
}
//========================================================
// Constructors
//========================================================
//-----------------------------------------------------------
//-----------------------------------------------------------
SensorDs18b20::SensorDs18b20(SimpleStack *ss) {

    //Read variable configuration
    deserializeConfig(ss);
    
    //Set sensor fixed information
    init();
}
    
//-----------------------------------------------------------
//-----------------------------------------------------------
SensorDs18b20::SensorDs18b20(char* name, uint8_t pin, uint8_t tempC, DeviceAddress address) {
  //Set variable configuration
  strcpy(info.name, name);
  _pin = pin;
  _tempC = tempC;
  //Copy sensor address
  for (uint8_t i = 0; i < 8; i++)
      _addr[i] = address[i];
  
  //Set sensor fixed information
  init();
}
//========================================================
// Methods
//========================================================
void SensorDs18b20::begin() {
  // Start up the library
  _sensor->begin(); 
  if (_addr[0] == 0 && _addr[1] == 0 && _addr[2] == 0 && _addr[3] == 0 &&
      _addr[4] == 0 && _addr[5] == 0 && _addr[6] == 0 && _addr[7] == 0)
    //Find sensor address
    findSensor();
}
//-----------------------------------------------------------
//-----------------------------------------------------------
char* SensorDs18b20::status() { return _status; }
//-----------------------------------------------------------
//-----------------------------------------------------------
void SensorDs18b20::update() {
  if (_sensor->isConnected(_addr))
      strcpy_P(_status, PSTR("Connected"));
  else
      strcpy_P(_status, PSTR("Disconnected"));
        
}
//-----------------------------------------------------------
// read current value
//-----------------------------------------------------------
void SensorDs18b20::read(hb_sensor_read_t* value) {

  strcpy((*value).name, info.name);
  (*value).time = scheduler->getTime();
    
  // Send the command to get temperatures 
  _sensor->requestTemperatures(); 

  if (_tempC)
      (*value).value = _sensor->getTempC(_addr); 
  else
      (*value).value = _sensor->getTempF(_addr); 
      
  if (controller!=0)
    controller->onSensorRead(*value);
}
//-----------------------------------------------------------
// de-serialize configuration
// from stack
//-----------------------------------------------------------
void SensorDs18b20::deserializeConfig(SimpleStack *ss) {

  ss->popName(info.name);
  _pin   = ss->popByte();
  _tempC = ss->popByte();
  uint16_t l;
  ss->popArray(_addr, &l);
}
//-----------------------------------------------------------
// serialize configuration
// into stack
//-----------------------------------------------------------
void SensorDs18b20::serializeConfig(SimpleStack *ss) {
  //Push data
  ss->pushName(info.name);
  ss->pushByte(_pin);
  ss->pushByte(_tempC);
  ss->pushArray(_addr, 8);
}
#endif