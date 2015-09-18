/*
  =======================================================
  HBus - Home automation/IoT application protocol 
  
  Sensors classes
  
  Copyright (C) 2014 Vincenzo Mennella (see license.txt)

  History
    1.0.0 30/08/2014:   First revision of separated code
    1.0.1 30/12/2014:   New revision of code
  =======================================================
*/
#ifndef HBUS_SENSOR_H
#define HBUS_SENSOR_H
#define VERSION_HBUS_SENSOR_H   "1.0.1"

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
#include <scheduler.h>
#include <simpleStack.h>

// #define SENSOR_UNKNOWN        0x00
// #define SENSOR_READY          0x01

// #define SENSOR_ERROR_UNKNOWN  0x80
// #define SENSOR_ERROR_READ     0x81

// Sensor information
typedef struct {
	char name[NAME_LENGTH+1];		//name of sensor
  uint8_t index;              //sensor index
	char source[NAME_LENGTH+1];	//pin source name
	char unit[MAX_STRING];      //unit of measure
	char hardware[MAX_STRING];  //hardware name (eg. ds18b20)
	float minRange;					    //min value of sensor (unit scale)
	float maxRange;					    //max value of sensor (unit scale)
	float scale;					      //scale factor for sensor values
} hb_sensor_info_t;
// Sensor read
typedef struct {
	char name[NAME_LENGTH+1];		//name of sensor
	float value;					      //Sensor value (in sensor unit, eg. Degrees)
	uint32_t time;					    //Read timestamp
} hb_sensor_read_t;


//External handler
extern "C" {
	// callback function types
    typedef uint8_t (*sensorReadCallback)(hb_sensor_read_t sensorRead);
} 
class HBusSensorController
{
   public:
      virtual void onSensorRead(hb_sensor_read_t sensorRead) {};
};

class Sensor {
    private:
        //HBusSensorController *_handler;
    public:
        //-----------------------------------------------------------
        //Methods
        //-----------------------------------------------------------
        virtual void begin() {};
        virtual char* status() {};
        virtual void reset() {};
        virtual void update() {};
        virtual void read(hb_sensor_read_t* value) {};
        virtual void dispose() {}

        //-----------------------------------------------------------
        //Configuration
        //-----------------------------------------------------------
        //de-serialize configuration from stack
        virtual void deserializeConfig(SimpleStack *ss) { }
        // serialize configuration into stack
        virtual void serializeConfig(SimpleStack *ss) { }
        
        //-----------------------------------------------------------
        //Conversions
        //-----------------------------------------------------------

        // toString: print information
        char* toString() { return info.name; };
        
        // serialize sensor information into stack
        void serializeInfo(SimpleStack *ss) {
            ss->pushName(info.name);
            ss->pushByte(info.index);
            ss->pushName(info.source);
            ss->pushString(info.unit);
            ss->pushString(info.hardware);
            ss->pushFloat(info.minRange);
            ss->pushFloat(info.maxRange);
            ss->pushFloat(info.scale);
        }
        
        //-----------------------------------------------------------
        //Properties
        //-----------------------------------------------------------
        hb_sensor_info_t info;
        Scheduler *scheduler;
        HBusSensorController *controller;
        hb_address_t listener;
        uint8_t listenerPort;
        uint8_t interval;
        uint8_t count;
        uint16_t expire;
};
#endif