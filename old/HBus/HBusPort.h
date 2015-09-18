/*
  =======================================================
  HBus - Home automation/IoT application protocol 
	
  Copyright (C) 2014 Vincenzo Mennella (see license.txt)

  History
    0.2.2 13/06/2014:   First public code release
  =======================================================
*/

#ifndef HBUS_PORT_H
#define HBUS_PORT_H

//-------------------------------------------------------
//External includes and global defines
//-------------------------------------------------------
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include <Crc16.h>

class HBusPort
{
    public:
        virtual void begin() {};
		virtual void write(uint8_t* buffer, uint16_t length) {};
		virtual int dataAvailable() { return 0;};
		virtual uint8_t read() { return 0;};
		virtual void clear() {};
		Crc16 crc;							    // Crc16 calculation
};
#endif