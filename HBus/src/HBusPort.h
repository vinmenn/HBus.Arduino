/*
  =======================================================
  HBus - Home automation/IoT application protocol 
	
  Copyright (C) 2014-2015 Vincenzo Mennella (see license.txt)

  History
    1.0.0 13/06/2014:   First code release
  =======================================================
*/

#ifndef HBUS_PORT_H
#define HBUS_PORT_H
#define VERSION_HBUS_PORT_H   "1.0.0"

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
      
      Crc16 crc;							    // Crc16 calculation

      // -------------------------------------------------------
      // Initialize port
      // Parameters: 
      //  none
      //
      // Returns:	      
      //  none
      // -------------------------------------------------------
      virtual void begin() {};
      
      // -------------------------------------------------------
      // Write buffer to serial port
      // Parameters: 
      //  buffer        Buffer array pointer
      //  length        Length of buffer array
      //
      // Returns:	      none
      // -------------------------------------------------------
      virtual void write(uint8_t* buffer, uint16_t length) {};
      
      // -------------------------------------------------------
      // return number of available bytes
      // Parameters: 
      //  none
      //
      // Returns:	      
      //  number of available bytes in rx buffer
      // -------------------------------------------------------
      virtual int dataAvailable() { return 0;};
      
      // -------------------------------------------------------
      // Read next available received byte
      //  and updates crc engine
      // Parameters: 
      //  none
      //
      // Returns:	      
      //  byte read from rx buffer
      // -------------------------------------------------------
      virtual uint8_t read() { return 0;};
      
      // -------------------------------------------------------
      // Clear rx buffer and crc engine
      // Parameters: 
      //  none
      //
      // Returns:	      
      //  none
      // -------------------------------------------------------
      virtual void clear() {};
      
};
#endif