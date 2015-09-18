/*
  =======================================================
  HBus - Home automation/IoT application protocol 
	
  Copyright (C) 2014 Vincenzo Mennella (see license.txt)

  History
    0.2.2 13/06/2014: First public code release
    0.2.3 02/01/2015: Library version & check txenable
  =======================================================
*/

#ifndef HBUS_SERIAL_PORT_H
#define HBUS_SERIAL_PORT_H
#define LIBRARY_VERSION_HBUS_SERIAL_PORT_H   "0.2.3"

//-------------------------------------------------------
//External includes and global defines
//-------------------------------------------------------
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include "HardwareSerial.h" 
#include <HbusPort.h> 
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
#include <Streaming.h> 
#endif

class HBusSerialPort : public HBusPort
{
  private:
    HardwareSerial *_port;
    uint8_t _txe;
  public:
    HBusSerialPort(HardwareSerial *port, uint8_t txEnable = 0) 
    {
      _port = port;
      _txe = txEnable;
    }
    void begin() 
    {
      if (_txe > 0) {
        pinMode(_txe, OUTPUT);
        digitalWrite(_txe, LOW); 
      }
    }
    void write(uint8_t* buffer, uint16_t length) 
    {
 // #if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
      // Serial << "write " << length << " txen " << _txe << endl;
      // for(uint8_t i = 0; i< length;i++)
        // Serial << "  [" << i << "] = " << buffer[i] << " - " << _HEX(buffer[i]) << " : " << (char) buffer[i] << endl;
 // #endif
      //tx enable for RS485 bus (MASTER)
      if (_txe > 0) { 
        digitalWrite(_txe, HIGH);
        //delay(20);
      }
      _port->write(buffer, length);
      _port->flush();

      //rx enable for RS485 bus (MASTER)
      if (_txe > 0) { 
        //delay(20);
        digitalWrite(_txe, LOW);
      }
    }
    int dataAvailable()
    {
      return _port->available();
    }
    uint8_t read()
    {
      uint8_t value = _port->read();
      crc.updateCrc(value);
      return value;
    }
    void clear()
    {
      while (_port->available()) {
        _port->read();
        delay(1);
      }
  }
};
#endif