/*
  =======================================================
  HBus - Home automation/IoT application protocol 
	
  HBusSerialPort: Serial implementation (rs232 & rs485) 
  
  Copyright (C) 2014-2015 Vincenzo Mennella (see license.txt)

  History
    1.0.0 13/06/2014: First code release
    1.0.1 16/08/2015: Library version & check txenable
  =======================================================
*/

#ifndef HBUS_SERIAL_PORT_H
#define HBUS_SERIAL_PORT_H
#define VERSION_HBUS_SERIAL_PORT_H   "1.0.1"

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
  
    // -------------------------------------------------------
    // HBusSerialPort constructor
    // Parameters: 
    //  port          Hardware serial port
    //  txEnable      RS485 TxEnable pin
    //
    // Returns:	      none
    // -------------------------------------------------------
    HBusSerialPort(HardwareSerial *port, uint8_t txEnable = 0) 
    {
      _port = port;
      _txe = txEnable;
    }
    // -------------------------------------------------------
    // Initialize port
    // Parameters: 
    //  none
    //
    // Returns:	      
    //  none
    // -------------------------------------------------------
    void begin() 
    {
      if (_txe > 0) {
        pinMode(_txe, OUTPUT);
        digitalWrite(_txe, LOW); 
      }
    }
    // -------------------------------------------------------
    // Write buffer to serial port
    // Parameters: 
    //  buffer        Buffer array pointer
    //  length        Length of buffer array
    //
    // Returns:	      none
    // -------------------------------------------------------
    void write(uint8_t* buffer, uint16_t length) 
    {
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
    // -------------------------------------------------------
    // return number of available bytes
    // Parameters: 
    //  none
    //
    // Returns:	      
    //  number of available bytes in rx buffer
    // -------------------------------------------------------
    int dataAvailable()
    {
      return _port->available();
    }
    // -------------------------------------------------------
    // Read next available received byte
    //  and updates crc engine
    // Parameters: 
    //  none
    //
    // Returns:	      
    //  byte read from rx buffer
    // -------------------------------------------------------
    uint8_t read()
    {
      uint8_t value = _port->read();
      crc.updateCrc(value);
      delay(1);
      return value;
    }
    // -------------------------------------------------------
    // Clear rx buffer and crc engine
    // Parameters: 
    //  none
    //
    // Returns:	      
    //  none
    // -------------------------------------------------------
    void clear()
    {
      while (_port->available()) {
        _port->read();
        delay(1);
      }
      crc.clearCrc();
  }
};
#endif