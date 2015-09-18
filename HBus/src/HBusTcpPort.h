/*
  =======================================================
  HBus - Home automation/IoT application protocol 
  
   HBusTcpPort: Tcp/ip implementation
  
  Copyright (C) 2014-2015 Vincenzo Mennella (see license.txt)

  History
    1.0.0 13/06/2014:   First code release
    1.0.1 05/01/2015:   Revision of client instance
  =======================================================
*/

#ifndef HBUS_TCP_PORT_H
#define HBUS_TCP_PORT_H
#define VERSION_HBUS_TCP_PORT_H   "1.0.1"

#define HBUS_TCP_SEND   5000    //Send port
#define HBUS_TCP_LISTEN 5001    //Receive port

//-------------------------------------------------------
//External includes and global defines
//-------------------------------------------------------
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <SPI.h> 
#include <Ethernet.h> 
#include <HbusPort.h> 

#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
#include <Streaming.h> 
#endif

class HBusTcpPort : public HBusPort
{
  private:
    // Initialize the Ethernet client library
    // with the IP address and port of the server 
    EthernetClient _client;
    EthernetServer *_server;
    IPAddress _host;
    int _hostPort;
    uint8_t _buf[MAX_BUFFER];
    int _rd;
    int _length;
    
  public:
  
    // -------------------------------------------------------
    // HBusTcpPort constructor
    // Parameters: 
    //  host          IPAddress of host node
    //
    // Returns:	      none
    // -------------------------------------------------------
  HBusTcpPort(IPAddress host) {
      _host = host;
      _server = new EthernetServer(HBUS_TCP_LISTEN);
      //_client = 0;
      _hostPort = HBUS_TCP_SEND;
      _rd = 0;
      _length = 0;
    }   
    // -------------------------------------------------------
    // HBusTcpPort constructor
    // Parameters: 
    //  host          IPAddress of host node
    //  sendPort      send port number (default HBUS_TCP_SEND)
    //  receivePort   receive port number (default HBUS_TCP_LISTEN)
    //
    // Returns:	      none
    // -------------------------------------------------------
  HBusTcpPort(IPAddress host, int sendPort, int receivePort) {
      _host = host;
      _server = new EthernetServer(receivePort);
      //_client = 0;
      _hostPort = sendPort;
      _rd = 0;
      _length = 0;
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
      _server->begin();
    }
    
    // -------------------------------------------------------
    // Write buffer to tcp port
    // Parameters: 
    //  buffer        Buffer array pointer
    //  length        Length of buffer array
    //
    // Returns:	      none
    // -------------------------------------------------------
		void write(uint8_t* buffer, uint16_t length) 
    {
      //EthernetClient client;
      size_t s = 0;
      while(s == 0) {
        if (_client.connect(_host, _hostPort)) {

          s = _client.write(buffer, length);
      
          delay(1);
       }
        else {
          _client.stop();
        }
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
      if (!_server) return 0;
      if (_length > 0) return _length;
      
      EthernetClient client = _server->available();
      
      if (!client) return 0;
    
      _rd = 0;
      
      _length = client.available();
      
      if (_length > 0) {
        if (_length > MAX_BUFFER) _length = MAX_BUFFER;
        
        client.read(_buf, _length);
      }
      delay(1);
      
      // close the connection:
      client.stop();       
      
      return _length;
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
      if (_length == 0) return 0;
      
      _length--;
      
      uint8_t value = _buf[_rd++];
      crc.updateCrc(value);
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
      crc.clearCrc();
    }
};
#endif