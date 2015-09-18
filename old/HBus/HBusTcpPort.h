/*
  =======================================================
  HBus - Home automation/IoT application protocol 
	
  Copyright (C) 2014 Vincenzo Mennella (see license.txt)

  History
    0.2.2 13/06/2014:   First public code release
    0.2.3 05/01/2015:   Revision of client instance
  =======================================================
*/

#ifndef HBUS_TCP_PORT_H
#define HBUS_TCP_PORT_H

#define HBUS_TCP_SEND   5000
#define HBUS_TCP_LISTEN 5001

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
            
  public:
    HBusTcpPort(IPAddress host, int sendPort, int receivePort) {
      _host = host;
      _server = new EthernetServer(receivePort);
      //_client = 0;
      _hostPort = sendPort;
    }   
    
    void begin() 
    {
      _server->begin();
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
    Serial << "tcp begin = " << _host << ":" << _hostPort << endl;
#endif
      _client.connect(_host, _hostPort);
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
    Serial << "client " << _client.connected() << endl;
    delay(200);
#endif
    }
    
		void write(uint8_t* buffer, uint16_t length) 
    {
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
      Serial << "tx to tcp " << length << " " << _host << ":" << _hostPort << endl;
#endif

      if (!_client.connected()) {
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
    Serial << "client " << _client.connected() << endl;
    delay(200);
#endif
        _client.connect(_host, _hostPort);
      }
      if (_client.connected())
      _client.write(buffer, length);
    }
    
		int dataAvailable()
    {
      if (!_server) return 0;
      
      EthernetClient client = _server->available();
      uint8_t data = 0;
      
      if (client && client.connected()) {
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
    if (client && client.available()>0)
      Serial << "rx from tcp " << client.available() << endl;
#endif
      
        data = client.available();
      }
              
      return data;
    }
    
		uint8_t read()
    {
      if (!_server) return 0;
      
      EthernetClient client = _server->available();
        
      uint8_t data = 0;
      
      if (client) {
        if (client.connected()) {
          data = client.read();
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
          Serial << "read from tcp " << data << endl;
#endif
        }
        else {
          client.stop();
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
      Serial << "tcp close" << endl;
#endif
        }
      }
        
      return data;
    }
    
		void clear()
    {
      if (!_server) return;
      
      EthernetClient client = _server->available();
      
      if (client) {
        if (client.connected()) {
          client.flush();
        } else {
          client.stop();
        }
      }
    }
};
#endif