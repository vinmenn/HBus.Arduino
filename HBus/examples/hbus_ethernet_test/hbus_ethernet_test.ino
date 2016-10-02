/*
  =======================================================
  HBus - Home automation/IoT application protocol 
	
  HBus - Bus controller ethernet test program

  This example uses ethernet shield for HBus communications
  And serial port for debug messages

  Copyright (C) 2014 Vincenzo Mennella (see license.txt)

  History
    1.0.0 25/08/2015:   Code created from serial_test example
  =======================================================
*/

//-----------------------------------------------------------------------------
// Global defines
//-----------------------------------------------------------------------------
#define DEBUG               1   // 0 No debug, 1 serial, 2 lcd, 3 led
#define DEBUG_PORT          Serial
#define DEFAULT_WIDTH       1   // Address width
#define DEFAULT_WIDTH       1   // Address width
#define NODE_ADDRESS        2   // HBus address
#define HW_TYPE             HW_ARDUINO_UNO //Only for allocating resources
#define USE_WATCHDOG        0
#define LED_PIN             13

//-----------------------------------------------------------------------------
// External libraries
//----------------------------------------------------------------------------- 
#include <EEPROM.h>
#include <SPI.h>
#include <Ethernet.h> 
#include <Streaming.h> 
//-----------------------------------------------------------------------------
//HBus libraries
//-----------------------------------------------------------------------------
#include <HBusDefines.h>
#include <HBus.h> 	 
#include <HBusPort.h> 	
#include <HBusTcpPort.h> 	 
#include <Crc16.h> 	
#include <SimpleStack.h>

//Ethernet port
byte mac[] = { 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F };
IPAddress ip(192,168,1,100);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255, 255, 255, 0); 
IPAddress host(192,168,1,2);        //Send messages to host node
HBusPort *port = new HBusTcpPort(host, HBUS_TCP_SEND, HBUS_TCP_LISTEN);  
HBus *bus = new HBus(port);

byte data[] = "LS201";
byte data2[] = "LS101";
byte data3[] = "LS301";
unsigned short length = 5;
//Ack
unsigned short ackid;
byte ackdata[] = {0,0,0,1};
unsigned short acklength = 4;
byte rx_ack = false;
byte rx_cmd = false;
hb_address_t dest = 2;

void setup() {
#if defined(DEBUG) && DEBUG == 1 //DEBUG on SERIAL
  //Debug port
  DEBUG_PORT.begin(57600);
#endif
  // start the Ethernet connection:
  Ethernet.begin(mac, ip);
  // give the Ethernet shield a second to initialize:
  delay(1000);
#if defined(DEBUG) && DEBUG == 1 //DEBUG on SERIAL
  DEBUG_PORT << "Connecting..." << endl;
#endif

  bus->receivedCommand = cmdReceived;
  bus->begin(NODE_ADDRESS); //Address
  
#if defined(DEBUG) && DEBUG == 1 //DEBUG on SERIAL
  DEBUG_PORT << "HBus controller ethernet test program" << endl;
#endif
 #if defined(DEBUG) && DEBUG == 3 //DEBUG on LED
   pinMode(LED_PIN, OUTPUT);
   blink(3);
#endif
  delay(1000);
}

void loop() {
      //Check hbus commands
      bus->checkMessages();

  if (DEBUG_PORT.available() > 0){
    
    rx_ack = false;
    rx_cmd = false;
    
#if defined(DEBUG) && DEBUG == 1 //DEBUG on SERIAL
    switch(DEBUG_PORT.read())
    {
      //SINGLE ADDRESS NORMAL COMMANDS - SINGLE PORT
      case '0':
        DEBUG_PORT << "send normal command without address (point to point connection)" << endl;
        bus->sendCommand(CMD_PING, 0);
        if (rx_ack)
          DEBUG_PORT << "Received ack from CMD_PING" << endl;
        break;
      case '1':
        DEBUG_PORT << "send normal command with address" << endl;
        bus->sendCommand(CMD_PING, dest, 0);
        if (rx_ack)
          DEBUG_PORT << "Received ack from CMD_PING" << endl;
        break;
      case '2':
        DEBUG_PORT << "send normal command with destination address and payload" << endl;
        bus->sendCommand(CMD_ACTIVATE, dest, data, length, 0);
        if (rx_ack)
          DEBUG_PORT << "Received ack from CMD_ACTIVATE" << endl;
        break;
      //BROADCAST ADDRESS NORMAL COMMANDS
      case '3':
        DEBUG_PORT << "send normal command BROADCAST" << endl;
        // port index doesn't care if destination is BROADCAST_ADDRESS
        bus->sendCommand(CMD_PING, BROADCAST_ADDRESS, 0); 
        if (rx_ack)
          DEBUG_PORT << "wrong ack from BROADCAST_ADDRESS" << endl;
        break;
      case '4':
        DEBUG_PORT << "send normal command BROADCAST with payload" << endl;
        // port index doesn't care if destination is BROADCAST_ADDRESS
        if (rx_ack)
          DEBUG_PORT << "wrong ack from BROADCAST_ADDRESS" << endl;
        bus->sendCommand(CMD_ACTIVATE, BROADCAST_ADDRESS, data, length, 0);
        break;
      //SINGLE ADDRESS IMMEDIATE COMMANDS - SINGLE PORT
      case '5':
        DEBUG_PORT << "send immediate command without address (point to point connection)" << endl;
        bus->sendImmediate(CMD_PING, 0);
        if (rx_ack)
          DEBUG_PORT << "wrong ack from immediate command" << endl;
        break;
      case '6':
        DEBUG_PORT << "send immediate command with address" << endl;
        bus->sendImmediate(CMD_PING, dest , 0);
        if (rx_ack)
          DEBUG_PORT << "wrong ack from immediate command" << endl;
        break;
      case '7':
        DEBUG_PORT << "send immediate command with destination address and payload" << endl;
        bus->sendImmediate(CMD_ACTIVATE, dest, data, length, 0);
        if (rx_ack)
          DEBUG_PORT << "wrong ack from immediate command" << endl;
        break;
      //BROADCAST ADDRESS IMMEDIATE COMMANDS
      case '8':
        DEBUG_PORT << "send immediate command BROADCAST" << endl;
        // port index doesn't care if destination is BROADCAST_ADDRESS
        bus->sendImmediate(CMD_PING, BROADCAST_ADDRESS, 0); 
        if (rx_ack)
          DEBUG_PORT << "wrong ack from immediate BROADCAST command" << endl;
        break;
      case '9':
        DEBUG_PORT << "send immediate command BROADCAST with payload" << endl;
        // port index doesn't care if destination is BROADCAST_ADDRESS
        bus->sendImmediate(CMD_ACTIVATE, BROADCAST_ADDRESS, data, length, 0);
        if (rx_ack)
          DEBUG_PORT << "wrong ack from immediate BROADCAST command" << endl;
        break;
      case 'a':
        DEBUG_PORT << "send ack response with payload data" << endl;
        bus->sendAck(CMD_PING, dest, ackid, ackdata, acklength, 0);
        if (rx_ack)
          DEBUG_PORT << "wrong ack from ack response" << endl;
        break;
      case 'b':
        DEBUG_PORT << "send nack response with payload data" << endl;
        bus->sendNack(CMD_PING, dest, ackid, ERR_UNKNOWN, 0);
        if (rx_ack)
          DEBUG_PORT << "wrong ack from ack response" << endl;
        break;
      case 'c':
        bus->clearErrors();
        break;
      case 'd':
        bus->setError(ERR_UNKNOWN);
        break;
      case 'e':
        bus->resetError();
        break;
      case 'f':
        DEBUG_PORT << "bus error : " << bus->getError() << " of " << bus->totalErrors() << endl;
        break;
      case 'g':
        DEBUG_PORT << "bus status : " << bus->getStatus() << endl;
        break;
      case 'h':
        DEBUG_PORT << "send immediate remote node" << endl;
        // port index doesn't care if destination is BROADCAST_ADDRESS
        bus->sendImmediate(CMD_ACTIVATE, 1, data2, length, 0);
        if (rx_ack)
          DEBUG_PORT << "wrong ack from immediate command" << endl;
        break;
      case 'i':
        DEBUG_PORT << "send immediate BROADCAST remote node" << endl;
        // port index doesn't care if destination is BROADCAST_ADDRESS
        bus->sendImmediate(CMD_ACTIVATE, BROADCAST_ADDRESS, data2, length, 0);
        if (rx_ack)
          DEBUG_PORT << "wrong ack from immediate BROADCAST command" << endl;
        break;
      case 'j':
        DEBUG_PORT << "send immediate BROADCAST remote node" << endl;
        // port index doesn't care if destination is BROADCAST_ADDRESS
        bus->sendImmediate(CMD_ACTIVATE, 4, data2, length, 0);
        if (rx_ack)
          DEBUG_PORT << "wrong ack from immediate BROADCAST command" << endl;
        break;
      case 'k':
        DEBUG_PORT << "send immediate BROADCAST remote node" << endl;
        // port index doesn't care if destination is BROADCAST_ADDRESS
        bus->sendImmediate(CMD_ACTIVATE, BROADCAST_ADDRESS, data3, length, 0);
        if (rx_ack)
          DEBUG_PORT << "wrong ack from immediate BROADCAST command" << endl;
        break;
    }
  }
#endif
}

// -------------------------------------------------------
// Message received handler
// Parameters: 
//  message:  HBus message received through port
//  port:     HBus port index
// Returns:	      
//  true if message correctly processed
// -------------------------------------------------------
uint8_t cmdReceived(hb_message_t message, uint8_t port) {
#if defined(DEBUG) && DEBUG == 1 //DEBUG on SERIAL
  DEBUG_PORT << "Received command " << message.command << endl
             << "\tfrom " << message.source << endl
             << "\tflags " << message.flags << endl
             << "\tdata length " << message.length << endl
             << "\tport " << port << endl;
#endif
  
  if ((message.flags & 0x03) == ACK_MSG)
    rx_ack = true;
    
  if ((message.flags & 0x03) == NORMAL_MSG || (message.flags & 0x03) == IMMEDIATE_MSG)
    rx_cmd = true;
    
  return true;
}

