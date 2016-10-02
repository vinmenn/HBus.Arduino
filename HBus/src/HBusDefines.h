/*
  =======================================================
  HBus - Home automation/IoT application protocol 

  Constants definitions
   
  Copyright (C) 2014-2015 Vincenzo Mennella (see license.txt)

  History
    1.0.0 31/12/2014:   First code release with last architecture
  =======================================================
*/

#ifndef HBUS_DEFINES_H
#define HBUS_DEFINES_H

//-----------------------------------------------------------------------------
// External libraries
//-----------------------------------------------------------------------------
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

//-------------------------------------------------------
// Max constants
//-------------------------------------------------------
//Hw types:
#define HW_ARDUINO_UNO 1
#define HW_ARDUINO_MEGA 2
#define HW_ARDUINO_MINI 3

#if defined(HW_TYPE) && HW_TYPE == HW_ARDUINO_MEGA
const uint8_t MAX_NODES = 8;		// Max network nodes
const uint8_t MAX_PORTS = 3;		// Max Communication serial ports
const uint8_t MAX_READ = 100;		// Data reception timeout
const uint8_t MAX_BUFFER = 100;     // Max buffer allocation
const uint8_t TX_BUFFER = 100;      // Max buffer allocation
const uint8_t RX_BUFFER = 64;       // Max buffer allocation
const uint8_t MAX_PARAMETERS = 8;	// Max command parameters
const uint8_t MAX_STRING = 16;		// Max string length
const uint8_t MAX_PINS = 32;	    // Max configured node pins
const uint8_t MAX_DEVICES = 6;	    // Max node devices
const uint8_t MAX_SENSORS = 6;	    // Max node sensors
const uint8_t MAX_LISTENERS = 4;    // Max number of node listeners
const uint8_t MAX_COMMANDS = 32;    // Max number of supported commands
const uint8_t MAX_ACTIONS = 4;      // Max number of device actions
const uint16_t MAX_EEPROM = 0x1000;   // Max EEPRom memory
const uint16_t DEFAULT_EEPROM = 0x800;   //Offset for default configuration
const uint16_t MIN_ANALOG = 0;      //Min analog ADC value
const uint16_t MAX_ANALOG = 1023;   //Max analog ADC value
#endif
#if defined(HW_TYPE) && (HW_TYPE == HW_ARDUINO_UNO)
const uint8_t MAX_NODES = 8;		  // Max network nodes
const uint8_t MAX_PORTS = 2;		  // Max Communication serial ports
const uint8_t MAX_READ = 100;		  // Data reception timeout
const uint8_t MAX_BUFFER = 100;   // Max buffer allocation
const uint8_t TX_BUFFER = 100;    // Max buffer allocation
const uint8_t RX_BUFFER = 64;     // Max buffer allocation
const uint8_t MAX_PARAMETERS = 6;	// Max command parameters
const uint8_t MAX_STRING = 16;	  // Max string length
const uint8_t MAX_PINS = 16;	    // Max configured node pins
const uint8_t MAX_DEVICES = 4;	  // Max node devices
const uint8_t MAX_SENSORS = 4;	  // Max node sensors
const uint8_t MAX_LISTENERS = 4;  // Max number of node listeners
const uint8_t MAX_COMMANDS = 32;  // Max number of supported commands
const uint8_t MAX_ACTIONS = 4;    // Max number of device actions
const uint16_t MAX_EEPROM = 0x400;   // Max EEPRom memory
const uint16_t DEFAULT_EEPROM = 0x200;   //Offset for default configuration
const uint16_t MIN_ANALOG = 0;      //Min analog ADC value
const uint16_t MAX_ANALOG = 1023;   //Max analog ADC value
#endif
#if defined(HW_TYPE) && (HW_TYPE == HW_ARDUINO_MINI)
const uint8_t MAX_NODES = 4;		  // Max network nodes
const uint8_t MAX_PORTS = 1;		  // Max Communication serial ports
const uint8_t MAX_READ = 100;		  // Data reception timeout
const uint8_t MAX_BUFFER = 100;   // Max buffer allocation
const uint8_t TX_BUFFER = 100;    // Max buffer allocation
const uint8_t RX_BUFFER = 64;     // Max buffer allocation
const uint8_t MAX_PARAMETERS = 6;	// Max command parameters
const uint8_t MAX_STRING = 16;	  // Max string length
const uint8_t MAX_PINS = 8;	      // Max configured node pins
const uint8_t MAX_DEVICES = 3;	  // Max node devices
const uint8_t MAX_SENSORS = 4;	  // Max node sensors
const uint8_t MAX_LISTENERS = 2;  // Max number of node listeners
const uint8_t MAX_COMMANDS = 32;  // Max number of supported commands
const uint8_t MAX_ACTIONS = 4;    // Max number of device actions
const uint16_t MAX_EEPROM = 0x400;   // Max EEPRom memory
const uint16_t DEFAULT_EEPROM = 0x200;   //Offset for default configuration
const uint16_t MIN_ANALOG = 0;      //Min analog ADC value
const uint16_t MAX_ANALOG = 1023;   //Max analog ADC value
#endif

//-------------------------------------------------------
// Communication parameters
//-------------------------------------------------------
const uint32_t HBUS_BAUD_RATE = 57600;	        // Communication speed
const uint8_t  HBUS_DEFAULT_ADDRESS_WIDTH = 1;  // Address width in bytes

//-------------------------------------------------------
//Global parameters
//-------------------------------------------------------
const uint8_t NAME_LENGTH   = 5;            //Fixed string length (used for names)
const char* EMPTY_NAME			= "     ";	    //Empty name
const uint32_t DEF_DEBOUNCE = 55;           //in ms
const uint8_t FEED_INTERVAL = 10;           //in seconds

/*
  -------------------------------------------------------
   MESSAGE SEQUENCE:
  -------------------------------------------------------
   0x00 START
   0x01 FLAGS
   0x02 [USER LENGTH HI] (Optional)
   0x03 [USER LENGTH LO] (Optional)
   0xnn [DESTINATION HI] / [DESTINATION]
   ..
   0xnn [DESTINATION LO]
   0xnn [SOURCE HI] / [SOURCE]
   ..
   0xnn [SOURCE LO]
   0xnn [COMMAND]
   0xnn [DATA PAYLOAD] (first uint8_t)
   ...
   0xnn [DATA PAYLOAD] (last uint8_t)
   0xnn [CRC HI]
   0xnn [CRC LO]
   
  message flags:
   Flags = 7 6 5 4 3 2 1 0
  	7 = PAYLOAD_3
  	6 = PAYLOAD_2
  	5 = PAYLOAD_1
  	4 = PAYLOAD_0
  	3 = ADDRESS_WIDTH_1
  	2 = ADDRESS_WIDTH_0
  	1 = MSG_TYPE_BIT_1
  	0 = MSG_TYPE_BIT_0
  -------------------------------------------------------
*/
// Message header
const uint8_t START_BYTE 		= 0xAA;

//-------------------------------------------------------
//Address constants
//-------------------------------------------------------
//ADDRESS
const uint8_t NO_ADDRESS 		= 0x00;
const uint8_t ADDRESS_1 		= 0x04;
const uint8_t ADDRESS_2			= 0x08;
const uint8_t ADDRESS_4 		= 0x0C;

#if DEFAULT_WIDTH == 0
const uint8_t ADDRESS_WIDTH 	 = NO_ADDRESS;
const uint8_t MESSAGE_LENGTH 	 = 5; 		// START + FLAGS + COMMAND + CRC*2
typedef uint8_t hb_address_t;				//not really necessary
const uint8_t BROADCAST_ADDRESS  = 0xFF; 	//11111111
const uint8_t HOST_ADDRESS 		 = 0x81; 	//10000001
#endif
#if DEFAULT_WIDTH == 1
const uint8_t ADDRESS_WIDTH 	 = ADDRESS_1;
const uint8_t MESSAGE_LENGTH 	 = 7; 		// START + FLAGS + DESTINATION + SOURCE + COMMAND + CRC*2
// HBus message
typedef uint8_t hb_address_t;
// Default addresses
const uint8_t BROADCAST_ADDRESS  = 0xFF; 	//11111111
const uint8_t HOST_ADDRESS 		 = 0x81; 	//10000001
#endif
#if DEFAULT_WIDTH == 2
const uint8_t ADDRESS_WIDTH 	 = ADDRESS_2;
const uint8_t MESSAGE_LENGTH 	 = 9; 		// START + FLAGS + DESTINATION*2+ SOURCE*2 + COMMAND + CRC*2
typedef uint16_t hb_address_t;
// Default addresses
const uint16_t BROADCAST_ADDRESS = 0xFFFF;	//11111111
const uint16_t HOST_ADDRESS		 = 0x8100;	//10000001
#endif
#if DEFAULT_WIDTH == 4
const uint8_t ADDRESS_WIDTH 	 = ADDRESS_4;
const uint8_t MESSAGE_LENGTH 	 = 13; 		// START + FLAGS + DESTINATION*4+ SOURCE*4 + COMMAND + CRC*2
typedef uint32_t hb_address_t;
// Default addresses
const uint32_t BROADCAST_ADDRESS = 0xFFFFFFFF; 	//11111111
const uint32_t HOST_ADDRESS		 = 0x81000000; 	//10000001
#endif
//PAYLOAD
const uint8_t PAYLOAD_0			= 0x00;
const uint8_t PAYLOAD_1			= 0x10;
const uint8_t PAYLOAD_2			= 0x20;
const uint8_t PAYLOAD_3			= 0x30;
const uint8_t PAYLOAD_4			= 0x40;
const uint8_t PAYLOAD_5			= 0x50;
const uint8_t PAYLOAD_6			= 0x60;
const uint8_t PAYLOAD_7			= 0x70;
const uint8_t PAYLOAD_8			= 0x80;
const uint8_t PAYLOAD_16		= 0x90;
const uint8_t PAYLOAD_32		= 0xA0;
const uint8_t PAYLOAD_64		= 0xB0;
const uint8_t PAYLOAD_128		= 0xC0;
const uint8_t PAYLOAD_256		= 0xD0;
const uint8_t PAYLOAD_512		= 0xE0;
const uint8_t PAYLOAD_USER		= 0xF0;

//-------------------------------------------------------
//Message types
//-------------------------------------------------------
enum hb_message_types {
	NORMAL_MSG		= 0x00,
	IMMEDIATE_MSG	= 0x01,
	ACK_MSG 		= 0x02,
	NACK_MSG		= 0x03
};

//-------------------------------------------------------
//Bus status
//-------------------------------------------------------
const uint8_t BUS_RESET = 0x00;		// Status after controller reset
const uint8_t BUS_READY = 0x01;		// Controller is ready to trasnmit/receive
const uint8_t BUS_SEND = 0x02;		// Controller is sending data
const uint8_t BUS_RECEIVE = 0x03;	// Controller is receiving data

//-------------------------------------------------------
//Parse message results
//-------------------------------------------------------
const uint8_t NO_MESSAGE = 0x00;	//No message recevied/executed: retransmit if needed
const uint8_t MESSAGE_OK = 0x01;	//Message was received correctly and executed no ack/nack/retransmission needed

//-------------------------------------------------------
// Error codes
//-------------------------------------------------------
//HBus specific errors
const uint8_t ERR_UNKNOWN  = 0xFF;				// Error unknown
const uint8_t ERR_BUS_BUSY = 0xF0;				// Bus is sending/receiving
const uint8_t ERR_MESSAGE_CORRUPTED = 0xF1;		// Message corrupted
const uint8_t ERR_RX_OVERSIZE = 0xF2;			// Rx buffer overrun
const uint8_t ERR_RX_TIMEOUT = 0xF3;			// Rx timeout
const uint8_t ERR_BAD_CRC = 0xF4;				// crc error
const uint8_t ERR_ROUND_CHECK = 0xF5;			// Message originated from this node returned
const uint8_t ERR_ACK_LOST = 0xF6;				// Ack received from different source
const uint8_t ERR_COMMAND_UNKNOWN = 0xF7;		// Command unknown / unsupported
const uint8_t ERR_MEMORY_FULL = 0xF8;			// No more space for buffer allocation

const uint8_t PIN_NOT_FOUND = 0x70;             //Pin <name> not found
const uint8_t SENSOR_NOT_FOUND = 0x71;          //Sensor <name> not found
const uint8_t PIN_TYPE_INVALID = 0x72;          //Pin type is invalid
const uint8_t ADDRESS_NOT_FOUND = 0x73;         //Address not found
const uint8_t MAX_LISTENERS_REACHED = 0x74;     //Max listeners for pin/device/sensor
const uint8_t SENSOR_READ_FAILED = 0x75;        //Sensor read failed
const uint8_t DEVICE_NOT_FOUND = 0x76;          //Device <name> not found
const uint8_t DEVICE_ACTION_UNKNOWN = 0x77;     //Device action unknown
const uint8_t WIRE_NOT_FOUND = 0x78;            //Wire <input> not found
const uint8_t COMMAND_FAILED = 0x79;            //Command execution failed
const uint8_t MAX_SENSORS_REACHED = 0x7A;       //Max sensors added

#endif