/*
  =======================================================
  HBus - Home automation/IoT application protocol 

  Constants definitions
   
  Copyright (C) 2014 Vincenzo Mennella (see license.txt)

  History
    0.2.2 13/06/2014:   First public code release
  =======================================================
*/

#ifndef HBUS_DEFINES_H
#define HBUS_DEFINES_H
#define HBUS_VERSION "0.2.2"
#define HBUS_BUILD 	"221014"

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
// 1 = Arduino Uno
// 2 = Arduino Mega
// 3 = Arduino Mini
#if defined(HW_TYPE) && HW_TYPE == 2
const uint8_t MAX_NODES = 8;		// Max network nodes
const uint8_t MAX_PORTS = 3;		// Max Communication serial ports
const uint8_t MAX_READ = 100;		// Data reception timeout
const uint8_t MAX_BUFFER = 100;     // Max buffer allocation
const uint8_t TX_BUFFER = 100;      // Max buffer allocation
const uint8_t RX_BUFFER = 64;       // Max buffer allocation
const uint8_t MAX_PARAMETERS = 8;	// Max command parameters
const uint8_t MAX_STRING = 16;		// Max string length
const uint8_t MAX_PINS = 32;	    // Max configured node pins
const uint8_t MAX_DEVICES = 4;	    // Max node devices
const uint8_t MAX_SENSORS = 4;	    // Max node sensors
const uint8_t MAX_LISTENERS = 4;    // Max number of node listeners
const uint8_t MAX_COMMANDS = 32;    // Max number of supported commands
const uint8_t MAX_ACTIONS = 4;      // Max number of device actions
const uint16_t DEFAULT_EEPROM = 0x1ff;   //Offset for default configuration
#endif

#if defined(HW_TYPE) && HW_TYPE == 3
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
const uint16_t DEFAULT_EEPROM = 0xff;   //Offset for default configuration
#endif

// // Network map costants
// #define USE_STATIC_MAP	1	//Set to 1 to use static network map
// #define DEFAULT_WIDTH	1	//Se to default address width
// const uint8_t   NODES_LENGTH = 2;
// const uint16_t  NODE_MAP[][NODES_LENGTH] = { { 1, 129 }};

//-------------------------------------------------------
// Communication parameters
//-------------------------------------------------------
const uint32_t HBUS_BAUD_RATE = 19200;	    // Communication speed
const uint8_t  HBUS_DEFAULT_ADDRESS_WIDTH = 1;  // Address width in bytes

//-------------------------------------------------------
//Global parameters
//-------------------------------------------------------
const uint8_t NAME_LENGTH = 5;              // Fixed string length (used for names)
const char* EMPTY_NAME			= "     ";	//Empty name
const uint32_t DEF_DEBOUNCE = 15; //in ms
const uint8_t FEED_INTERVAL = 10; //in seconds

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

//-------------------------------------------------------
//ControlNode commands
//-------------------------------------------------------
//Configuration
const uint8_t CMD_RESET					= 0x01;	// Reset node
const uint8_t CMD_FACTORY_DEFAULT		= 0x02;	// Reset node to default configuration
const uint8_t CMD_READ_CONFIG			= 0x03;	// Read configuration from EEProm
const uint8_t CMD_WRITE_CONFIG			= 0x04;	// Write configuration on EEProm
const uint8_t CMD_SET_PIN 				= 0x05;	// Configure general pin
const uint8_t CMD_CONNECT				= 0x06;	// Connect input index to output name
const uint8_t CMD_START			    	= 0x07;	// Start node 
const uint8_t CMD_STOP				          = 0x08;	// Stop node (doesn't respond inputs)
const uint8_t CMD_ADD_NODE_LISTENER		  = 0x09;	// Add node listener (returns pin/device/sensor events)
const uint8_t CMD_DELETE_NODE_LISTENER  = 0x0A;	// Delete node listener
const uint8_t CMD_PUSH_NODE_STATUS      = 0x0B;	// Push node status (same as readAll)
	
//Write commands
const uint8_t CMD_CHANGE_DIGITAL		= 0x10;	// Set digital output On/Off
const uint8_t CMD_TOGGLE_DIGITAL		= 0x11;	// toggle digital output state
const uint8_t CMD_TIMED_DIGITAL			= 0x12;	// set digital output = ON for n seconds
const uint8_t CMD_DELAY_DIGITAL			= 0x13;	// Set digital output = ON after x seconds
const uint8_t CMD_PULSE_DIGITAL			= 0x14;	// set digital output = ON after x seconds and OFF after y seconds
const uint8_t CMD_CYCLE_DIGITAL			= 0x15;	// set digital output = ON after x seconds y seconds for z seconds
const uint8_t CMD_CHANGE_ALL_DIGITAL	= 0x16;	// set all digital outputs ON/OFF
const uint8_t CMD_CHANGE_PWM			= 0x17;	// set PWM output With x duty cyle
const uint8_t CMD_CHANGE_PIN    		= 0x18;	// set numeric output with int value (2 uint8_ts)
const uint8_t CMD_DELAY_TOGGLE_DIGITAL	= 0x19;	// set numeric output with int value (2 uint8_ts)
const uint8_t CMD_DELTA_PWM				= 0x1a;	// set numeric output with int value (2 uint8_ts)
const uint8_t CMD_FADE_PWM				= 0x1b;	// set numeric output with int value (2 uint8_ts)

//Read commands
const uint8_t CMD_READ_PIN				= 0x20;	// Read input pin (digital or numeric)
const uint8_t CMD_READ_KEY				= 0x21;	// Read last pressed key 
const uint8_t CMD_READ_ALL				= 0x22;	// Read all inputs/outputs/analogs
const uint8_t CMD_READ_ACTIVE			= 0x23;	// Read specific output (returns 1 if active)
const uint8_t CMD_READ_LAST_INPUT		= 0x24; // Read last triggered input
const uint8_t CMD_READ_LAST_ACTIVE    	= 0x25; // Read last activated output

//Information commands
const uint8_t CMD_GET_INFO 				= 0x30;	// Return node information
const uint8_t CMD_GET_PIN_INFO 			= 0x31;	// Return pin information
const uint8_t CMD_GET_CONNECT_INFO 		= 0x32;	// Return input->output connection information
const uint8_t CMD_GET_SENSOR_INFO 		= 0x33;	// Return sensor information
const uint8_t CMD_GET_DEVICE_INFO 		= 0x34;	// Return device information
const uint8_t CMD_GET_NAME_INFO			= 0x35;	// Return info about name

//Activation commands
const uint8_t CMD_ACTIVATE				= 0x40;	//Activate device
const uint8_t CMD_DEACTIVATE 			= 0x41; //Deactivate output according to its type
const uint8_t CMD_MULTI_ACTIVATE		= 0x42;	//Activate multiple devices (simple sequence)
const uint8_t CMD_ADD_PIN_LISTENER      = 0x43; //Add pin listener
const uint8_t CMD_DELETE_PIN_LISTENER   = 0x44; //Delete pin listener
const uint8_t CMD_PUSH_PIN_EVENT        = 0x45; //Push pin event to listeners


//Sensor commands
const uint8_t CMD_READ_SENSOR			= 0x50;	// Read a specific sensor value
const uint8_t CMD_ADD_SENSOR_LISTENER	= 0x51;	// Add listener to sensor readings
const uint8_t CMD_DELETE_SENSOR_LISTENER = 0x52; // Remove listener from sensor readings
const uint8_t CMD_PUSH_SENSOR_READ		= 0x53;	// Send sensor automatic read to listeners
const uint8_t CMD_RESET_SENSOR			= 0x54;	// Reset sensor

//Device commands
const uint8_t CMD_GET_DEVICE_STATUS 		= 0x60; // Get device status
const uint8_t CMD_EXECUTE_DEVICE_ACTION 	= 0x61;	// Execute specific device action
const uint8_t CMD_ADD_DEVICE_LISTENER 		= 0x62;	// Add listener to device events
const uint8_t CMD_DELETE_DEVICE_LISTENER 	= 0x63;	// Remove listener from device events
const uint8_t CMD_PUSH_DEVICE_EVENT 		= 0x64;	// Send device event to listeners

//Bus commands
const uint8_t CMD_PING 					= 0xE0; // Ping node (return ack)
const uint8_t CMD_DISCOVER_MAP 			= 0xE1; // Discovery nodes on the network
const uint8_t CMD_SET_MAP 					= 0xE2; // Set nodes map
const uint8_t CMD_GET_MAP 					= 0xE3;	// Get nodes map
const uint8_t CMD_GET_COMMANDS 			= 0xE4;	// Get supported commands
// const uint8_t CMD_GET_STATUS 				= 0xE5;	// Get bus status
// const uint8_t CMD_GET_ERROR 				= 0xE6;	// Get bus errors information
#endif