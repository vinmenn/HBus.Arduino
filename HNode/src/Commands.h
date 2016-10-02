/*
  =======================================================
  HBus - Home automation/IoT application protocol
  Commands defines for nodes
  
  Copyright (C) 2014-15-16 Vincenzo Mennella (see license.txt)

  History
    0.0.1 14/08/2014:   First revision
    0.0.2 01/09/2015:   Updated namespace
  =======================================================
*/
#ifndef HBUS_COMMANDS_H
#define HBUS_COMMANDS_H

#define VERSION_HBUS_COMMANDS_H "0.0.2"
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
const uint8_t CMD_PING 					    = 0xE0; // Ping node (return ack)
const uint8_t CMD_DISCOVER_MAP 			= 0xE1; // Discovery nodes on the network
const uint8_t CMD_SET_MAP 					= 0xE2; // Set nodes map
const uint8_t CMD_GET_MAP 					= 0xE3;	// Get nodes map
const uint8_t CMD_GET_COMMANDS 			= 0xE4;	// Get supported commands
#endif