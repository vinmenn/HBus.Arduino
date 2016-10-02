/*
-------------------------------------------------------------------------------------
  HBus - An home automation/IoT project

  Example of node configured with hub functions
  
  This node read sensor data and send activation commands from/to other node
    
  Configuration:
    DEBUG:  set to show debug messages on Serial (that shouldn't be used as HBus port)
    DEFAULT_WIDTH:  Normally 1 = 1 byte address
    NODE_ADDRESS:   set a value from 1 to 254
    HW_TYPE:        see definitions and use value that matches best your board
    USE_PINS:       set to 1 to use pins capabilities
    USE_DEVICES:    set to 1 to use devices capabilities
    USE_SENSORS:    set to 1 to use sensors capabilities
    USE_KEYPAD:     obsolete (set to 0)
    USE_CONFIGURATION: set to 1 to use storage for node configuration
    USE_WATCHDOG:   set to 1 to add watchdog feature (reset after configured time)
    
  Copyright (C) 2014 Vincenzo Mennella (see license.txt)

  History
   1.0.0 16/09/2015:   First code release based on test_sensor.ino
-------------------------------------------------------------------------------------
*/

//-------------------------------------------------------
// Global defines
//-------------------------------------------------------
#define DEBUG               1   // 0 No debug, 1 serial, 2 lcd, 3 led
#define DEBUG_PORT          Serial
#define DEFAULT_WIDTH       1   // Address width
#define NODE_ADDRESS        3   // HBus address
#define HW_TYPE             HW_ARDUINO_MEGA
#define HBUS_SERIAL         Serial1
//Node configuration
#define USE_PINS            0
#define USE_DEVICES         0
#define USE_SENSORS         1
#define USE_CONFIGURATION   0
#define USE_WATCHDOG        0
//Pins definitions
#define TXEN_PIN            0  
#define DHT_PIN             3
#define LED_PIN             13

//-----------------------------------------------------------------------------
// External libraries
//-----------------------------------------------------------------------------
#if defined(USE_CONFIGURATION) && USE_CONFIGURATION == 1
#include <EEPROM.h>
#endif
#if defined(USE_PINS) && USE_PINS == 1
//Add external pins libraries here
#endif
#if defined(USE_SENSORS) && USE_SENSORS == 1
//Add external sensor libraries here
#include <dht.h> 
#endif
#if defined(USE_DEVICES) && USE_DEVICES == 1
  //Add external devices libraries here
#endif
#if defined(USE_WATCHDOG) && USE_WATCHDOG == 1
#include <avr/wdt.h>
#endif
#include <Streaming.h>  

//-----------------------------------------------------------------------------
//HBus libraries
//-----------------------------------------------------------------------------
#include <HBusDefines.h>
#include <HBus.h> 	
#include <HNode.h> 	  
#include <HBusPort.h> 	
#include <HBusSerialPort.h> 	
#include <SimpleStack.h>
#include <Crc16.h> 	
#include <scheduler.h>

#if defined(USE_PINS) && USE_PINS == 1
  //Add specific pins libraries here
#include <pins/pin.h>
#endif
#if defined(USE_DEVICES) && USE_DEVICES == 1
//Add HBus devices libraries here
#include <devices/device.h>
#endif

#if defined(USE_SENSORS) && USE_SENSORS == 1
//Add HBus sensor libraries here
#include <sensors/sensor.h>
#include <sensors/SensorDht.h>
#endif

//-----------------------------------------------------------------------------
//Node configuration
//-----------------------------------------------------------------------------
//Serial port with RS485 enable pin
HBusPort *port = new HBusSerialPort(&HBUS_SERIAL, TXEN_PIN);
HBus *bus = new HBus(port);
HNode node(bus);
hb_address_t address_2 = 2;
hb_address_t address_4 = 4;
hb_address_t address_5 = 5;

char data[MAX_BUFFER];

//-----------------------------------------------------------------------------
//Node devices & sensors
//-----------------------------------------------------------------------------
#if defined(USE_DEVICES) && USE_DEVICES == 1
  //Define devices here
#endif
#if defined(USE_SENSORS) && USE_SENSORS == 1
  hb_sensor_read_t rd; //read from other nodes
  SensorDht sns31("SNS31", DHT_PIN, DHT11_TYPE, true);    // Dht11/22 temperature 
  SensorDht sns32("SNS32", DHT_PIN, DHT11_TYPE, false);   // Dht11/22 humidity
#endif

uint8_t OnSensorReadCallback(hb_sensor_read_t sr)
{
#if defined(DEBUG) && DEBUG == 1 //DEBUG on SERIAL
  DEBUG_PORT << "sensor " << sr.name << " value = " << sr.value << " @ " << sr.time << endl;
#endif  
  return true;
}

void setup() {
#if defined(DEBUG) && DEBUG == 1 //DEBUG on SERIAL
  //Debug port
  DEBUG_PORT.begin(57600);
  DEBUG_PORT << "node setup..." << endl;
#endif

  //Hbus serial port
  HBUS_SERIAL.begin(HBUS_BAUD_RATE);

#if defined(USE_CONFIGURATION) || USE_CONFIGURATION != 1
  //Use fixed configuration from code
  strcpy(node.info.name , "ND003");
  strcpy(node.info.description , "hub node");
  strcpy(node.info.type , "HUB");
  strcpy(node.info.hardware , "MEGA");
  strcpy(node.info.version , VERSION_HBUS_H);
#if defined(DEBUG) && DEBUG == 1 //DEBUG on SERIAL
  DEBUG_PORT << "node configured" << endl;
#endif

//Pins configuration
#if defined(USE_PINS) && USE_PINS == 1
  //Add pins configuration here
#endif

#if defined(USE_DEVICES) && USE_DEVICES == 1
  //Add devices configuration here
#endif

#if defined(USE_SENSORS) && USE_SENSORS == 1
  //Add sensors configuration here
  node.addSensor(&sns31);   //Temperature sensor  
  node.addSensor(&sns32);   //Humidity sensor  
#endif
#else
  //Read normal configuration from storage
  node.readConfiguration(false);
#endif 

  //Add sensor read callback
  node.onSensorReadCallback = OnSensorReadCallback;
  
  //Start bus
  bus->begin(NODE_ADDRESS);    
  
  //Start node
  node.begin();
  node.start();

#if defined(DEBUG) && DEBUG == 1 //DEBUG on SERIAL
  DEBUG_PORT << "node started" << endl;
#endif
  delay(1000);
#if defined(USE_WATCHDOG) && USE_WATCHDOG == 1
  //2 seconds watchdog
  wdt_enable(WDTO_2S);    
#endif
}

void loop() {
  //----------------------------------
  //Main node function:
  //  Check inputs
  //  Update outputs
  //  Feed listeners
  //----------------------------------
  node.loop();
#if defined(USE_WATCHDOG) && USE_WATCHDOG == 1
  //watchdog reset
  wdt_reset();
#endif

    if (DEBUG_PORT.available() > 0){
    
    switch(DEBUG_PORT.read())
    {
      case 'l': //Toggle remote led on node 4
        strncpy(data, "LS401", 5);
        bus->sendCommand(CMD_ACTIVATE, address_4, (uint8_t *) data, 5, 0);
        break;
      case 'w': //Toggle remote led on node 2 Win10
        strncpy(data, "LS501", 5);
        bus->sendCommand(CMD_ACTIVATE, address_5, (uint8_t *) data, 5, 0);
        break;
      case 'r': //Read remote temperature from SNS41 on node 4
        strncpy(data, "SNS41", 5);
        bus->sendCommand(CMD_READ_SENSOR, address_4, (uint8_t *) data, 5, 0);
        break;
      case 's': //Subscribe sensor SNS41 every 2 seconds expires after 10 readings
        strncpy(data, "SNS41", 5);
        data[5] = 2; data[6] = 0; data[7] = 10;
        bus->sendCommand(CMD_ADD_SENSOR_LISTENER, address_4, (uint8_t *)data, 8, 0);
        break;
      case 'u': //Unsubscribe sensor
        strncpy(data, "SNS41", 5);
        bus->sendCommand(CMD_DELETE_SENSOR_LISTENER, address_4, (uint8_t *)data, 5, 0);
        break;
      case 't': //Read temperature from local DHT11 sensor (response showed with callback)
        node.readSensor("SNS31");
        break;
      case 'h': //Read humidity from local DHT11 sensor (response showed with callback)
        node.readSensor("SNS32");
        break;
      case 'o': //open shutter on node 2
        strcpy(data, "DS201open");
        data[10] = 0; data[11] = 0;
        bus->sendCommand(CMD_EXECUTE_DEVICE_ACTION, address_2, (uint8_t *) data, 12, 0);
        break;
      case 'c': //Toggle remote led on node 4
        strcpy(data, "DS201close");
        data[11] = 0; data[12] = 0;
        bus->sendCommand(CMD_EXECUTE_DEVICE_ACTION, address_2, (uint8_t *) data, 13, 0);
        break;
    }
  } 
}
