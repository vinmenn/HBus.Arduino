/*
-------------------------------------------------------------------------------------
  HBus - An home automation/IoT project

  Example of node configured with sensor functions for ARTIK PROJECT
  
  This node use 3 sensors:
    DHT11 = temperature
    DHT11 = humidity
    PHOTO RESISTOR = light
    
  Configuration:
    DEBUG:  set to show debug messages on Serial (that shouldn't be used as HBus port)
    DEFAULT_WIDTH:  Normally 1 = 1 byte address
    NODE_ADDRESS:   set a value from 1 to 254 ( 127 is normally used as "host" node
    HW_TYPE:        see definitions and use value that matches best your board
    USE_PINS:       set to 1 to use pins capabilities
    USE_DEVICES:    set to 1 to use devices capabilities
    USE_SENSORS:    set to 1 to use sensors capabilities
    USE_KEYPAD:     obsolete (set to 0)
    USE_CONFIGURATION: set to 1 to use storage for node configuration
    USE_WATCHDOG:   set to 1 to add watchdog feature (reset after configured time)
    
  Copyright (C) 2014 Vincenzo Mennella (see license.txt)

  History
   1.0.0 02/10/2016:  Created from sensor_node_ino
-------------------------------------------------------------------------------------
*/

//-------------------------------------------------------
// Global defines
//-------------------------------------------------------
#define DEBUG               3   // 0 No debug, 1 serial, 2 lcd, 3 led
#define DEBUG_PORT          Serial
#define DEFAULT_WIDTH       1   // Address width
#define NODE_ADDRESS        7   // HBus address
#define HW_TYPE             HW_ARDUINO_MINI
#define HBUS_SERIAL         Serial
//Node configuration
#define USE_PINS            1
#define USE_DEVICES         0
#define USE_SENSORS         1
#define USE_CONFIGURATION   0
#define USE_WATCHDOG        0
//Pins definitions
#define TXEN_PIN            9 
#define DHT_PIN             3
#define PHOTO_PIN           A0
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
#include <Scheduler.h>

#if defined(USE_PINS) && USE_PINS == 1
//Add specific pins libraries here
#include <pins/Pin.h>
#endif
#if defined(USE_DEVICES) && USE_DEVICES == 1
//Add HBus devices libraries here
#include <devices/Device.h>
#endif

#if defined(USE_SENSORS) && USE_SENSORS == 1
//Add HBus sensor libraries here
#include <sensors/Sensor.h>
#include <sensors/SensorDht.h>
#include <sensors/SensorAnalog.h>
#endif

  //-----------------------------------------------------------------------------
//Node configuration
//-----------------------------------------------------------------------------
//Serial port with RS485 enable pin
HBusPort *port = new HBusSerialPort(&HBUS_SERIAL, TXEN_PIN);
HBus *bus = new HBus(port);
HNode node(bus);

//-----------------------------------------------------------------------------
//Node devices & sensors
//-----------------------------------------------------------------------------
#if defined(USE_PINS) && USE_PINS == 1
  //Define pins here
  Pin pin1("LS701", "debug led", "internal", LED_PIN, 0, pinOutput, outputToggle);  
#endif
#if defined(USE_DEVICES) && USE_DEVICES == 1
  //Define devices here
#endif
#if defined(USE_SENSORS) && USE_SENSORS == 1
  //Define sensors here
  SensorDht sns1("SN701", DHT_PIN, DHT11_TYPE, true, 60);    // Dht11/22 temperature 
  SensorDht sns2("SN702", DHT_PIN, DHT11_TYPE, false, 120);   // Dht11/22 humidity 
  SensorAnalog sns3("SN703", PHOTO_PIN, "Photo", "lux", 0, 150, 2.6697);    // Light analog sensor    hb_sensor_read_t rd;
#endif

#if defined(DEBUG) && DEBUG == 3 //DEBUG on LED
// Debug message with led
void blink(uint8_t n) {
    for(uint8_t i=0;i<n;i++) {
      digitalWrite(LED_PIN, HIGH);
      delay(250);
      digitalWrite(LED_PIN, LOW);
      delay(250);
    }
}
#endif

void setup() {
#if defined(DEBUG) && DEBUG == 1 //DEBUG on SERIAL
  //Debug port
  DEBUG_PORT.begin(57600);
#endif

  //Hbus serial port
  HBUS_SERIAL.begin(HBUS_BAUD_RATE);

#if defined(USE_CONFIGURATION) || USE_CONFIGURATION != 1
  //Use fixed configuration from code
  strcpy(node.info.name , "ND007");
  strcpy(node.info.description , "sensor node");
  strcpy(node.info.type , "SENSR");
  strcpy(node.info.hardware , "UNO");
  strcpy(node.info.version , VERSION_HBUS_H);
#if defined(DEBUG) && DEBUG == 1 //DEBUG on SERIAL
  DEBUG_PORT << "node configured" << endl;
#endif

//Pins configuration
#if defined(USE_PINS) && USE_PINS == 1
  //Add pins configuration here
  node.addPin(&pin1); 
  
#if defined(DEBUG) && DEBUG == 1 //DEBUG on SERIAL
  DEBUG_PORT << "pins & wires configured" << endl;
#endif
#endif

#if defined(USE_DEVICES) && USE_DEVICES == 1
  //Add devices configuration here
#if defined(DEBUG) && DEBUG == 1 //DEBUG on SERIAL
  DEBUG_PORT << "devices configured" << endl;
#endif
#endif

#if defined(USE_SENSORS) && USE_SENSORS == 1
  //Add sensors configuration here
  node.addSensor(&sns1);   //Temperature sensor 
  node.addSensor(&sns2);   //Humidity sensor 
  node.addSensor(&sns3);   //Light sensor 
#if defined(DEBUG) && DEBUG == 1 //DEBUG on SERIAL
  DEBUG_PORT << "sensors configured" << endl;
#endif
#endif
#else
  //Read normal configuration from storage
  node.readConfiguration(false);
#endif 

  //Start bus
  bus->begin(NODE_ADDRESS);    
  
  //Start node
  node.begin();
  node.start();

 #if defined(DEBUG) && DEBUG == 3 //DEBUG on LED
   pinMode(LED_PIN, OUTPUT);
   blink(3);
#endif

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
}
