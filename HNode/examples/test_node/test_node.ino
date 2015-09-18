/*
-------------------------------------------------------------------------------------
  HBus - An home automation/IoT project

  This example explore HNode features and is used as template for real nodes

  Configuration:
    DEBUG:  set to show debug messages on Serial (that shouldn't be used as HBus port)
    DEFAULT_WIDTH:  Normally 1 = 1 byte address
    NODE_ADDRESS:   set a value from 1 to 254 ( 127 is normally used as "host" node
    HW_TYPE:        see definitions and use value that matches best your board
    USE_PINS:       set to 1 to use pins capabilities
    USE_DEVICES:    set to 1 to use devices capabilities
    USE_SENSORS:    set to 1 to use sensors capabilities
    USE_CONFIGURATION: set to 1 to use storage for node configuration
    USE_WATCHDOG:   set to 1 to add watchdog feature (reset after configured time)
    
  Copyright (C) 2014 Vincenzo Mennella (see license.txt)

  History
   1.0.0 31/12/2014:   First code release of last architecture
   1.1.0 27/07/2015:   Wire class revised: now send HBus command when triggered
-------------------------------------------------------------------------------------
*/

//-------------------------------------------------------
// Global defines
//-------------------------------------------------------
#define PROGRAM_VERSION     "1.1.0"
#define DEBUG               1  // 0 No debug, 1 serial, 2 lcd, 3 led
#define DEBUG_PORT          Serial 
#define DEFAULT_WIDTH       1   // Address width
#define NODE_ADDRESS        3   // HBus address
// Hardware types:
// HW_ARDUINO_UNO
// HW_ARDUINO_MEGA
// HW_ARDUINO_MINI
#define HW_TYPE             HW_ARDUINO_MEGA
#define HBUS_SERIAL         Serial1 
//Node configuration
#define USE_PINS            1
#define USE_DEVICES         0
#define USE_SENSORS         0
#define USE_CONFIGURATION   1
#define USE_WATCHDOG        0
//Pins definitions
#define LM35_PIN            A0
#define TXEN_PIN            0
#define IN_PIN              6
#define IN1_PIN             7
#define LED_PIN             13
#define UP_PIN              3
#define DN_PIN              4
#define R_PIN               5
#define G_PIN               6
#define B_PIN               7

//-----------------------------------------------------------------------------
// External libraries
//-----------------------------------------------------------------------------
#if defined(USE_CONFIGURATION) && USE_CONFIGURATION == 1
#include <EEPROM.h>
#if defined(DEBUG) && DEBUG == 1 //DEBUG on SERIAL
#include <Streaming.h>  
#endif
#endif
#if defined(USE_PINS) && USE_PINS == 1
  //Add specific pins libraries here
#endif
#if defined(USE_SENSORS) && USE_SENSORS == 1
  //Add specific sensor libraries here
  //e.g. #include <dht11.h>
#endif
#if defined(USE_DEVICES) && USE_DEVICES == 1
  //Add specific devices libraries here
#endif
#if defined(USE_WATCHDOG) && USE_WATCHDOG == 1
#include <avr/wdt.h>
#endif

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
  #include <pins/Wire.h>
#endif
#if defined(USE_DEVICES) && USE_DEVICES == 1
  //Add HBus devices libraries here
#include <devices/Device.h>
#include <devices/ShutterDevice.h>
#include <devices/RgbDevice.h>
#endif

#if defined(USE_SENSORS) && USE_SENSORS == 1
  //Add HBus sensor libraries here
#include <sensors/Sensor.h>
#include <sensors/SensorAnalog.h>
//#include <sensors/SensorDht11.h>
#endif

//-----------------------------------------------------------------------------
//Node configuration
//-----------------------------------------------------------------------------
//Serial port
HBusPort *port = new HBusSerialPort(&HBUS_SERIAL, TXEN_PIN);
HBus *bus = new HBus(port);
HNode node(bus);

#if !defined(USE_CONFIGURATION) || USE_CONFIGURATION == 0
//-----------------------------------------------------------------------------
//Node pins, devices & sensors
//-----------------------------------------------------------------------------
#if defined(USE_PINS) && USE_PINS == 1
  Pin pin01("IN001", IN_PIN, pinInput, inputLowHigh, 0, 0);
  Pin pin02("IN002", IN1_PIN, pinInput, inputLowHigh, 0, 0);
  Pin pin03("OUT01", LED_PIN, pinOutput, outputToggle, 0, 0);
  Wire wire01("IN001", "OUT01");
  Wire wire02(("IN002", CMD_ACTIVATE, BROADCAST_ADDRESS, "OUT31", 5, 0);
#endif
#if defined(USE_DEVICES) && USE_DEVICES == 1
  //Define devices here
   ShutterDevice dev01("SHT01", UP_PIN, DN_PIN, 100);
   RgbDevice dev02("RGB01", R_PIN, G_PIN, B_PIN, 100, 100, 80, false);
#endif
#if defined(USE_SENSORS) && USE_SENSORS == 1
  //Define sensors here
  //Generic analog sensor:
  //set min-max range and conversion scale factor
  SensorAnalog sns01("SNS01", LM35_PIN, "A0", "LM35", "C", 0.0f, 100.0f, 0.48828125f);	
#endif
#endif  //Fixed configuration

#if defined(DEBUG) && DEBUG == 3 //DEBUG on LED
// Debug message with led
void blink(uint8_t n) {
    for(uint8_t i=0;i<n;i++) {
      digitalWrite(LED_PIN, LOW);
      delay(250);
      digitalWrite(LED_PIN, HIGH);
      delay(250);
    }
}
#endif

void setup() {

#if defined(DEBUG) && DEBUG == 1 //DEBUG on SERIAL
  //Debug port
  DEBUG_PORT.begin(57600);
  DEBUG_PORT << "HNode test program version" << PROGRAM_VERSION << endl;
#endif

  //Hbus serial port
  HBUS_SERIAL.begin(HBUS_BAUD_RATE); 
  
//-----------------------------------------------------------------------------
// Fixed configuration 
//-----------------------------------------------------------------------------
#if !defined(USE_CONFIGURATION) || USE_CONFIGURATION == 0
  //Use fixed configuration from code
  strcpy(node.info.name , "ND003");
  strcpy(node.info.description , "test node with home automation/IoT functions");
  strcpy(node.info.type , "MULTI");
  strcpy(node.info.hardware , "MEGA");
  strcpy(node.info.version , VERSION_HBUS_H);

#if defined(DEBUG) && DEBUG == 1 //DEBUG on SERIAL
  DEBUG_PORT << "Node info added" << endl;
#endif 

#if defined(USE_PINS) && USE_PINS == 1
  node.addPin(&pin01);
  node.addPin(&pin02);
  node.addPin(&pin03);
  node.addWire(&wire01);
  node.addWire(&wire02);
#if defined(DEBUG) && DEBUG == 1 //DEBUG on SERIAL
  DEBUG_PORT << "Pins added" << endl;
#endif 
#endif

#if defined(USE_DEVICES) && USE_DEVICES == 1
  node.addDevice(&dev01);   //Shutter device
  node.addDevice(&dev02);   //Rgb light
#if defined(DEBUG) && DEBUG == 1 //DEBUG on SERIAL
  DEBUG_PORT << "Devices added" << endl;
#endif 
#endif

#if defined(USE_SENSORS) && USE_SENSORS == 1
  node.addSensor(&sns01);   //Temperature sensor 
#if defined(DEBUG) && DEBUG == 1 //DEBUG on SERIAL
  DEBUG_PORT << "Sensors added" << endl;
#endif 
#endif

#else
//-----------------------------------------------------------------------------
// Read configuration from storage
//-----------------------------------------------------------------------------
  node.readConfiguration(0);
#endif 

  //Start bus
  bus->begin(NODE_ADDRESS);    

  //Init node
  node.begin();
  
  //Start node
  node.start();

#if defined(DEBUG) && DEBUG == 1 //DEBUG on SERIAL
  DEBUG_PORT << "Node started" << endl;
#endif 

#if defined(DEBUG) && DEBUG == 3 //DEBUG on LED
  pinMode(LED_PIN, OUTPUT);
   blink(3);
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
      //SINGLE ADDRESS NORMAL COMMANDS - SINGLE PORT
      case '0': 
        node.stop();
        break;
      case '1': 
        node.start();
        break;
      case '2': 
        node.reset();
        break;
    }
  }
}
