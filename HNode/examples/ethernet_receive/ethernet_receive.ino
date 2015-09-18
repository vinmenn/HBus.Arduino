/*
-------------------------------------------------------------------------------------
  HBus - An home automation/IoT project

  Ethernet receive sensor node
  
  This node is configured as receiver for sensor read from another node
  (see example ethernet_transmit)
  You need a rgb led on pins 3,4,5
  The example set color according to received temperature:
  < 10°C       => blue
  >=10°C <18°C => blue + green
  >=18°C <23°C => green
  >=23°C <25°C => green + red
  >=25°C       => red
  
  Configuration:
    DEBUG:  set to show debug messages on Serial (that shouldn't be used as HBus port)
    DEFAULT_WIDTH:  Normally 1 = 1 byte address
    NODE_ADDRESS:   set a value from 1 to 254 ( 129 is normally used as "host" node)
    HW_TYPE:        see definitions and use value that matches best your board
    
  Copyright (C) 2014 Vincenzo Mennella (see license.txt)

  History
   1.0.0 08/09/2015:   First code release based on sensor_sensor.ino
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
//Node configuration
#define USE_PINS            0
#define USE_DEVICES         1
#define USE_SENSORS         1
#define USE_CONFIGURATION   0
#define USE_WATCHDOG        0
//Pins definitions
#define LED_PIN             13
#define R_PIN               3
#define G_PIN               4
#define B_PIN               5

//-----------------------------------------------------------------------------
// External libraries
//-----------------------------------------------------------------------------
#include <SPI.h> 
#include <Ethernet.h> 
#include <EEPROM.h> 
#if defined(DEBUG) && DEBUG == 1 //DEBUG on SERIAL
#include <Streaming.h>  
#endif
//-----------------------------------------------------------------------------
//HBus libraries
//-----------------------------------------------------------------------------
#include <HBusDefines.h>
#include <HBus.h> 	
#include <HNode.h> 	  
#include <HBusPort.h> 	
#include <HBusTcpPort.h> 	
#include <SimpleStack.h>
#include <Crc16.h> 	
#include <scheduler.h>
#include <devices/device.h>
#include <devices/RgbDevice.h>
#include <sensors/sensor.h>

//-----------------------------------------------------------------------------
//Node configuration
//-----------------------------------------------------------------------------
// Enter a MAC address for your controller below.
// Newer Ethernet shields have a MAC address printed on a sticker on the shield
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED }; 
//Node ip address
IPAddress ip(192, 168, 1, 3); 

HBusPort *port = new HBusTcpPort(ip);
HBus *bus = new HBus(port);
HNode node(bus);

//-----------------------------------------------------------------------------
//Node devices & sensors
//-----------------------------------------------------------------------------
//Define devices here
 RgbDevice dev01("RGB01", R_PIN, G_PIN, B_PIN, 100, 100, 80, false);  

//Sensor read callback
uint8_t mySensorReadCallback(hb_sensor_read_t sensor_read) {
  hb_device_action_t action;
  
  //Set action
  strcpy(action.action, "set");
  action.values = 6;
  //Reset color
  for(uint8_t i=0;i<6;i++)
    action.value[i] = 0;
    
  if (sensor_read.value < 10) {
    //blue
    action.value[4] = 0xff;
    action.value[5] = 0xff;
  }
  else if (sensor_read.value >= 10 && sensor_read.value < 18) {
    //green
    action.value[2] = 0xff;
    action.value[3] = 0xff;
    //blue
    action.value[4] = 0xff;
    action.value[5] = 0xff;
  }
  else if (sensor_read.value >= 18 && sensor_read.value < 23) {
    //green
    action.value[2] = 0xff;
    action.value[3] = 0xff;
  }
  else if (sensor_read.value >= 23 && sensor_read.value < 25) {
    //green
    action.value[2] = 0xff;
    action.value[3] = 0xff;
    //red
    action.value[0] = 0xff;
    action.value[1] = 0xff;
  }
  else {
    //red
    action.value[0] = 0xff;
    action.value[1] = 0xff;
  }
  
  dev01.executeAction(&action);
  
  return true;
}

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
#endif

  //HBus TCP port
  // start the Ethernet connection:
  Ethernet.begin(mac, ip);
#if defined(DEBUG) && DEBUG == 1 //DEBUG on SERIAL
  DEBUG_PORT << "Connecting..." << endl;
#endif 
  // give the Ethernet shield a second to initialize:
  delay(1000);

  //Use fixed configuration from code
  strcpy(node.info.name , "ND003");
  strcpy(node.info.description , "receiver node");
  strcpy(node.info.type , "SENSR");
  strcpy(node.info.hardware , "MEGA");
  strcpy(node.info.version , VERSION_HBUS_H);
  
#if defined(DEBUG) && DEBUG == 1 //DEBUG on SERIAL
  DEBUG_PORT << "node configured" << endl;
#endif

  //Add devices configuration here
  node.addDevice(&dev01);   //Rgb light
#if defined(DEBUG) && DEBUG == 1 //DEBUG on SERIAL
  DEBUG_PORT << "devices configured" << endl;
#endif
  //Add sensor read callback
  node.onSensorReadCallback = mySensorReadCallback;

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
}

void loop() {
  //----------------------------------
  //Main node function:
  //  Check inputs
  //  Update outputs
  //  Feed listeners
  //----------------------------------
  node.loop();

#if defined(DEBUG) && DEBUG == 1 //DEBUG on SERIAL
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
#endif
}
