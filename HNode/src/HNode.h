/*
  =======================================================
  HNode - Home automation/IoT node
  
  Copyright (C) 2014 Vincenzo Mennella (see license.txt)

 History
  1.0.0 31/12/2014:   First code release of last architecture
  1.0.1 27/07/2015:   Wire class revised: now send HBus command when triggered
  =======================================================
*/
#ifndef H_NODE_H
#define H_NODE_H
#define VERSION_H_NODE "1.0.1"

//-----------------------------------------------------------------------------
// External libraries
//-----------------------------------------------------------------------------
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#if defined(USE_WATCHDOG) && USE_WATCHDOG == 1
#include <avr/wdt.h>
#endif
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
#include <Streaming.h> 
#endif
//-----------------------------------------------------------------------------
//HBus libraries
//-----------------------------------------------------------------------------
#include <HBusDefines.h>
#include <Commands.h> 	  
#include <HBus.h>
#include <Scheduler.h>
#include <SimpleStack.h>
#include <pins/Pin.h>
#include <pins/Wire.h>
#include <devices/Device.h>
#include <sensors/Sensor.h>

#if defined(USE_WATCHDOG) && USE_WATCHDOG == 1
#define resetAvr() wdt_enable(WDTO_2S); while(1) {}
#endif

//-------------------------------------------------------
// Global defines
//-------------------------------------------------------
enum hb_node_status {
	NODE_UNKNOWN = 0x00,
	NODE_RESET	 = 0x01,
	NODE_READY 	 = 0x02,
	NODE_ACTIVE  = 0x03,
	NODE_ERROR   = 0x04
};
enum hb_name_types {
    UNKNOWN_TYPE = 0x00,
    PIN_TYPE,
    DEVICE_TYPE,
    SENSOR_TYPE
};
//===========================================================
// HBus node info class
// Contains node configuration
//===========================================================
class NodeInfo 
{
  public:
    char name[NAME_LENGTH+1];
    char description[MAX_STRING];
    char type[MAX_STRING];
    char hardware[NAME_LENGTH+1];
    char version[NAME_LENGTH+1];
    uint8_t inputs;
    uint8_t outputs;
    uint8_t analogs;
    uint8_t counters;
    uint8_t pwms;
    uint8_t pins;
    uint8_t wires;
    uint8_t devices;
    uint8_t sensors;
    uint8_t reset_pin;
        
    NodeInfo(); 
    NodeInfo(SimpleStack *ss);
    void load(SimpleStack *ss);
    void save(SimpleStack *ss);
    char* toString();
};

// -------------------------------------------------------
// Empty constructor
// Parameters: none
// -------------------------------------------------------
NodeInfo::NodeInfo() 
{
  strcpy_P(name, PSTR(""));
  strcpy_P(description, PSTR(""));
  strcpy_P(type, PSTR(""));
  strcpy_P(hardware, PSTR(""));
  strcpy_P(version, PSTR(""));
  inputs = 0;
  outputs = 0;
  analogs = 0;
  counters = 0;
  pwms = 0;
  pins = 0;
  wires = 0;
  devices = 0;
  sensors = 0;
  // commands_length = 0;
  reset_pin = 0;
}

// -------------------------------------------------------
// Default constructor
// Parameters: 
//  serialized information (use SimpleStack library)
// -------------------------------------------------------
NodeInfo::NodeInfo(SimpleStack *ss) 
{
    load(ss);
}
// -------------------------------------------------------
// De-serialize node information from buffer array
// Parameters: stack object (use SimpleStack library)
// Returns: none
// -------------------------------------------------------
void NodeInfo::load(SimpleStack *ss) 
{
  //Node
  ss->popName(name);
  ss->popString(description);
  ss->popString(type); 
  ss->popName(hardware);
  ss->popName(version);
  inputs = ss->popByte();
  outputs = ss->popByte();
  analogs = ss->popByte();
  counters = ss->popByte();
  pwms = ss->popByte();
  wires = ss->popByte();
  devices = ss->popByte();
  sensors = ss->popByte();
  //ss->popArray(commands, &commands_length); 
  pins = inputs + outputs + analogs + counters + pwms;
  reset_pin = ss->popByte();
}
// -------------------------------------------------------
// Serialize node information into buffer array
// Parameters: stack object (use SimpleStack library)
// Returns: none
// -------------------------------------------------------
void NodeInfo::save(SimpleStack *ss) 
{
 //Push properties
  ss->pushName(name);
  ss->pushString(description);
  ss->pushString(type);
  ss->pushName(hardware);
  ss->pushName(version);
  ss->pushByte(inputs);
  ss->pushByte(outputs);
  ss->pushByte(analogs);
  ss->pushByte(counters);
  ss->pushByte(pwms);
  ss->pushByte(wires);
  ss->pushByte(devices);
  ss->pushByte(sensors);
  //ss->pushArray(commands, commands_length); 
  ss->pushByte(reset_pin);
}
// -------------------------------------------------------
// Default class string conversion
// Parameters: none
// Returns: string with main information
// -------------------------------------------------------
char* NodeInfo::toString() {
    return name;
}

//Simple function handlers
//Is used to hook events directly to Arduino code (ino files)
extern "C" {
	// callback function types
    typedef void (*pinChangeCallback)(char *name, uint32_t value);
} 
//===========================================================
// HBus node controller class
//===========================================================
class HNode : HBusMessageHandler, public HBusPinController, HBusDeviceController, HBusSensorController {
  private:
    HBus *_bus;                                 //Bus controller
    Scheduler *_scheduler;                      //Scheduler management
    SimpleStack _ee;                            //Configuration stack
    SimpleStack *_rx;                           //Reception stack
    SimpleStack *_tx;                           //Transmission stack
    hb_node_status _status;                     //Node status
    hb_address_t _node_listener[MAX_LISTENERS];  //Node listeners
    uint8_t _node_listener_mask[MAX_LISTENERS];  //Node listeners masks
    uint8_t _node_listener_port[MAX_LISTENERS];  //Node listeners port
    uint8_t _node_listeners;                     //Node listeners number
    
#if defined(USE_PINS) && USE_PINS == 1        
    Pin *_pin[MAX_PINS];                    //Pins configuration
    Wire *_wire[MAX_PINS];                  //Wires configuration
    char _last_input[NAME_LENGTH+1];            //Last activated input
    char _last_output[NAME_LENGTH+1];           //Last activated output
    hb_address_t _pin_listener[MAX_PINS];       //Pins listeners;
    
    Pin* pin;
    Wire wire;
#endif
#if defined(USE_DEVICES) && USE_DEVICES == 1
    Device *_device[MAX_DEVICES];              //Devices instances
    hb_device_action_t _action;
    hb_device_event_t _dev_event;
    uint8_t _send_dev_evt;
#endif
#if defined(USE_SENSORS) && USE_SENSORS == 1
    Sensor *_sensor[MAX_SENSORS];              //Sensors instances
#endif        

    //support variables
    uint32_t _last_time;
    uint8_t _feed_index;
    uint8_t _value[3];
    uint16_t _int_value[3];
    char _tmp_name[NAME_LENGTH+1];       
    Pin *_tmp_pin;                
    Wire *_tmp_wire;       
    hb_message_t _msg;
  
    //===========================================================
    // Private functions
    //===========================================================
    uint8_t addNodeListener(uint8_t mask, hb_address_t source, uint8_t port);
    uint8_t delNodeListener(hb_address_t source);
    void feedNodeListeners(uint8_t index);
  public:
  
    //===========================================================
    //Public properties
    //===========================================================
    NodeInfo info;
    //===========================================================
    // Public functions
    //===========================================================
    
    //Constructors
    HNode(HBus *bus);
    //General methods
    void begin();
    void loop();
    void reset(uint8_t full = 0);
    void start();
    void stop();
    //Pins
 #if defined(USE_PINS) && USE_PINS == 1        
    uint8_t addPin(Pin *pin);
    uint8_t addWire(Wire *wire);
    uint8_t addPinListener(char* name, hb_address_t source, uint8_t port);
    uint8_t delPinListener(char* name, hb_address_t source);
    void getPinInfo(uint8_t index, hb_pin_type type);
    void getWire(uint8_t index);
    uint8_t activate(char* name);
    uint8_t deactivate(char* name);
    void changeAll(uint8_t value);
    Pin* getPinByName(char* name);
    Pin* getPinByType(uint8_t index, hb_pin_type type);
    void onPinChange(char *name, uint32_t value);
    pinChangeCallback onPinChangeCallback;
#endif
    // Device commands
#if defined(USE_DEVICES) && USE_DEVICES == 1
    uint8_t addDevice(Device *device);
    void getDeviceInfo(uint8_t index);
    uint8_t getDeviceStatus(char* name);
    uint8_t executeDeviceAction(char* name, hb_device_action_t* action);
    uint8_t addDeviceListener(char* name, hb_address_t source, uint8_t port);
    uint8_t delDeviceListener(char* name, hb_address_t source);
    Device* getDevice(char* name);
    void onDeviceEvent(hb_device_event_t *device_event);
    deviceEventCallback onDeviceEventCallback;
#endif
    //Sensors
#if defined(USE_SENSORS) && USE_SENSORS == 1
    uint8_t addSensor(Sensor *sensor);
    void getSensorInfo(uint8_t index);
    hb_sensor_read_t readSensor(char* name);
    uint8_t pushSensorRead(char* name);
    uint8_t resetSensor(char* name);
    uint8_t addSensorListener(char* name, hb_address_t source, uint8_t port, uint8_t interval, uint16_t expire);
    uint8_t delSensorListener(char* name, hb_address_t source);
    Sensor* getSensor(char* name);
    void feedSensorListeners();
    void onSensorRead(hb_sensor_read_t sensor_read);
    sensorReadCallback onSensorReadCallback;
#endif
    void readConfiguration(uint8_t default_config);
    void writeConfiguration(uint8_t default_config); 
    void readAll(uint8_t mask);
    uint8_t getNameInfo(char* name);
    // Event handlers
    uint8_t processCommand(hb_message_t message, uint8_t port);
    uint8_t processAck(hb_message_t ack, uint8_t port);
    //Obsolete
#if defined(USE_KEYPAD) && USE_KEYPAD == 1
    void readLastKey();
#endif 
    // DEBUG
    int freeRam ();
};
//===========================================================
// Node listeners
//===========================================================
//-----------------------------------------------------------
// Add node listener
// Parameters:  flags mask, listener address
// Returns: true/false
//-----------------------------------------------------------
uint8_t HNode::addNodeListener(uint8_t mask, hb_address_t source, uint8_t port)
{
  if (_node_listeners >= MAX_LISTENERS) return false;
  
  for(uint8_t i=0;i<_node_listeners;i++) {
    if (_node_listener[i] == source)
    {
      _node_listener_mask[i] = mask;
      _node_listener_port[i] = port;
      
      return true;
    }
  }
  _node_listener[_node_listeners] = source;
  _node_listener_mask[_node_listeners] = mask;
  _node_listener_port[_node_listeners++] = port;

  return true;
}
//-----------------------------------------------------------
// Delete node listener
// Parameters:  listener address
// Returns: true/false
//-----------------------------------------------------------
uint8_t HNode::delNodeListener(hb_address_t source)
{
  for(uint8_t i = 0; i < _node_listeners; i++)
  {
    //Search listener
    if (_node_listener[i] == source) {
        
      //move next listeners one step ahead
      if (i < (_node_listeners-1)) {
        for(int ii=i; i < (_node_listeners-1);ii++) {
          _node_listener[ii] = _node_listener[ii+1];
          _node_listener_mask[ii] = _node_listener_mask[ii+1];
          _node_listener_port[ii] = _node_listener_port[ii+1];
        }
      }
      //Reset last position
      _node_listener[--_node_listeners] = 0;
      return true;
    }
  }
  return false;
}
//-----------------------------------------------------------
// Send node status to registered listeners
// Parameters:  listener index
// Returns: none
//-----------------------------------------------------------
void HNode::feedNodeListeners(uint8_t index) 
{
  if (_node_listener[index] != 0 && _node_listener_mask[index] != 0) {
  
    readAll(_node_listener_mask[index]);
    
    _bus->sendImmediate(CMD_PUSH_NODE_STATUS, _node_listener[index], _node_listener_port[index]);
      
  }
}
//===========================================================
//Constructors
//===========================================================
//-------------------------------------------------------
// Default constructor
//
// Parameters: HBus controller
//-------------------------------------------------------
HNode::HNode(HBus *bus) 
{
  _bus = bus;
  _bus->registerCallback(this);
  _scheduler = new Scheduler();
  _status = NODE_RESET;
}
//===========================================================
//General methods
//===========================================================
//-----------------------------------------------------------
// Initialize node
//
// Parameters: none
// Returns:	none
//-----------------------------------------------------------
void HNode::begin()
{
  int i;
  _ee.begin(MAX_EEPROM, NAME_LENGTH); //EEProm stack

  //Bus initialization
  if (_bus != 0) {
    _bus->registerCallback(this);
    _tx = _bus->tx();
    _rx = _bus->rx();
  }
    
#if defined(USE_PINS) && USE_PINS == 1        
  for(i=0;i<info.pins;i++) {
    _pin[i]->controller = this;
    _pin[i]->begin();
  }
  //Clear last activated input/output
  strcpy_P(_last_input, PSTR(""));
  strcpy_P(_last_output, PSTR(""));
#endif            
#if defined(USE_DEVICES) && USE_DEVICES == 1
  //Devices
  for(i=0;i<info.devices;i++) {
    _device[i]->scheduler = _scheduler;
    _device[i]->controller = this;
    _device[i]->begin();
  }
  _node_listeners = 0;
  for(i = 0; i < MAX_LISTENERS; i++) {
    _node_listener[i] = 0;  //Node listeners
  }
#endif            
#if defined(USE_SENSORS) && USE_SENSORS == 1
  //Sensors
  for(i=0;i<info.sensors;i++) {
    _sensor[i]->scheduler = _scheduler;
    _sensor[i]->controller = this;
    _sensor[i]->begin();
  }
#endif            
  _node_listeners = 0;
  
  for(i = 0; i < MAX_LISTENERS; i++) {
    _node_listener[i] = 0;  //Node listeners
  }
  _feed_index = 0;
  _status = NODE_READY;
}
//-----------------------------------------------------------
// loop routine
//
// Parameters: none
// Returns:	none
//-----------------------------------------------------------
void HNode::loop() 
{
  //Check hbus commands
  _bus->checkMessages();

  if (_status != NODE_ACTIVE) return;
    
#if defined(USE_PINS) && USE_PINS == 1        
  _msg.source = _bus->address;
  
  //Check input pins
  for(uint8_t i = 0; i < info.pins; i++)
  {
    if (_pin[i]->isActive()) {

      //Store last active input name
      strcpy(_last_input, _pin[i]->name);
        
      for(uint8_t w = 0; w < info.wires; w++)
      {
        uint16_t l = _wire[w]->length;
        
        if (_wire[w]->use_input_value) {
          //Add input value to stack data
          _wire[w]->data[l++] = (_pin[i]->lastValue >> 24) & 0xff;
          _wire[w]->data[l++] = (_pin[i]->lastValue >> 16) & 0xff;
          _wire[w]->data[l++] = (_pin[i]->lastValue >> 8) & 0xff;
          _wire[w]->data[l++] =  _pin[i]->lastValue & 0xff;
        }
        if (strcmp(_pin[i]->name, _wire[w]->input) == 0) {
          if (_wire[w]->address == 0) {
            //Process local HBus command simulating reception from remote
            _msg.command = _wire[w]->command;
            memcpy(_msg.data, _wire[w]->data, l);
            _msg.length = l;
            processCommand(_msg, 0xff);
          }
          else {
            //Send remote HBus command
            _tx->clearWriteIndex();
            _bus->sendCommand(_wire[w]->command, _wire[w]->address, _wire[w]->data, l, 0xff);
          }
          break;
        }
      }
    }
  }
#endif            
  
#if defined(USE_DEVICES) && USE_DEVICES == 1
  //Send device event to listener (one for device)
  if (_send_dev_evt<0xff) { //?? _send_dev_evt
    _bus->sendImmediate(CMD_PUSH_DEVICE_EVENT, _device[_send_dev_evt]->listener, _device[_send_dev_evt]->listenerPort); 
    _send_dev_evt = 0xff;
  }
#endif            

  //Update outputs and devices
  _scheduler->update();
  
  //Update listeners
  if ((_last_time)< _scheduler->getTime())
  {
// #if defined(USE_PINS) && USE_PINS == 1        
    // feedPinListeners();
// #endif            
    
#if defined(USE_SENSORS) && USE_SENSORS == 1
    feedSensorListeners();
#endif            
    //Feed one node listener for time
    feedNodeListeners(_feed_index++);
    
    if (_feed_index>=MAX_LISTENERS) _feed_index = 0;
    
    _last_time = _scheduler->getTime();
    
  }
  
}
//===========================================================
//  Node commands
//===========================================================
//---------------------------------------------------------------
// Reset node status
//---------------------------------------------------------------
void HNode::reset(uint8_t full) 
{
  int i;
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  Serial << "reset " << full << endl;
#endif
  if (full) {
    if (info.reset_pin > 0) {
      digitalWrite(info.reset_pin, LOW);
      pinMode(info.reset_pin, OUTPUT); 
    }
#if defined(USE_WATCHDOG) && USE_WATCHDOG == 1        
    else {
      resetAvr();
    }
#endif
  }
  //---------------------------------
  //Status reset: node doesn't reboot
  //---------------------------------

  _scheduler->clean();
  _last_time = 0;
    
#if defined(USE_PINS) && USE_PINS == 1        
  //deactivate outputs / reset inputs
  for(i = 0; i < info.pins; i++) {
    _pin[i]->reset();
  }
  //Clear last activated input/output
  strcpy_P(_last_input, PSTR(""));
  strcpy_P(_last_output, PSTR(""));
#endif            
#if defined(USE_DEVICES) && USE_DEVICES == 1
  //Reset devices
  for(i = 0; i < info.devices; i++) {
    _device[i]->reset();
  }
  _send_dev_evt = 0xff;
#endif
#if defined(USE_SENSORS) && USE_SENSORS == 1
  //Reset sensors
  for(i = 0; i < info.sensors; i++) {
    _sensor[i]->reset();
  }
#endif            
  _node_listeners = 0;
  for(i = 0; i < MAX_LISTENERS; i++) {
    _node_listener[i] = 0;  //Node listeners
  }
  _feed_index = 0;
  _status = NODE_RESET;
}
//---------------------------------------------------------------
// Start active functions
//---------------------------------------------------------------
void HNode::start() 
{
  _scheduler->start();
  _status = NODE_ACTIVE;
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  Serial << "start " << endl;
#endif
}
//---------------------------------------------------------------
// Stop active functions
//---------------------------------------------------------------
void HNode::stop()  
{
  _scheduler->stop();
  _status = NODE_READY;
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  Serial << "stop " << endl;
#endif
}
//-----------------------------------------------------------
// Load node configuration from storage
//-----------------------------------------------------------
void HNode::readConfiguration(uint8_t default_config) 
{
  uint8_t i;
  
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  // Serial << "=> readConfiguration " << endl;
#endif
  _ee.clear();
  if (default_config)
    _ee.setReadIndex(DEFAULT_EEPROM); //TODO set specific EEProm limits

  //Node info;
  info.load(&_ee);
  
#if defined(USE_PINS) && USE_PINS == 1        
  //Pins
  for(i=0;i<info.pins;i++) {
      _pin[i]->load(&_ee);
  }
  
  //Wires
  for(i=0;i<info.wires;i++) {
      _wire[i]->load(&_ee);
  }
#endif
        
#if defined(USE_DEVICES) && USE_DEVICES == 1
  //Devices
  for(i=0;i<info.devices;i++) {
      //Configure device
      _device[i]->deserializeConfig(&_ee);
  }
#endif

#if defined(USE_SENSORS) && USE_SENSORS == 1
  //Sensors
  for(i=0;i<info.sensors;i++) {
      _sensor[i]->deserializeConfig(&_ee);
  }
#endif

  _status = NODE_READY;
}
//-----------------------------------------------------------
// Save node configuration to storage
//-----------------------------------------------------------
void HNode::writeConfiguration(uint8_t default_config) 
{
  uint8_t i;
  Device *dev;
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  Serial << "=> writeConfiguration " << endl;
#endif
  _ee.clear();
  if (default_config)
    _ee.setWriteIndex(DEFAULT_EEPROM); //TODO set specific EEProm limits
      
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  Serial << "\t" << _ee.writeIndex() << endl;
#endif

  //NodeInfo;
  info.save(&_ee);
  
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  Serial << "\tNode " << _ee.writeIndex() << endl;
#endif

#if defined(USE_PINS) && USE_PINS == 1        
  //Pins
  for(i=0;i<info.pins;i++) {
    _pin[i]->save(&_ee);
  }
  
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  Serial << "\tPins " << _ee.writeIndex() << endl;
#endif

  //Wires
  for(i=0;i<info.wires;i++) {
    _wire[i]->save(&_ee);
  }
  
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  Serial << "\tWires " << _ee.writeIndex() << endl;
#endif

#endif 
#if defined(USE_DEVICES) && USE_DEVICES == 1
  //Devices
  for(i=0;i<info.devices;i++) {
    dev = _device[i];
    dev->serializeConfig(&_ee);
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  Serial << "\tDevice: " << _ee.writeIndex() << endl;
#endif
  }
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  Serial << "\tDevices " << _ee.writeIndex() << endl;
#endif
#endif            
#if defined(USE_SENSORS) && USE_SENSORS == 1
  //Sensors
  for(i=0;i<info.sensors;i++) {
      _sensor[i]->serializeConfig(&_ee);
  }
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  Serial << "\tSensors: " << _ee.writeIndex() << endl;
#endif
#endif            
}
// -------------------------------------------------------
// Serialize overall node status into tx stack
// Parameters: read mask:
// 0x01 = Input pins
// 0x02 = Output pins
// 0x04 = Analog pins
// 0x08 = Counter pins
// 0x10 = Devices
// 0x20 = Sensors
// Returns:
// none
// -------------------------------------------------------
void HNode::readAll(uint8_t mask) 
{
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  // Serial << "=> readAll "<< mask << endl;
#endif
    _tx->clearWriteIndex();
    _tx->pushByte(mask);
    _tx->pushLong(_scheduler->getTime());    //Node time
    // _tx->pushInteger(0);
    // _tx->pushInteger(freeRam());
    _tx->pushByte(_status);                  //Node status
    _tx->pushByte(_bus->getStatus());        //Bus status
    _tx->pushByte(_bus->getError());         //Last error
    _tx->pushLong(_bus->totalErrors());      //Total errors

    _tx->pushByte(info.inputs);
    _tx->pushByte(info.outputs);
    _tx->pushByte(info.analogs);
    _tx->pushByte(info.counters);
    _tx->pushByte(info.pwms);
    _tx->pushByte(info.devices);
    _tx->pushByte(info.sensors);

#if defined(USE_PINS) && USE_PINS == 1        
    _tx->pushName(_last_input);
    _tx->pushName(_last_output);

    //Inputs
    if ((mask & 0x01)  != 0) 
      for(uint8_t i = 0; i < info.pins; i++) 
        if (_pin[i]->type == pinInput)
          _tx->pushByte((uint8_t) digitalRead(_pin[i]->pin));
    
    //Outputs
    if ((mask & 0x02)  != 0) 
      for(uint8_t i = 0; i < info.pins; i++) 
        if (_pin[i]->type == pinOutput)
          _tx->pushByte((uint8_t) digitalRead(_pin[i]->pin));
    
    //Analogs
    if ((mask & 0x04)  != 0)
        for(uint8_t i = 0; i < info.pins; i++) 
            if (_pin[i]->type == pinAnalog)
                _tx->pushInteger((uint16_t) analogRead(_pin[i]->pin));
    
    //Counters
    if ((mask & 0x08)  != 0)
        for(uint8_t i = 0; i < info.pins; i++) 
            if (_pin[i]->type == pinCounter)
                _tx->pushLong(_pin[i]->lastValue);
    
    //Pwm
    if ((mask & 0x10)  != 0)
        for(uint8_t i = 0; i < info.pins; i++) 
            if (_pin[i]->type == pinPwm)
                _tx->pushInteger((uint16_t) _pin[i]->lastValue);
#else
    _tx->pushName("");  //Last input
    _tx->pushName("");  //Last output
#endif            

#if defined(USE_DEVICES) && USE_DEVICES == 1
    //Devices
    if ((mask & 0x20)  != 0)
        for(uint8_t i = 0; i < info.devices; i++) {
            _tx->pushString((char*)_device[i]->status());
        }
#endif                    
#if defined(USE_SENSORS) && USE_SENSORS == 1
    //Sensors
    if ((mask & 0x40)  != 0) {
        hb_sensor_read_t value;
        
        for(uint8_t i = 0; i < info.sensors; i++) {
            _sensor[i]->read(&value);
            //Push sensor read into stack
            _tx->pushFloat(value.value);
        }
    }
#endif
}
//-----------------------------------------------------------
// Get name information
//-----------------------------------------------------------
uint8_t HNode::getNameInfo(char* name)
{
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  // Serial << "=> getNameInfo "<< name << endl;
#endif
  _tx->clearWriteIndex();
#if defined(USE_PINS) && USE_PINS == 1        
    Pin *pin = getPinByName(name);
    if (pin!=0) {
       _tx->pushByte(PIN_TYPE);
       pin->save(_tx);
       return true;
    }
#endif
#if defined(USE_DEVICES) && USE_DEVICES == 1
    Device *dev = getDevice(name);
    if (dev!=0) {
       _tx->pushByte(DEVICE_TYPE);
       dev->serializeInfo(_tx);
       return true;
    }
#endif            
#if defined(USE_SENSORS) && USE_SENSORS == 1
    Sensor *sns = getSensor(name);
    if (sns!=0) {
       _tx->pushByte(SENSOR_TYPE);
       sns->serializeInfo(_tx);
       return true;
    }
#endif            
    _tx->pushByte(UNKNOWN_TYPE);
    return false;
}
//===========================================================
// Pins commands
//===========================================================
#if defined(USE_PINS) && USE_PINS == 1        
//-----------------------------------------------------------
// Add pin to the node
//-----------------------------------------------------------
uint8_t HNode::addPin(Pin *pin) 
{
  uint8_t i=0;
  
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
//      Serial << "setPin ";
#endif
  //Search existing pin
  for(;i<info.pins;i++)
    if (_pin[i]->index == pin->index && _pin[i]->type == pin->type) {
      free(_pin[i]);
      //Set new pin
      _pin[i] = pin;
      _pin[i]->scheduler = _scheduler;  
        
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
//      Serial << "=> " << i << endl;
#endif
      return i;
    }
      
  //new pin
  if (info.pins == MAX_PINS) {
    free(pin);
    return 0xff;
  }
  
  i = info.pins++;
  
  _pin[i] = pin;
  _pin[i]->scheduler = _scheduler;  
  
  switch(pin->type)
  {
    case pinInput:
      info.inputs++;
      break;
    case pinOutput:
      info.outputs++;
      break;
    case pinAnalog:
      info.analogs++;
      break;
    case pinPwm:
      info.pwms++;
      break;
    case pinCounter:
      info.counters++;
      break;
  }

  return i;
}
//-----------------------------------------------------------
// Set wire configuration
//-----------------------------------------------------------
uint8_t HNode::addWire(Wire *wire) 
{
  uint8_t i=0;
  
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  // Serial << "=> setWire " << endl;
#endif
  //Search existing wire
  for(;i<info.wires;i++)
    if (strcmp(_wire[i]->input, wire->input)==0) {
      free(_wire[i]);
      _wire[i] = wire;
      return i;
    }
  
  if (info.wires == MAX_PINS) {
    free(wire);
    return 0xff;
  }
    
  i = info.wires++;
  
  _wire[i] = wire;
  
  return i;
}
//-----------------------------------------------------------
// Add pin listener
//-----------------------------------------------------------
uint8_t HNode::addPinListener(char* name, hb_address_t source, uint8_t port)
{
  for(uint8_t i = 0; i < info.pins; i++)
  {
    //Search pin
    if (strcmp(name, _pin[i]->name) == 0 || strcmp(name, EMPTY_NAME) == 0) {
      _pin[i]->listener = source;
      _pin[i]->listenerPort = port;
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  // Serial << "=> addPinListener " << name << endl;
#endif
      return true;
    }
  }
  return false;
}
//-----------------------------------------------------------
// Delete pin listener
//-----------------------------------------------------------
uint8_t HNode::delPinListener(char* name, hb_address_t source)
{
  for(uint8_t i = 0; i < info.pins; i++)
  {
    //Search pin
    if ((strcmp(name, _pin[i]->name) == 0  || strcmp(name, EMPTY_NAME) == 0)) {
      _pin[i]->listener = 0;
      _pin[i]->listenerPort = 0;
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  // Serial << "=> delPinListener " << name << endl;
#endif
      return true;
    }
  }
  return false;
}
//-----------------------------------------------------------
// Returns pin information
//-----------------------------------------------------------
void HNode::getPinInfo(uint8_t index, hb_pin_type type)
{
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  // Serial << "=> getPin " << index << " - " << type << endl;
#endif
  _tmp_pin = getPinByType(_value[0], (hb_pin_type) _value[1]);
  //If pin found get information
  if (_tmp_pin != 0) 
    _tmp_pin->save(_tx);
  else
    _bus->setError(PIN_NOT_FOUND);
}
//-----------------------------------------------------------
// Handle pin change event
//-----------------------------------------------------------
void HNode::onPinChange(char *name, uint32_t value)
{
  pin = getPinByName(name);
  
  if (pin->listener > 0) {
    //Send device event to listener
    _tx->clearWriteIndex();
    _tx->pushName(name);
    _tx->pushLong(value);
    _bus->sendImmediate(CMD_PUSH_PIN_EVENT, pin->listener, pin->listenerPort); 
  }
  if (onPinChangeCallback != 0)
    onPinChangeCallback(name, value);
}
//-----------------------------------------------------------
// Returns wire information
//-----------------------------------------------------------
void HNode::getWire(uint8_t index)
{
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  // Serial << "=> getWire " << index << endl;
#endif
  if (info.wires >= index)
    _wire[index]->save(_tx);
  else
    _bus->setError(WIRE_NOT_FOUND);
}
//-----------------------------------------------------------
// Activate device or pin 
// Parameters: 
//   Pin name
// Returns:	
//   true if pin is found
//-----------------------------------------------------------
uint8_t HNode::activate(char* name)
{
  Pin* pin = getPinByName(name);
    
  if (pin == 0) return false;
  pin->activate();
// #if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  // Serial << "=> activate " << name << endl;
// #endif
  return true;
}
//-----------------------------------------------------------
// Deactivate device or pin 
// Parameters: 
//   Pin name
// Returns:	
//   true if pin is found
//-----------------------------------------------------------
uint8_t HNode::deactivate(char* name)
{
  Pin* pin = getPinByName(name);
  
  if (pin == 0) return false;
  pin->deactivate();
// #if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  // Serial << "=> deactivate " << name << endl;
// #endif
  return true;
}
//-----------------------------------------------------------
// Get all outputs pin with same value
// Parameters: value
// Returns:	none
//-----------------------------------------------------------
void HNode::changeAll(uint8_t value)
{
  //Set value to all pins
  for(uint8_t i=0;i<info.pins;i++) 
      if (_pin[i]->type == pinOutput) {
        _pin[i]->change(value);
      }
}
//-----------------------------------------------------------
// Get pin by name
// Parameters: pin name
// Returns:	pin information or none if not found
//-----------------------------------------------------------
Pin* HNode::getPinByName(char* name)
{
  for(uint8_t i=0;i < info.pins;i++)
  {
    if (strcmp(name, _pin[i]->name) == 0)
    {
// #if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  // Serial << "=> getPin " << name << endl;
// #endif
      return _pin[i];
    }
  }
  return 0;
}
//-----------------------------------------------------------
// Get pin by index & type
// Parameters: pin index - type
// Returns:	pin information or none if not found
//-----------------------------------------------------------
Pin* HNode::getPinByType(uint8_t index, hb_pin_type type)
{
  for(uint8_t i=0;i < info.pins;i++)
  {
    if (_pin[i]->index == index && _pin[i]->type == type)
    {
// #if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  // Serial << "getPin " << index << "," << type << endl;
  // Serial << "=> name " << _pin[i]->name << endl;
// #endif
      return _pin[i];
    }
  }
  return 0;
}
#endif //End of pins section
//===========================================================
// Devices commands
//===========================================================
#if defined(USE_DEVICES) && USE_DEVICES == 1
//-----------------------------------------------------------
// Add device to the node
//-----------------------------------------------------------
uint8_t HNode::addDevice(Device *device) 
{
  if (info.devices == MAX_DEVICES)  return 0xff;

  uint8_t i = info.devices++;
    
  //Configure device
  _device[i] = device;
  _device[i]->info.index = i;
  _device[i]->controller = this;
  
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  // Serial << "=> addDevice " << endl;
  // Serial << "  " << _device[i]->info.name << endl;
#endif
  return i;
}

//-----------------------------------------------------------
// Returns device information
//-----------------------------------------------------------
void HNode::getDeviceInfo(uint8_t index)
{
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  // Serial << "=> getDeviceInfo " << index << endl;
#endif
    if (info.devices >= index) {
        _device[index]->serializeInfo(_tx);
    }
    else
        _bus->setError(DEVICE_NOT_FOUND);
}
//-----------------------------------------------------------
// Returns device status
//-----------------------------------------------------------
uint8_t HNode::getDeviceStatus(char* name)
{
    Device *dev = getDevice(name);
    if (dev == 0) return false;
    _tx->clearWriteIndex();
    _tx->pushString((char*)dev->status());
    
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  // Serial << "=> getDeviceStatus " << endl;
#endif
    return true;
}
//-----------------------------------------------------------
//Execute device action
//-----------------------------------------------------------
uint8_t HNode::executeDeviceAction(char* name, hb_device_action_t* action)
{
    Device *dev = getDevice(name);
    

    if (dev == 0) return false;
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  // Serial << "=> executeDeviceAction " << endl;
#endif
    //Execute device action
    return dev->executeAction(action);
}
//-----------------------------------------------------------
// Add device listener to device events
//-----------------------------------------------------------
uint8_t HNode::addDeviceListener(char* name, hb_address_t source, uint8_t port)
{
    for(uint8_t i = 0; i < info.devices; i++)
    {
        //Search device
        if (strcmp(name, _device[i]->info.name) == 0 || strcmp(name, EMPTY_NAME) == 0) {
          _device[i]->listener = source;
          _device[i]->listenerPort = port;

#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  // Serial << "=> addDeviceListener " << endl;
#endif
          return true;
        }
    }
    return false;
}
//-----------------------------------------------------------
// Remove device listener
//-----------------------------------------------------------
uint8_t HNode::delDeviceListener(char* name, hb_address_t source)
{
    for(uint8_t i = 0; i < info.devices; i++)
    {
        //Search device
        if (strcmp(name, _device[i]->info.name) == 0 || strcmp(name, EMPTY_NAME) == 0) {
            _device[i]->listener = 0;
            _device[i]->listenerPort = 0;
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  // Serial << "=> delDeviceListener " << endl;
#endif
            return true;
        }
    }
    return false;
}
//-----------------------------------------------------------
//Device event handler
//-----------------------------------------------------------
void HNode::onDeviceEvent(hb_device_event_t *device_event) 
{
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  // Serial << "=> onDeviceEvent "<< device_event->device << " - " << device_event->event << endl;
#endif
  
  if (device_event == 0) return;

  //Send event to remote listener
  for(uint8_t i = 0; i < info.devices; i++) {
    //Search device listener
    if (strcmp(device_event->device, _device[i]->info.name) == 0 && _device[i]->listener != 0) {
      _tx->clearWriteIndex();
      _tx->pushName(_device[i]->info.name);
      _tx->pushString(device_event->event);
      _tx->pushLong(_scheduler->getTime());
      _tx->pushArray(device_event->value, device_event->values);
      //Send device event to listener
      _send_dev_evt = i;
      //digitalWrite(LED_PIN, digitalRead(PIR_PIN));
      break;
    }
  }
  //Send event to external handler
  if (onDeviceEventCallback != 0)
    onDeviceEventCallback(*device_event);
}
//-----------------------------------------------------------
// Return device by name
//-----------------------------------------------------------
Device* HNode::getDevice(char* name)
{
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  // Serial << "=> getDevice "<< name << endl;
#endif
  for(uint8_t i=0;i < info.devices;i++)
  {
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
Serial << "? "<< _device[i]->info.name << " == " << name << endl;
#endif
      if (strcmp(name, _device[i]->info.name) == 0)
      {
          return _device[i];
      }
  }
  return 0;
}
#endif //End of devices section
//===========================================================
// Sensor commands
//===========================================================
#if defined(USE_SENSORS) && USE_SENSORS == 1
//-----------------------------------------------------------
// Add sensor to the node
//-----------------------------------------------------------
uint8_t HNode::addSensor(Sensor *sensor) 
{

  if (info.sensors == MAX_SENSORS)  return 0xff;

    uint8_t i = info.sensors++;
    
    //Configure sensor
    _sensor[i] = sensor;
    _sensor[i]->info.index = i;
    _sensor[i]->scheduler = _scheduler;
    _sensor[i]->controller = this;
    
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  // Serial << "=> addSensor " << i << ":" << _sensor[i]->info.name << endl;
#endif

    return i;
}
//-----------------------------------------------------------
// Return sensor information
//-----------------------------------------------------------
void HNode::getSensorInfo(uint8_t index)
{
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  // Serial << "=> getSensorInfo " << index << ":" << _sensor[index]->info.name << endl;
#endif
    if (info.sensors >= index) {
        _sensor[index]->serializeInfo(_tx);
    }
    else
        _bus->setError(SENSOR_NOT_FOUND);
}
//-----------------------------------------------------------
// Read sensor value and return it
//-----------------------------------------------------------
hb_sensor_read_t HNode::readSensor(char* name)
{
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  Serial << "\tsensorRead: " << name << endl;
#endif
  Sensor *sensor = getSensor(name);

  hb_sensor_read_t value;

  if (sensor != 0) ;
    sensor->read(&value);
  
  return value;
}
//-----------------------------------------------------------
// Read sensor value and push into tx stack
//-----------------------------------------------------------
uint8_t HNode::pushSensorRead(char* name)
{
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  Serial << "\tpushSensorRead: " << name << endl;
#endif
  Sensor *sensor = getSensor(name);

  if (sensor == 0) return false;

  hb_sensor_read_t value;

  sensor->read(&value);
  
  //Push sensor read into stack
  _tx->clearWriteIndex();
  _tx->pushName(name);
  _tx->pushLong(value.time);
  _tx->pushFloat(value.value);

  return true;
}
//-----------------------------------------------------------
// Reset sensor status
//-----------------------------------------------------------
uint8_t HNode::resetSensor(char* name)
{
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  // Serial << "=> resetSensor "<< name << endl;
#endif
    Sensor *sensor = getSensor(name);
    
    if (sensor == 0) return false;
    
    sensor->reset();
    
    return true;
}
//-----------------------------------------------------------
// Add sensor listener
//-----------------------------------------------------------
uint8_t HNode::addSensorListener(char* name, hb_address_t source, uint8_t port, uint8_t interval, uint16_t expire)
{
    for(uint8_t i = 0; i < info.sensors; i++)
    {
        //Search sensor
        if (strcmp(name, _sensor[i]->info.name) == 0 || strcmp(name, EMPTY_NAME) == 0) {
            _sensor[i]->listener = source;
            _sensor[i]->listenerPort = port;
            _sensor[i]->interval = interval;
            _sensor[i]->expire = expire;
            _sensor[i]->count = interval;
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
   // Serial << "=> addSL " << name 
     // << " s: " << _sensor[i]->listener
     // << " p: " << _sensor[i]->listenerPort
     // << " i: " << _sensor[i]->interval
     // << " e: " << _sensor[i]->expire
     // << endl;
#endif
            return true;
        }
    }
    return false;
}
//-----------------------------------------------------------
// Remove sensor listener
//-----------------------------------------------------------
uint8_t HNode::delSensorListener(char* name, hb_address_t source)
{
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  // Serial << "=> delSensorListener "<< name << endl;
#endif
    for(uint8_t i = 0; i < info.sensors; i++)
    {
        //Search pin
        if (strcmp(name, _sensor[i]->info.name) == 0 || strcmp(name, EMPTY_NAME) == 0)  {
            _sensor[i]->listener = 0;
            _sensor[i]->listenerPort = 0;
            return true;
        }
    }
    return false;
}
//-----------------------------------------------------------
// Return sensor by name
//-----------------------------------------------------------
Sensor* HNode::getSensor(char* name)
{
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  // Serial << "=> getSensor "<< name << endl;
#endif
    for(uint8_t i=0;i < info.sensors;i++)
    {
        if (strcmp(name, _sensor[i]->info.name) == 0)
        {
            return _sensor[i];
        }
    }
    return 0;
}

// //-----------------------------------------------------------
// // Sensor read event
// //-----------------------------------------------------------
void HNode::onSensorRead(hb_sensor_read_t sensor_read) 
{
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  // Serial << "=> onSensorRead " << endl;
#endif

  //Send event to external handler
  if (onSensorReadCallback != 0)
    onSensorReadCallback(sensor_read);
}

// -------------------------------------------------------
// Send sensor readings to connected listeners
// Parameters: none
// Returns:	none
// -------------------------------------------------------
void HNode::feedSensorListeners()
{
  hb_sensor_read_t value;
  
  for(uint8_t i=0;i< info.sensors;i++)
  {
    if (_sensor[i]->listener) 
      if (_sensor[i]->count == 0 && _sensor[i]-> expire > 0) {
        
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
 Serial << "\ts[" << i << "] " << _sensor[i]->info.name << " => "
  << _sensor[i]->listener << ":" << _sensor[i]->listenerPort 
  << " cn: " << _sensor[i]->count
  << " ex: " << _sensor[i]->expire
  <<  endl;
#endif
      _sensor[i]->read(&value);
      //Push sensor read into stack
      _tx->clearWriteIndex();
      _tx->pushName(_sensor[i]->info.name);
      _tx->pushLong(value.time);
      _tx->pushFloat(value.value);
      _sensor[i]->count = _sensor[i]->interval;
      if (_sensor[i]->expire<0xffff) 
        _sensor[i]->expire--;
      //Send sensor read
      _bus->sendImmediate(CMD_PUSH_SENSOR_READ, _sensor[i]->listener, _sensor[i]->listenerPort);
    }
    else
      _sensor[i]->count--;
  }
}
#endif //End of sensors
//===========================================================
// Various
//===========================================================
#if defined(USE_KEYPAD) && USE_KEYPAD == 1
// -------------------------------------------------------
// Serialize last pressed key into tx stack
// Parameters: none
// Returns:	none
// -------------------------------------------------------
void HNode::readLastKey()
{
    _tx->clearWriteIndex();
    _tx->pushByte(keypad.getKey());
}
#endif 

//===========================================================
// General information
//===========================================================
//===========================================================
// Event handlers
//===========================================================
//-----------------------------------------------------------
// Process HBus command
//-----------------------------------------------------------
uint8_t HNode::processCommand(hb_message_t message, uint8_t port) 
{

  if (_status != NODE_ACTIVE) return false;

  //Init rx & tx stacks
  _rx->setData(message.data, message.length);
  _rx->clearReadIndex();
  _tx->clearWriteIndex();
  _bus->resetError();

  uint8_t processed = true;
  Pin *new_pin;
  Wire *new_wire;
  hb_sensor_read_t value;
    
  switch(message.command)
  {
    //---------------------------------------------------------------
    // Map commands
    //---------------------------------------------------------------
    case CMD_PING:
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
//      Serial << "=> CMD_PING" << endl;
#endif
      _tx->pushName(info.name);
      break;
    //---------------------------------------------------------------
    // Configuration commands
    //---------------------------------------------------------------
    case CMD_RESET:
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  // Serial << "=> CMD_RESET" << endl;
#endif
      _value[0] = _rx->popByte();	//full reset
      reset(_value[0]);
      break;
    case CMD_START:
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
//      Serial << "=> CMD_START" << endl;
#endif
      start();
      break;
    case CMD_STOP:
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
//      Serial << "=> CMD_STOP" << endl;
#endif
      stop();
      break;
#if defined(READ_CONFIGURATION) && READ_CONFIGURATION == 1
    case CMD_FACTORY_DEFAULT:
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
//      Serial << "=> CMD_FACTORY_DEFAULT" << endl;
#endif
      readConfiguration(true);
      break;
    case CMD_READ_CONFIG:
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
//      Serial << "=> CMD_READ_CONFIG" << endl;
#endif
      readConfiguration(false);
      break;
#endif                    
#if defined(WRITE_CONFIGURATION) && WRITE_CONFIGURATION == 1
    case CMD_WRITE_CONFIG:
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
//      Serial << "=> CMD_WRITE_CONFIG" << endl;
#endif
      writeConfiguration(false);
      break;
#endif                    
    case CMD_ADD_NODE_LISTENER:
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
//      Serial << "=> CMD_ADD_NODE_LISTENER" << endl;
#endif
      _value[0] = _rx->popByte();	//mask
      processed = addNodeListener(_value[0], message.source, port);
      break;
    case CMD_DELETE_NODE_LISTENER:
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
//      Serial << "=> CMD_DELETE_NODE_LISTENER" << endl;
#endif
      processed = delNodeListener(message.source);
      break;
#if defined(USE_PINS) && USE_PINS == 1        
    //---------------------------------------------------------------
    // Pin commands
    //---------------------------------------------------------------
    case CMD_SET_PIN:
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
//      Serial << "=> CMD_SET_PIN" << endl;
#endif
      _tmp_pin  = new Pin(message.data, message.length);
      addPin(new_pin);
      break;
    case CMD_CONNECT:
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
//      Serial << "=> CMD_CONNECT" << endl;
#endif
      new_wire = new Wire(message.data, message.length);
      addWire(_tmp_wire);
      break;
    case CMD_ADD_PIN_LISTENER:
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
//      Serial << "=> CMD_ADD_PIN_LISTENER" << endl;
#endif
      _rx->popName(_tmp_name); //Device name
      processed = addPinListener(_tmp_name, message.source, port);
      break;
    case CMD_DELETE_PIN_LISTENER:
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
//      Serial << "=> CMD_DELETE_PIN_LISTENER" << endl;
#endif
      _rx->popName(_tmp_name); //Device name
      processed = delPinListener(_tmp_name, message.source);
      break;
    //---------------------------------------------------------------
    // Activation commands
    //---------------------------------------------------------------
    case CMD_ACTIVATE:
      _rx->popName(_tmp_name); //name
      processed = activate(_tmp_name);
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
Serial << "CMD_ACTIVATE" << _tmp_name << endl;
#endif
      break;
    case CMD_DEACTIVATE:
      _rx->popName(_tmp_name); //name
      processed = deactivate(_tmp_name);
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
//Serial << "CMD_DEACTIVATE" << _tmp_name << endl;
#endif
      break;
    // case CMD_MULTI_ACTIVATE:
      // _value[0] = _rx->popByte();	//delay
      // _value[1] = _rx->popByte();	//length
      // //Extract name array
      // _rx->popFixedStringArray((char**) names, _value[1], NAME_LENGTH);
      // multiActivate(_value[0], _value[1], (char**)names);
      // break;
    //---------------------------------------------------------------
    // Write commands
    //---------------------------------------------------------------
    case CMD_CHANGE_ALL_DIGITAL:
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  Serial << "=> CMD_CHANGE_ALL_DIGITAL" << endl;
#endif
      //0 = OFF | 1 = ON
      _value[0] = _rx->popByte();
      changeAll(_value[0]);
      break;

    case CMD_CHANGE_DIGITAL:
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  Serial << "=> CMD_CHANGE_DIGITAL" << endl;
#endif
      //output index
      //0 = OFF | 1 = ON
      _value[0] = _rx->popByte();	//index
      _value[1] = _rx->popByte();   //_value
      // changeOut(_value[0], _value[1]);
      _tmp_pin = getPinByType(_value[0], pinOutput);
      if (_tmp_pin != 0)
        _tmp_pin->change(_value[1]);
      else
        _bus->setError(PIN_NOT_FOUND);
      break;

    case CMD_TOGGLE_DIGITAL:
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  Serial << "=> CMD_TOGGLE_DIGITAL" << endl;
#endif
      _value[0] = _rx->popByte();	//index
      _tmp_pin = getPinByType(_value[0], pinOutput);
      //toggleOut(_value[0]);
      if (_tmp_pin != 0)
        _tmp_pin->toggle();
      else
        _bus->setError(PIN_NOT_FOUND);
      break;

    case CMD_TIMED_DIGITAL:
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  Serial << "=> CMD_TIMED_DIGITAL" << endl;
#endif
      _value[0] = _rx->popByte();	//index
      _value[1] = _rx->popByte();	//width
      _value[2] = _rx->popByte();	//value
      _tmp_pin = getPinByType(_value[0], pinOutput);
      if (_tmp_pin != 0)
        _tmp_pin->timed(_value[1], _value[2]);
      else
        _bus->setError(PIN_NOT_FOUND);
      break;

    case CMD_DELAY_DIGITAL:
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  Serial << "=> CMD_DELAY_DIGITAL" << endl;
#endif
      _value[0] = _rx->popByte();	//index
      _value[1] = _rx->popByte();	//delay
      _value[2] = _rx->popByte();	//value
      _tmp_pin = getPinByType(_value[0], pinOutput);
      if (_tmp_pin != 0)
        _tmp_pin->delayed(_value[1], _value[2]);
      else
        _bus->setError(PIN_NOT_FOUND);
      break;
    case CMD_DELAY_TOGGLE_DIGITAL:
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
 Serial << "CMD_DELAY_TOGGLE_DIGITAL" << endl;
#endif
      _value[0] = _rx->popByte();	//index
      _value[1] = _rx->popByte();	//delay
      _tmp_pin = getPinByType(_value[0], pinOutput);
      if (_tmp_pin != 0)
        _tmp_pin->delayedToggle(_value[1]);
      else
        _bus->setError(PIN_NOT_FOUND);
      break;

    case CMD_PULSE_DIGITAL:
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  Serial << "=> CMD_PULSE_DIGITAL" << endl;
#endif
      _value[0] = _rx->popByte();	//index
      _value[1] = _rx->popByte();	//delay
      _value[2] = _rx->popByte();	//width
      _value[3] = _rx->popByte();	//value
      _tmp_pin = getPinByType(_value[0], pinOutput);
      if (_tmp_pin != 0)
        _tmp_pin->pulsed(_value[1], _value[2], _value[3]);
      else
        _bus->setError(PIN_NOT_FOUND);
      break;

    case CMD_CYCLE_DIGITAL:
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
//      Serial << "=> CMD_CYCLE_DIGITAL" << endl;
#endif
      _value[0] = _rx->popByte();	//index
      _value[1] = _rx->popByte();	//delay
      _value[2] = _rx->popByte();	//width
      _value[3] = _rx->popByte();	//cycles
      _tmp_pin = getPinByType(_value[0], pinOutput);
      if (_tmp_pin != 0)
        _tmp_pin->cycled(_value[1], _value[2], _value[3]);
      else
        _bus->setError(PIN_NOT_FOUND);
      break;

    case CMD_CHANGE_PWM:
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
//      Serial << "=> CMD_CHANGE_PWM" << endl;
#endif
      _value[0] = _rx->popByte();	//index
      _value[1] = _rx->popByte();	//high width
      _value[2] = _rx->popByte();	//total width
      _tmp_pin = getPinByType(_value[0], pinPwm);
      if (_tmp_pin != 0)
        _tmp_pin->pwm(_value[1], _value[2]);
      else
        _bus->setError(PIN_NOT_FOUND);
      break;

    case CMD_DELTA_PWM:
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
//      Serial << "=> CMD_DELTA_PWM" << endl;
#endif
      _value[0] = _rx->popByte();	      //index
      _int_value[0] = _rx->popInteger();// Delta value
      // analogDeltaOut(_value[0], (int) _int_value[0]);
      _tmp_pin = getPinByType(_value[0], pinPwm);
      if (_tmp_pin != 0)
        _tmp_pin->change(_int_value[0]);
      else
        _bus->setError(PIN_NOT_FOUND);
      break;
      
    case CMD_FADE_PWM:
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
//      Serial << "=> CMD_FADE_PWM" << endl;
#endif
      _value[0] = _rx->popByte();	//index
      _int_value[0] = _rx->popInteger();//value start
      _int_value[1] = _rx->popInteger();//value end
      _value[1] = _rx->popByte();	//steps
      _value[2] = _rx->popByte();	//delay in 1/10 secs
      _tmp_pin = getPinByType(_value[0], pinPwm);
      if (_tmp_pin != 0)
        _tmp_pin->fade(_int_value[0], _int_value[1], _value[1], _value[2]);
      else
        _bus->setError(PIN_NOT_FOUND);
      break;

    case CMD_CHANGE_PIN:
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
//      Serial << "=> CMD_CHANGE_PIN" << endl;
#endif
      _rx->popName(_tmp_name);  //name
      _tmp_pin = getPinByName(_tmp_name);
      
      if (_tmp_pin != 0) {
        _int_value[0] = _rx->popInteger();         // pin value
        _tmp_pin->change(_int_value[0]);
        processed = true;
      }
      else
        processed = false;
      break;

    //---------------------------------------------------------------
    // Read commands
    //---------------------------------------------------------------
    case CMD_READ_PIN:
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
//      Serial << "=> CMD_READ_PIN" << endl;
#endif
      _rx->popName(_tmp_name); //Pin name
      _tmp_pin = getPinByName(_tmp_name);
      if (_tmp_pin != 0) {
        _tx->pushLong(_tmp_pin->read());
        processed = true;
      }
      else
        processed = false;
      break;
    case CMD_READ_ACTIVE:
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
//      Serial << "=> CMD_READ_ACTIVE" << endl;
#endif
      _rx->popName(_tmp_name); //Pin name
      _tmp_pin = getPinByName(_tmp_name);
      if (_tmp_pin != 0) {
        _tx->pushByte(_tmp_pin->isActive());
        processed = true;
      }
      else
        processed = false;
      break;
    case CMD_READ_LAST_INPUT:
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
//      Serial << "=> CMD_READ_LAST_INPUT" << endl;
#endif
      if (strlen(_last_input) > 0 ) {
          _tx->pushFixedString(_last_input, NAME_LENGTH);
          //Clear last activated input
          strcpy_P(_last_input, PSTR(""));
      }
      else
          _tx->pushFixedString("", NAME_LENGTH);
      break;
    case CMD_READ_LAST_ACTIVE:
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
//      Serial << "=> CMD_READ_LAST_ACTIVE" << endl;
#endif
      if (strlen(_last_output) > 0 ) {
        _tx->pushFixedString(_last_output, NAME_LENGTH);
        //Clear last activated output
        strcpy_P(_last_output, PSTR(""));
      }
      else
          _tx->pushFixedString("", NAME_LENGTH);
      break;
#endif
#if defined(USE_KEYPAD) && USE_KEYPAD == 1
    case CMD_READ_KEY:
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
//      Serial << "=> CMD_READ_KEY" << endl;
#endif
      _tx->pushByte(keypad.getKey());
      break;
#endif
    //---------------------------------------------------------------
    // Information commands
    //---------------------------------------------------------------
    case CMD_READ_ALL:
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
//      Serial << "=> CMD_READ_ALL" << endl;
#endif
      //read type: (BITMASK)
      // 	255 = all
      // 	1 = inputs
      // 	2 = outputs
      // 	4 = analogs
      // 	8 = counters
      // 	16 = pwms
      // 	32 = sensors
      _value[0] = _rx->popByte();	//mask
      readAll(_value[0]);
      break;
    case CMD_GET_INFO:
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
// Serial << "=> CMD_GET_INFO" << endl;
// Serial << "\t inputs  " << info.inputs << endl;
// Serial << "\t outputs " << info.outputs << endl;
// Serial << "\t analogs " << info.analogs << endl;
// Serial << "\t pwms " << info.pwms << endl;
// Serial << "\t wires " << info.wires << endl;
// Serial << "\t devices " << info.devices << endl;
// Serial << "\t sensors " << info.sensors << endl;
#endif          
      info.save(_tx);
      break;
#if defined(USE_PINS) && USE_PINS == 1        
    case CMD_GET_PIN_INFO:
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
//      Serial << "=> CMD_GET_PIN_INFO" << endl;
#endif
      
      _value[0] = _rx->popByte(); //index
      _value[1] = _rx->popByte(); //type
      
      getPinByType(_value[0], (hb_pin_type) _value[1]);
      // _tmp_pin = getPin(_value[0], (hb_pin_type) _value[1]);
      // //If pin found get information
      // if (_tmp_pin != 0) 
        // _tmp_pin->save(_tx);
      // else
        // _bus->setError(PIN_NOT_FOUND);
      break;
    case CMD_GET_CONNECT_INFO:
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
//      Serial << "=> CMD_GET_CONNECT_INFO" << endl;
#endif
      _value[0] = _rx->popByte();	//index
      getWire(_value[0]);
      // if (info.wires > _value[0])
        // _wire[_value[0]]->save(_tx);
      // else
        // _bus->setError(WIRE_NOT_FOUND);
      break;
#endif                    
    case CMD_GET_NAME_INFO:
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
//     Serial << "=> CMD_GET_NAME_INFO" << endl;
#endif
      _rx->popName(_tmp_name);
      processed = getNameInfo(_tmp_name);
      break;
#if defined(USE_DEVICES) && USE_DEVICES == 1
    //---------------------------------------------------------------
    // Device commands
    //---------------------------------------------------------------
    case CMD_GET_DEVICE_INFO:
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
//      Serial << "=> CMD_GET_DEVICE_INFO" << endl;
#endif
      _value[0] = _rx->popByte();	//index
      getDeviceInfo(_value[0]);
      break;
    case CMD_GET_DEVICE_STATUS:
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
//      Serial << "=> CMD_GET_DEVICE_STATUS" << endl;
#endif
      _rx->popName(_tmp_name);
      getDeviceStatus(_tmp_name);
      break;
    case CMD_EXECUTE_DEVICE_ACTION:
      //Get device by name
      _rx->popName(_tmp_name);
      //get device action
      _rx->popString(_action.action);
      //Get action values
      _rx->popArray(_action.value, &_action.values);
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
// Serial << "CMD_EXECUTE_DEVICE_ACTION" << _tmp_name << " " << _action.action << endl;
#endif
      //Execute device action
      processed = executeDeviceAction(_tmp_name, &_action);
      break;
    case CMD_ADD_DEVICE_LISTENER:
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
//      Serial << "=> CMD_ADD_DEVICE_LISTENER" << endl;
#endif
      _rx->popName(_tmp_name); //name
      processed = addDeviceListener(_tmp_name, message.source, port);
      break;
    case CMD_DELETE_DEVICE_LISTENER:
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
//      Serial << "=> CMD_DELETE_DEVICE_LISTENER" << endl;
#endif
      _rx->popName(_tmp_name); //name
      processed = delDeviceListener(_tmp_name, message.source);
      break;
    case CMD_PUSH_DEVICE_EVENT:
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
//      Serial << "=> CMD_PUSH_DEVICE_EVENT" << endl;
#endif
      _rx->popName(_dev_event.device);      //Device name
      _rx->popString(_dev_event.event);     //Device event name
      _dev_event.time = _rx->popLong();     //Device event time
      
      _rx->popArray(_dev_event.value, _int_value); //Device event values
      _dev_event.values = _int_value[0];
      
      //Call external handler
      if (onDeviceEventCallback!= 0)
        processed = onDeviceEventCallback(_dev_event);
      break;
#endif                    
#if defined(USE_SENSORS) && USE_SENSORS == 1
    //---------------------------------------------------------------
    // Sensor commands
    //---------------------------------------------------------------
    case CMD_GET_SENSOR_INFO:
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  Serial << "=> CMD_GET_SENSOR_INFO" << endl;
#endif
      _value[0] = _rx->popByte();	//index
      getSensorInfo(_value[0]);
      break;
    case CMD_READ_SENSOR:
      _rx->popName(_tmp_name); //name
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  Serial << "=> CMD_READ_SENSOR" << _tmp_name << endl;
#endif
      processed = pushSensorRead(_tmp_name);
      break;
    case CMD_RESET_SENSOR:
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  Serial << "=> CMD_RESET_SENSOR" << endl;
#endif
      //Sensor name
      _rx->popName(_tmp_name); //name
      processed = resetSensor(_tmp_name);
      break;
    case CMD_ADD_SENSOR_LISTENER:
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  Serial << "=> CMD_ADD_SENSOR_LISTENER" << endl;
#endif
      _rx->popName(_tmp_name); //name
      _value[0] = _rx->popByte();	//interval in seconds
      _int_value[0] = _rx->popInteger();// expire in seconds
      processed = addSensorListener(_tmp_name, message.source, port, _value[0], _int_value[0]);
      break;
    case CMD_DELETE_SENSOR_LISTENER:
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
 Serial << "=> CMD_DELETE_SENSOR_LISTENER" << endl;
#endif
      _rx->popName(_tmp_name); //name
      processed = delSensorListener(_tmp_name, message.source);
      break;
    case CMD_PUSH_SENSOR_READ:
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
      Serial << "=> CMD_PUSH_SENSOR_READ" << endl;
#endif
      //Pop sensor read from stack
      _rx->popName(value.name);
      value.time = _rx->popLong();
      value.value= _rx->popFloat();
      //Call external handler
      if (onSensorReadCallback!= 0)
        processed = onSensorReadCallback(value);
      break;
#endif                    
    default:
        processed = false;
        break;
  }
  //delay(100); //Time to settle from rx to tx
  return processed;
}
//-----------------------------------------------------------
// Process received ack
// HBus ack handler for reeceived acks from nodes
// Parameters: Received ack
// Returns:	true if success
//-----------------------------------------------------------
uint8_t HNode::processAck(hb_message_t ack, uint8_t port) 
{
  hb_sensor_read_t value;

  //Init rx stacks
  _rx->setData(ack.data, ack.length);
  _rx->clearReadIndex();
  _bus->resetError();

  uint8_t processed = true;

  if ((ack.flags & ACK_MSG) != 0)
    switch(ack.command)
    {
      case CMD_READ_SENSOR:
        //Pop sensor read from stack
        _rx->clear();
        _rx->popName(value.name);
        value.time = _rx->popLong();
        value.value= _rx->popFloat();
        //Call external handler
        if (onSensorReadCallback!= 0)
          return onSensorReadCallback(value);
        break;
    }
  return true; //not really processed from HBus nodes
}
//===========================================================
// DEBUG
//===========================================================
//-----------------------------------------------------------
// Returns free ram available
//-----------------------------------------------------------
int HNode::freeRam () 
{
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}


//End of HNode
#endif