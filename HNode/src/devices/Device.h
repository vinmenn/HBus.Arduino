/*
  =======================================================
  HBus - Home automation/IoT application protocol 
  
  Shutter device
  
  Copyright (C) 2014-15-16 Vincenzo Mennella (see license.txt)

  History
    1.0.0 10/08/2014:   First revision of separated code
    1.0.1 01/08/2015:   Review code
    1.0.2 11/10/2015:   Added additional device information
  =======================================================
*/
#ifndef HBUS_DEVICE_H
#define HBUS_DEVICE_H
#define VERSION_HBUS_DEVICE_H   "1.0.1"

#include <Arduino.h> 
#include <SimpleStack.h>
#include <Scheduler.h>
#include <HBus.h>

// Device information
typedef struct {
	char name[NAME_LENGTH+1];              // Name
  uint8_t index;                         // Device index
	char action [MAX_ACTIONS][MAX_STRING];  // Actions array
	uint8_t actions;     		               // Actions number
} hb_device_info_t;

//Device action
typedef struct {
	//char* action;
	char action[MAX_STRING];
  uint8_t value[MAX_PARAMETERS];
  uint16_t values;
} hb_device_action_t;

//Device event
typedef struct {
	char device[NAME_LENGTH+1];
	char event[MAX_STRING];
  uint32_t time;
	uint8_t value[MAX_PARAMETERS];
  uint16_t values;
} hb_device_event_t;

//External handler
extern "C" {
	// callback function types
    typedef uint8_t (*deviceEventCallback)(hb_device_event_t deviceEvent);
} 

class HBusDeviceController
{
   public:
      virtual void onDeviceEvent(hb_device_event_t *deviceEvent) {};
};
//Device interface
class Device
{
  public:
    //-----------------------------------------------------------
    //Methods
    //-----------------------------------------------------------
        
    //General
    virtual void begin() {}
    virtual const char* status() { return 0;}
    virtual uint8_t isActive() { return 0;}
    virtual uint8_t executeAction(hb_device_action_t *action) { return 0;}
    virtual void reset() {}
    virtual void dispose() {}

    //Configuration
    //de-serialize configuration from stack
    virtual void deserializeConfig(SimpleStack *ss) { }
    // serialize configuration into stack
    virtual void serializeConfig(SimpleStack *ss) { }

    //Conversions
    char* toString() { return info.name; };
        
    // serialize device information into stack
    void serializeInfo(SimpleStack *ss) {
        
        char action[MAX_STRING];
        
        ss->pushByte(info.index);
        ss->pushName(info.name);
        ss->pushString(""); //Description
        ss->pushString(""); //Location
        ss->pushString(""); //Class
        ss->pushString(""); //Hardware
        ss->pushString(""); //Version
        ss->pushByte(info.actions);
        if (info.actions > 0)
          for(uint8_t i=0;i<info.actions;i++) {
            strcpy_P(action, info.action[i]);
            ss->pushString(action);
        }
    }
    // de-serialize device information into stack
    void deserializeInfo(SimpleStack *ss) {
        
        char action[MAX_STRING];
        
        info.index = ss->popByte();
        ss->popName(info.name);
        ss->popString(action); //Description
        ss->popString(action); //Location
        ss->popString(action); //Class
        ss->popString(action); //Hardware
        ss->popString(action); //Version
        //Actions
        info.actions = ss->popByte();
        if (info.actions > 0)
          for(uint8_t i=0;i<info.actions;i++) {
            ss->popString(action);
            strcpy(info.action[i], action);
        }
    }
    //-----------------------------------------------------------
    //Properties
    //-----------------------------------------------------------
    hb_device_info_t info;
    Scheduler *scheduler;
    HBusDeviceController *controller;
    hb_address_t listener;
    uint8_t listenerPort;
};

class DeviceSchedule : public ISchedule {
    private:
      Device* _device;
      hb_device_action_t* _action;
        
    public:
      DeviceSchedule();
      DeviceSchedule(Device *pdevice, hb_device_action_t *paction, uint32_t pdelay, uint8_t preschedule);
      //Execute device action
      virtual void execute();
};

DeviceSchedule::DeviceSchedule() {
  _device = 0;
  id = 0;
  name = 0;
  time = 0;
  delay = 0;
  _device = 0;
  _action = 0;
}
DeviceSchedule::DeviceSchedule(Device *pdevice, hb_device_action_t *paction, uint32_t pdelay, uint8_t preschedule) {
  id = 0;
  name = pdevice->info.name;
  time = 0;
  delay = pdelay;
  reschedule = preschedule;

  _device = pdevice;
  _action = paction;
}
        //Execute device action
void DeviceSchedule::execute() {
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  //Serial << "devsched " << time << endl;
#endif
    // Serial << " device execute " 
        // << _device->info.name << " => "
        // << _action.action
        // << endl;
    if (_device != 0)
      _device->executeAction(_action);
}

#endif