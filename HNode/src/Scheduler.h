/*
  =======================================================
  HNode - Home automation/IoT node for HBus project
  
  Node scheduler
  
  Copyright (C) 2014-15 Vincenzo Mennella (see license.txt)

  History
    1.0.0 12/08/2014:   First revision of separated code
    1.0.1 01/08/2015:   Reviewed code after 1 year
  =======================================================
*/
#ifndef HNODE_SCHEDULER_H
#define HNODE_SCHEDULER_H
#define VERSION_HNODE_SCHEDULER_H   "1.0.1"

//-----------------------------------------------------------
// External libraries
//-----------------------------------------------------------
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
#include <Streaming.h> 
#endif

//===========================================================
// Default schedule interface for scheduler
// Is used to implement specific action called by scheduler
//
// All schedules implementations must be compliant
//===========================================================
class ISchedule 
{
    public:
    uint8_t id;
    char *name;
    uint32_t time;
    uint32_t delay;
    uint8_t reschedule;
    virtual void execute() { };
};

//===========================================================
// Scheduler engine
// Trigger specific schedules implementations when time expires
//===========================================================
class Scheduler {
  private:
		static const byte MAX_SCHEDULES = 32;

    unsigned long _time;
    uint32_t _timeIndex;
    uint8_t _start;
    uint8_t _schedules;
    ISchedule* _schedule[MAX_SCHEDULES];
        
  public:
    Scheduler();
    void update();
    int add(ISchedule *schedule);
    void purge(char *name);
    int del(uint8_t id);
    void clean();
    void start();
    void stop();
    uint32_t getTime();
};
//===========================================================
//Constructors
//===========================================================
// -------------------------------------------------------
// -------------------------------------------------------
Scheduler::Scheduler() 
{
  _start = 0;
  _time = 0;
  _timeIndex = 0;
  _schedules = 0;
  for (uint8_t i=0; i < MAX_SCHEDULES; i++)
    _schedule[i] = 0;
}
//===========================================================
//Methods
//===========================================================

// -------------------------------------------------------
// -------------------------------------------------------
void Scheduler::update() 
{
  unsigned long time = millis();
  
  if ((time - _time) >= 1000) {
      _timeIndex++;
      _time = time;
  }
      
  if (_start == 0) return;
  
  for (uint8_t i=0; i < MAX_SCHEDULES; i++){
    if (_schedule[i] != 0 && _schedule[i]->time <= _time) {
      _schedule[i]->execute();
      if (_schedule[i]->reschedule) {
        _schedule[i]->time = millis() + _schedule[i]->delay;
      }
      else {
        free(_schedule[i]);
        _schedule[i] = 0;
      }
    }
  }
}
// -------------------------------------------------------
// -------------------------------------------------------
int Scheduler::add(ISchedule *schedule) 
{
  for (uint8_t i=0; i < MAX_SCHEDULES; i++){
    if (_schedule[i] == 0) {
      
      _schedule[i] = schedule;
      _schedule[i]->id = i;
      _schedule[i]->time = millis() + (_schedule[i]->delay) * 100;
      
      uint32_t prt = (uint32_t) schedule;
      
      if (prt == 0) return -1;
      
      return i;
    }
  }
  return -1;
}
// -------------------------------------------------------
// -------------------------------------------------------
void Scheduler::purge(char *name) 
{
  for (uint8_t i=0; i < MAX_SCHEDULES; i++){
    if (_schedule[i] != 0 && strcmp(_schedule[i]->name, name) == 0) {
        free(_schedule[i]);
        _schedule[i] = 0;
    }
  }
}
// -------------------------------------------------------
// -------------------------------------------------------
int Scheduler::del(uint8_t id) 
{
  for (uint8_t i=0; i < MAX_SCHEDULES; i++){
    if (_schedule[i] != 0 && _schedule[i]->id == id) {
        free(_schedule[i]);
        _schedule[i] = 0;
        return i;
    }
    return -1;
  }
}
// -------------------------------------------------------
// -------------------------------------------------------
void Scheduler::clean() 
{
  for (uint8_t i=0; i < MAX_SCHEDULES; i++){
    if (_schedule[i] != 0 && _schedule[i]->time < millis()) {
        free(_schedule[i]);
        _schedule[i] = 0;
    }
  }
}
// -------------------------------------------------------
// -------------------------------------------------------
void Scheduler::start() {
  _start = 1;
}
// -------------------------------------------------------
// -------------------------------------------------------
void Scheduler::stop() {
  _start = 0;
}
// -------------------------------------------------------
// -------------------------------------------------------
uint32_t Scheduler::getTime() 
{
  return _timeIndex;
}
#endif  