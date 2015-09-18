/*
  =======================================================
  HNode - Home automation/IoT node for HBus project
  
  Pin informations
  
  Copyright (C) 2014-15 Vincenzo Mennella (see license.txt)
  
  History
    1.0.0 12/08/2014:   First revision of separated code
    1.0.1 01/08/2015:   Added reset function and reviewed code after 1 year
  =======================================================
*/
#ifndef HNODE_PIN_H
#define HNODE_PIN_H
#define VERSION_HNODE_PIN_H   "1.0.1"

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

//-----------------------------------------------------------
//HBus libraries
//-----------------------------------------------------------
#include <scheduler.h>
#include <simpleStack.h>

//-----------------------------------------------------------
// Global defines
//-----------------------------------------------------------
#define MIN_PWM   0
#define MAX_PWM   1023
#define MIN_ANALOG 0
#define MAX_ANALOG 1023

// Pin types
enum hb_pin_type {
	pinInput 	= 0x00,
	pinOutput	= 0x01,
	pinAnalog	= 0x02,
	pinCounter	= 0x03,
	pinPwm		= 0x04
};

// Input sub types
enum hb_pin_subtype {
	//Boolean conditions
	pinNone,
	inputLow,
	inputHigh,
	inputHighLow,
	inputLowHigh,
	//Numeric conditions
	inputBelow,
	inputBeyond,
	inputBetween,
	inputOutside,
	//General conditions
  inputChanged,
	inputEqualTo,
	outputLow,
	outputHigh,
	outputToggle,
	outputTimedHigh,
	outputTimedLow,
	outputDelayHigh,
	outputDelayLow,
	outputPulseHigh,
	outputPulseLow,
	outputDelayToggle,
	//Numeric types
	outputSetValue,
	outputAddValue
};
//-----------------------------------------------------------
//-----------------------------------------------------------
class HBusPinController
{
   public:
      virtual void onPinChange(char *name, uint32_t value) {};
};

//-----------------------------------------------------------
//Pin class implementation
//-----------------------------------------------------------
class Pin 
{
  public:
    char name[NAME_LENGTH+1];
    uint8_t index;
    uint8_t pin;
    hb_pin_type type;
    hb_pin_subtype subType;
    uint8_t parameters;
    uint8_t* parameter;
    uint32_t lastValue;
    long lastTime;
    uint8_t lastRead;
    
    Scheduler *scheduler;
    HBusPinController *controller;
    hb_address_t listener;
    uint8_t listenerPort;
    
    Pin();
    Pin(char *pin_name, uint8_t pin_pin, uint8_t pin_index, hb_pin_type pin_type, hb_pin_subtype pin_sub_type, uint8_t* pin_parameters = 0, uint8_t pin_parameters_length = 0);
    Pin(SimpleStack *ss);
    Pin(uint8_t* data, uint16_t length);
    void load(SimpleStack *ss);
    void save(SimpleStack *ss);
    void begin();
    //===========================================================
    //  Activation commands
    //===========================================================
    void activate();
    void deactivate();
    uint8_t isActive();
    void scheduleActivate(uint32_t time);
    uint32_t updatePin();
    void reset();
    //===========================================================
    //  Digital write commands
    //===========================================================
    void toggle();
    void timed(uint8_t interval, uint8_t value);
    void delayed(uint8_t delay, uint8_t value);
    void delayedToggle(uint8_t delay);
    void pulsed(uint8_t interval, uint8_t delay, uint8_t value);
    void cycled(uint8_t interval, uint8_t delay, uint8_t cycles);
    //===========================================================
    //  Analog write commands
    //===========================================================
    uint8_t change(uint16_t value);
    void pwm(uint8_t high_width, uint8_t width);
    void delta(int delta);
    void fade(uint16_t value_start, uint16_t value_end, uint8_t steps, uint8_t delay);
    //===========================================================
    //  Digital read commands
    //===========================================================
    uint32_t read();
    char* toString();
};

//-----------------------------------------------------------
// Schedule implementation for pins
//-----------------------------------------------------------
class PinSchedule: public ISchedule {
  private:
    Pin* _pin;
    uint32_t _value;
        
  public:
    PinSchedule();
    PinSchedule(Pin* pin, int32_t next_value, uint32_t pin_delay, uint8_t pin_reschedule);
    void execute();
};

//-----------------------------------------------------------
//Pin implementation
//-----------------------------------------------------------
Pin::Pin() 
{
    strcpy(name, ""); 
    pin = 0;
    index = 0;
    type = pinOutput;
    subType = pinNone;
    parameters = 0;
    //minValue = 0;
    //maxValue = 0;
    lastValue = 0;
    lastTime = 0;
}
//-----------------------------------------------------------
// Full parameters constructor
//-----------------------------------------------------------
Pin::Pin(char *pin_name, uint8_t pin_pin, uint8_t pin_index, hb_pin_type pin_type, hb_pin_subtype pin_sub_type, uint8_t* pin_parameters, uint8_t pin_parameters_length) 
{
  strcpy(name, pin_name); 
  pin = pin_pin;
  index = pin_index;
  type = pin_type;
  subType = pin_sub_type;
  parameter = pin_parameters;
  parameters = pin_parameters_length;
}
//-----------------------------------------------------------
// Serialized configuration constructor
//-----------------------------------------------------------
Pin::Pin(SimpleStack *ss) 
{
  load(ss);
}
//-----------------------------------------------------------
// Serialized configuration constructor
//-----------------------------------------------------------
Pin::Pin(uint8_t* data, uint16_t length) 
{
  SimpleStack ss;
  ss.begin(data, length, (uint8_t) NAME_LENGTH);
  load(&ss);
}

//-----------------------------------------------------------
// Load configuration
// Parameters:  memory stack
// Returns: none
//-----------------------------------------------------------
void Pin::load(SimpleStack *ss) 
{
  ss->popFixedString(name, NAME_LENGTH);
  index = ss->popByte();
  pin 	= ss->popByte();
  type	= (hb_pin_type) ss->popByte();
  subType	= (hb_pin_subtype) ss->popByte();
  uint16_t p = 0;
  ss->popArray(parameter, &p);
  parameters = p;
  // minValue    = ss->popInteger();
  // maxValue    = ss->popInteger();
}
//-----------------------------------------------------------
// save pin configuration
// Parameters:  memory stack
// Returns: none
//-----------------------------------------------------------
void Pin::save(SimpleStack *ss) 
{
  //Push data
  ss->pushFixedString(name, NAME_LENGTH);
  ss->pushByte(index);
  ss->pushByte(pin);
  ss->pushByte((uint8_t) type);
  ss->pushByte((uint8_t) subType);
  ss->pushArray(parameter, parameters);
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  // DEBUG_PORT << "=> pin->save " << endl;
  // DEBUG_PORT << "\t name " << name << endl;
  // DEBUG_PORT << "\t index " << index << endl;
  // DEBUG_PORT << "\t pin " << pin << endl;
  // DEBUG_PORT << "\t type " << type << endl;
  // DEBUG_PORT << "\t subType " << subType << endl;
  // DEBUG_PORT << "\t parameters " << parameters << " (" << (uint32_t) parameter << ")" << endl;
#endif
  // ss->pushInteger(minValue);
  // ss->pushInteger(maxValue);
}
void Pin::begin() 
{
  switch(type) {
    case pinInput:
    case pinCounter:
    case pinAnalog:
        pinMode(pin, INPUT);   
        break;
    case pinOutput:
    case pinPwm:
        pinMode(pin, OUTPUT);   
        break;
  }
  lastValue = 0;
  lastTime = 0;
  deactivate();
}
//===========================================================
//  Activation commands
//===========================================================
//-----------------------------------------------------------
// Activate pin according to its type
// Parameters:  none
// Returns: none
//-----------------------------------------------------------
void Pin::activate() 
{
  scheduler->purge(name);

  switch(subType)
  {
    case outputLow:
      digitalWrite(pin, LOW);
      break;
    case outputHigh:
      digitalWrite(pin, HIGH);
      break;
    case outputToggle:
      digitalWrite(pin, !digitalRead(pin));
      break;
    case outputTimedHigh:
      digitalWrite(pin, HIGH);
      scheduler->add(new PinSchedule(this, LOW, (parameter[0] << 8 | parameter[1]), false));
      break;
    case outputTimedLow:
      digitalWrite(pin, LOW);
      scheduler->add(new PinSchedule(this, HIGH, (parameter[0] << 8 | parameter[1]), false));
      break;
    case outputDelayLow:
      scheduler->add(new PinSchedule(this, LOW, (parameter[0] << 8 | parameter[1]), false));
      break;
    case outputDelayHigh:
      scheduler->add(new PinSchedule(this, HIGH, (parameter[0] << 8 | parameter[1]), false));
      break;
    case outputDelayToggle:
      scheduler->add(new PinSchedule(this, !digitalRead(pin), (parameter[0] << 8 | parameter[1]), false));
      break;
    case outputPulseLow:
      digitalWrite(pin, HIGH);
      scheduler->add(new PinSchedule(this, LOW, (parameter[0] << 8 | parameter[1]), false));
      scheduler->add(new PinSchedule(this, HIGH,  (parameter[2] << 8 | parameter[3]), false));
      break;
    case outputPulseHigh:
      digitalWrite(pin, LOW);
      scheduler->add(new PinSchedule(this, HIGH, (parameter[0] << 8 | parameter[1]), false));
      scheduler->add(new PinSchedule(this, LOW,  (parameter[2] << 8 | parameter[3]), false));
      break;
    //Numeric types
    case outputSetValue:
      lastValue = (parameter[0] << 8 | parameter[1]);
      analogWrite(pin, lastValue);
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  //DEBUG_PORT << "=> outputSetValue " << name << " pin: " << pin << " = " << lastValue << endl;
#endif
      // break;
    case outputAddValue:
      lastValue += (parameter[0] << 8 | parameter[1]);
      analogWrite(pin, lastValue);
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  //DEBUG_PORT << "=> outputAddValue " << name << " pin: " << pin << " = " << lastValue << endl;
#endif
      break;
    default:
        //NTD
      break;
  }
  if (type == pinOutput)
    lastValue = digitalRead(pin);
  }
//-----------------------------------------------------------
// Deactivate pin according to its type
// Parameters:  none
// Returns: none
//-----------------------------------------------------------
void Pin::deactivate()
{
  scheduler->purge(name);

  switch(subType)
  {
    case outputLow:
    case outputPulseLow:
    case outputDelayLow:
    case outputTimedLow:
      digitalWrite(pin, HIGH);
      break;
    case outputHigh:
    case outputToggle:
    case outputPulseHigh:
    case outputDelayHigh:
    case outputDelayToggle:
    case outputTimedHigh:
      digitalWrite(pin, LOW);
      break;
    //Numeric types
    case outputSetValue:
    case outputAddValue:
      analogWrite(pin, 0);
      break;
    default:
        //NTD
      break;
  }
  lastValue = digitalRead(pin);
  
// #if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  // DEBUG_PORT << "=> deactivate " << name << " pin: " << pin << " = " << digitalRead(pin) << endl;
// #endif
}
//-----------------------------------------------------------
// Check if pin is active according to its type and subtype
// Parameters:  none
// Returns: true if pin is active
//-----------------------------------------------------------
uint8_t Pin::isActive()
{
  uint32_t old_value = updatePin();
  
  //Check conditions
  switch(subType)
  {
    //Boolean conditions
    case pinNone:
      return false;
    case inputLow:
      return lastValue == LOW;
    case inputHigh:
      return lastValue == HIGH;
    case inputHighLow:
      return lastValue == LOW && old_value == HIGH;
    case inputLowHigh:
      return lastValue == HIGH && old_value == LOW;
    //Numeric conditions
    case inputBelow:
      return (lastValue < (parameter[0] << 8 | parameter[1]));
    case inputBeyond:
      return (lastValue > (parameter[0] << 8 | parameter[1]));
    case inputBetween:
      return ((parameter[0] << 8 | parameter[1]) <= lastValue && lastValue <= (parameter[2] << 8 | parameter[3]));
    case inputOutside:
      return (lastValue < (parameter[0] << 8 | parameter[1]) || (parameter[2] << 8 | parameter[3]) < lastValue);
    //General conditions
    case inputChanged:
      return lastValue != old_value;
    case inputEqualTo:
      return lastValue == (parameter[0] << 8 | parameter[1]);
    //Output pin
    case outputLow:
    case outputPulseLow:
        return type == lastValue == 0;
    case outputHigh:
    case outputPulseHigh:
    case outputDelayHigh:
    case outputToggle:
    case outputDelayToggle:
        return type == (lastValue != 0) ;
    case outputSetValue:
        return (lastValue == (parameter[0] << 8 | parameter[1]));
    case outputAddValue:
        return (lastValue != 0);
    default:
      return false;
  }
}

//-----------------------------------------------------------
// Schedule pin activation with scheduler
//-----------------------------------------------------------
void Pin::scheduleActivate(uint32_t time)
{
  //Delete schedules for the pin
  scheduler->purge(name);
   
  switch(subType)
  {
    case outputLow:
      scheduler->add(new PinSchedule(this, LOW, time, false));
      break;
    case outputHigh:
      scheduler->add(new PinSchedule(this, HIGH, time, false));
      break;
    case outputToggle:
      scheduler->add(new PinSchedule(this, !digitalRead(pin), time, false));
      break;
    case outputPulseLow:
      scheduler->add(new PinSchedule(this, LOW,  time, false));
      scheduler->add(new PinSchedule(this, HIGH, time + (parameter[0] << 8 | parameter[1]), false));
      scheduler->add(new PinSchedule(this, LOW,  time + (parameter[0] << 8 | parameter[1]) + (parameter[2] << 8 | parameter[3]), false));
      break;
    case outputPulseHigh:
      scheduler->add(new PinSchedule(this, HIGH, time, false));
      scheduler->add(new PinSchedule(this, LOW,  time + (parameter[0] << 8 | parameter[1]), false));
      scheduler->add(new PinSchedule(this, HIGH, time + (parameter[0] << 8 | parameter[1]) + (parameter[2] << 8 | parameter[3]), false));
      break;
    case outputDelayLow:
         scheduler->add(new PinSchedule(this, LOW,  time + (parameter[0] << 8 | parameter[1]), false));
      break;
    case outputDelayHigh:
         scheduler->add(new PinSchedule(this, HIGH, time + (parameter[0] << 8 | parameter[1]), false));
      break;
    case outputTimedLow:
      scheduler->add(new PinSchedule(this, LOW, time, false));
      scheduler->add(new PinSchedule(this, HIGH, time + (parameter[0] << 8 | parameter[1]), false));
      break;
    case outputTimedHigh:
      scheduler->add(new PinSchedule(this, HIGH, time, false));
      scheduler->add(new PinSchedule(this, (int32_t) LOW, (uint32_t) (time + (parameter[0] << 8 | parameter[1])), false));
      break;
    case outputDelayToggle:
      scheduler->add(new PinSchedule(this, !digitalRead(pin), time + (parameter[0] << 8 | parameter[1]), false));
      break;
    //Numeric types
    case outputSetValue:
     scheduler->add(new PinSchedule(this, (parameter[0] << 8 | parameter[1]), time, false));
     break;
    case outputAddValue:
      scheduler->add(new PinSchedule(this, lastValue + (parameter[0] << 8 | parameter[1]), time, false));
      break;
  }
}

//-----------------------------------------------------------
// Update pin status
// Parameters: none
// Returns: last pin value
//-----------------------------------------------------------
uint32_t Pin::updatePin() 
{
  uint32_t new_read;
  uint32_t old_value = lastValue;
    
  switch(type)
  {
    //Check digital input state
    case pinInput:
      //new read
      new_read = digitalRead(pin);
      
      if (new_read != lastRead) {
          lastRead = new_read;
          //start anti debounce delay
          lastTime = millis();
          return lastValue;
      }
      else {
        if ((millis() - lastTime) > DEF_DEBOUNCE) {
          //store new status
          lastValue = new_read;
        }
      }
      break;
          
    //Check analog/counter input state
    case pinOutput:
      lastTime = millis();
      lastValue = digitalRead(pin);
      break;

      //Check analog/counter input state
    case pinAnalog:
      lastTime = millis();
      lastValue = analogRead(pin);
      break;
          
      //Check analog/counter input state
    default:
      //NTD
      lastTime = millis();
      new_read = old_value;
      break;
  }
  
  if (controller && old_value != lastValue)
    controller->onPinChange(name, lastValue);
  
  return old_value;
}

//-----------------------------------------------------------
// Reset pin state & listener
//-----------------------------------------------------------
void Pin::reset() 
{
  deactivate();
  listener = 0;
}
//===========================================================
//  Digital write commands
//===========================================================
//-----------------------------------------------------------
// Toggle digital output value
// Parameters: none
// Returns:
//  none
//-----------------------------------------------------------
void Pin::toggle()
{
  scheduler->purge(name);
  digitalWrite(pin, !digitalRead(pin));
  lastValue = digitalRead(pin);
}
//-----------------------------------------------------------
// Set digital output for specific interval
// Parameters: 
//  width: time interval in seconds
//  value: pin value to be set for interval
// Returns:	
//  none
//-----------------------------------------------------------
void Pin::timed(uint8_t interval, uint8_t value)
{
  scheduler->purge(name);
  digitalWrite(pin, value);
  lastValue = digitalRead(pin);
  // scheduler->add(new PinSchedule(this, !value, (uint16_t) (interval*100), false));
}
//-----------------------------------------------------------
// Set digital output after delay
// Parameters: 
//  delay: time interval in seconds before set value
//  value: pin value to be set
// Returns:	
//  none
//-----------------------------------------------------------
void Pin::delayed(uint8_t delay, uint8_t value)
{
  scheduler->purge(name);
  // scheduler->add(new PinSchedule(this, value, (uint16_t) (delay*100), false));
}
//-----------------------------------------------------------
// Toggle digital output after delay
// Parameters: 
//  delay: time interval in seconds before invert value
// Returns:	
//  none
//-----------------------------------------------------------
void Pin::delayedToggle(uint8_t delay)
{
  scheduler->purge(name);
  // scheduler->add(new PinSchedule(this, !digitalRead(pin), (uint16_t) (delay*100), false));
}
//-----------------------------------------------------------
// Set digital output after delay for specific interval
// Parameters: 
//  interval: time interval in seconds
//  delay: time delay in seconds before set value
//  value: pin value to be set
// Returns:	
//  none
//-----------------------------------------------------------
void Pin::pulsed(uint8_t interval, uint8_t delay, uint8_t value)
{
  scheduler->purge(name);
  digitalWrite(pin, !value);
  lastValue = digitalRead(pin);
  // scheduler->add(new PinSchedule(this, value, (uint16_t) (interval*100), false));
  // scheduler->add(new PinSchedule(this, !value, (uint16_t) ((interval+delay)*100), false));
}
//-----------------------------------------------------------
// Set digital output after delay for specific interval
// for n cycles
// Parameters: 
//  interval: time interval in seconds
//  delay: time delay in seconds before set value
//  cycles: number of cycles to execute
// Returns:	
//  none
//-----------------------------------------------------------
void Pin::cycled(uint8_t interval, uint8_t delay, uint8_t cycles)
{
  uint32_t time = scheduler->getTime();

  scheduler->purge(name);
  
  for(uint8_t i=0;i< cycles;i++) {
      //Start of HIGH pulse
//         scheduler->add(new PinSchedule(this, HIGH, time + (uint16_t) (interval*100), false));
      //End of HIGH pulse
//          scheduler->add(new PinSchedule(this, LOW, time + (uint16_t) ((interval+delay)*100), false));
      //Next pulse start
      time += (uint16_t) ((interval+delay)*100);
  }
}
//===========================================================
//  Analog write commands
//===========================================================
//-----------------------------------------------------------
// Set pin to specific value
//-----------------------------------------------------------
uint8_t Pin::change(uint16_t value) 
{
  scheduler->purge(name);
  
  if (value < MIN_PWM)  
    value = MIN_PWM;
  if (value > MAX_PWM)
    value = MAX_PWM;
  
  switch(type) {
    case pinOutput:
      digitalWrite(pin, (uint8_t)(value & 0xff));
      break;
    case pinPwm:
      analogWrite(pin, value);
      break;
    default:
      //NTD
      break;
      
  }
  lastValue = value;
  return true;
}
//-----------------------------------------------------------
// Set pwm output with HIGH pulse and total width
// Parameters: 
//  high_width: high pulse interval
//  width: total width pulse
// Returns:	
//  none
//-----------------------------------------------------------
void Pin::pwm(uint8_t high_width, uint8_t width)
{
  scheduler->purge(name);
  uint16_t value = ((uint16_t) high_width * (MAX_PWM - MIN_PWM) / (uint16_t) width ) + MIN_PWM;
  analogWrite(pin, value);
  lastValue = value;
}
//-----------------------------------------------------------
// Set analog/pwm output using signed 16 bit value added
// to pin value
// Parameters: 
// delta: value to be added to pin value
// Returns:	
// none
//-----------------------------------------------------------
void Pin::delta(int delta)
{

  scheduler->purge(name);

  uint16_t value = lastValue + delta;
  
  if (value < MIN_PWM)  
    value = MIN_PWM;
  if (value > MAX_PWM)
    value = MAX_PWM;
  
  analogWrite(pin, value);
  lastValue = value;
}
//-----------------------------------------------------------
// Fade analog pin from start value to end value
// in n steps with delay
// Parameters:
//  start value
//  end value
//  steps
//  delay in 1/10 seconds
// Returns:	none
//-----------------------------------------------------------
void Pin::fade(uint16_t value_start, uint16_t value_end, uint8_t steps, uint8_t delay)
{
  scheduler->purge(name);
        
  //Calculate increment for each step
  int delta = ((int) value_end - (int) value_start) / (steps - 1);
  uint32_t value = (int) value_start + delta;
  uint32_t time = delay * 100;
  
  //Set actual value
  analogWrite(pin, value_start);
  lastValue = value_start;

  //Schedule next pin changes
  scheduler->purge(name);
  
  //Loop for n-1 steps
  for(uint8_t i = 1; i < steps; i++) {
//          scheduler->add(new PinSchedule(this, value, time, false));
      value += delta;
      if (value < MIN_PWM) value = MIN_PWM;
      if (value > MAX_PWM) value = MAX_PWM;
      time += delay * 100;
  }
}
//===========================================================
//  Digital read commands
//===========================================================

//-----------------------------------------------------------
// Read pin value according to its type
// Serialize sensor information into tx stack
// Parameters: 
//  pin index, pin type
// Returns:	
//  none
//-----------------------------------------------------------
uint32_t Pin::read()
{
  switch(type) 
  {
    case pinInput:
    case pinOutput:
      return digitalRead(pin);
      
    case pinAnalog:
      return analogRead(pin);
      
    default:
      return lastValue;
  }
  return 0;
}
//-----------------------------------------------------------
// Default conversion to string
//-----------------------------------------------------------
char* Pin::toString() 
{
    return name;
}

PinSchedule::PinSchedule() 
{
  _pin = 0;
  name = 0;
  _value = 0;
  delay = 0;
}
PinSchedule::PinSchedule(Pin* pin, int32_t next_value, uint32_t pin_delay, uint8_t pin_reschedule) 
{
  _pin = pin;
  _value = next_value;
  id = 0;
  name = pin->name; 
  time = 0;
  delay = pin_delay;
  reschedule = pin_reschedule;
}
void PinSchedule::execute() 
{
  if (_pin == 0) return;
  
  switch(_pin->type) {
    case pinOutput:
      digitalWrite(_pin->pin, _value);
      break;
    case pinPwm:
      analogWrite(_pin->pin, _value);
      break;
    default:
      //NTD
      break;
      _pin->lastValue = _value;
      _pin->lastTime = millis();
  }
}


#endif