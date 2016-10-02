/*
-------------------------------------------------------------------------------------
  HBus - Home automation/IoT application protocol 
  Pins members
  
  Copyright (C) 2014-15-16 Vincenzo Mennella (see license.txt)
  
  History
    1.0.0 12/08/2014:   First revision of separated code
    1.0.1 01/08/2015:   Added reset function and reviewed code after 1 year
    1.0.2 31/10/2015:   Many bug fixes
-------------------------------------------------------------------------------------
*/
#ifndef HBUS_PIN_H
#define HBUS_PIN_H
#define VERSION_HBUS_PIN_H   "1.0.2"

//-----------------------------------------------------------------------------
// External libraries
//-----------------------------------------------------------------------------
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
#include <Streaming.h> 
#endif

//-----------------------------------------------------------------------------
//HBus libraries
//-----------------------------------------------------------------------------
#include <scheduler.h>
#include <simpleStack.h>

//-------------------------------------------------------
// Global defines
//-------------------------------------------------------
#define MIN_PWM   0
#define MAX_PWM   1023
#define MIN_ANALOG 0
#define MAX_ANALOG 1023

// Pin types
enum hb_pin_type {
	pinInput 	= 0x01,
	pinOutput	= 0x02,
	pinAnalog	= 0x03,
	pinCounter	= 0x04,
	pinPwm		= 0x05
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

class HBusPinController
{
   public:
      virtual void onPinChange(uint8_t index, char *name, uint32_t newvalue) {};
};

//Simple function handlers
//Is used to hook events directly to Arduino code (ino files)
extern "C" {
	// callback function types
    typedef void (*pinChangeCallback)(uint8_t index, char *name, uint32_t value);
} 

//Pin declaration
class Pin 
{
  public:
    char name[NAME_LENGTH+1];
    //char description[MAX_STRING];
    //char location[MAX_STRING];
    uint8_t index;
    uint8_t pin;
    hb_pin_type type;
    hb_pin_subtype subType;
    uint8_t parameters;
    uint8_t* parameter;
    
    //Current & last value
    uint32_t value;
    uint32_t lastValue;
    long lastTime;
    uint8_t lastRead;
    
    Scheduler *scheduler;
    HBusPinController *controller;
    hb_address_t listener;
    uint8_t listenerPort;
    
    Pin();
    Pin(char *pin_name, char *pin_description, char *pin_location, uint8_t pin_pin, uint8_t pin_index, hb_pin_type pin_type, hb_pin_subtype pin_sub_type, uint8_t* pin_parameters = 0, uint8_t pin_parameters_length = 0);
    Pin(SimpleStack *ss);
    Pin(uint8_t* data, uint16_t length);
    void load(SimpleStack *ss);
    void save(SimpleStack *ss);
    void begin();
    void reset();
    //===========================================================
    //  Activation commands
    //===========================================================
    void activate();
    void deactivate();
    uint8_t isActive();
    //===========================================================
    //  Digital write commands
    //===========================================================
    void toggle();
    void timed(uint8_t interval, uint8_t newvalue);
    void delayed(uint8_t delay, uint8_t newvalue);
    void delayedToggle(uint8_t delay);
    void pulsed(uint8_t interval, uint8_t delay, uint8_t newvalue);
    void cycled(uint8_t interval, uint8_t delay, uint8_t cycles);
    //===========================================================
    //  Analog write commands
    //===========================================================
    uint8_t change(uint32_t newvalue);
    void pwm(uint8_t high_width, uint8_t width);
    void delta(int delta);
    void fade(uint16_t value_start, uint16_t value_end, uint8_t steps, uint8_t delay);
    //===========================================================
    //  Common commands
    //===========================================================
    uint32_t read();
    void write(uint32_t newvalue);
    char* toString();
};

//-----------------------------------------------------------------------------
// Declaration of schedule for pin type
//-----------------------------------------------------------------------------
class PinSchedule: public ISchedule {
  private:
    Pin* _pin;
    uint32_t _value;
        
  public:
    PinSchedule();
    PinSchedule(Pin* pin, int32_t next_value, uint32_t pin_delay, uint8_t pin_reschedule);
    void execute();
};

//-----------------------------------------------------------------------------
//Pin implementation
//-----------------------------------------------------------------------------
Pin::Pin() 
{
  strcpy_P(name, PSTR(""));
  //strcpy_P(description, PSTR(""));
  //strcpy_P(location, PSTR(""));
  pin = 0;
  index = 0;
  type = pinOutput;
  subType = pinNone;
  parameters = 0;
  value = 0;
  lastValue = 0;
  lastTime = 0;
  lastRead = 0;
}
//-----------------------------------------------------------
// Full parameters constructor
//-----------------------------------------------------------
Pin::Pin(char *pin_name, char *pin_description, char *pin_location, uint8_t pin_pin, uint8_t pin_index, hb_pin_type pin_type, hb_pin_subtype pin_sub_type, uint8_t* pin_parameters, uint8_t pin_parameters_length) 
{
  strcpy(name, pin_name); 
  //strcpy(description, pin_description); 
  //strcpy(location, pin_location); 
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
  char source[MAX_STRING];
  uint16_t p = 0;

  ss->popName(name);
  ss->popString(source); //description
  ss->popString(source); //location
  index = ss->popByte();
  ss->popString(source);
  pin = atoi(source);
  type = (hb_pin_type) ss->popByte();
  subType	= (hb_pin_subtype) ss->popByte();
  ss->popArray(parameter, &p);
  parameters = p;
}
//-----------------------------------------------------------
// save pin configuration
// Parameters:  memory stack
// Returns: none
//-----------------------------------------------------------
void Pin::save(SimpleStack *ss) 
{
  char source[MAX_STRING];
  //Push data
  ss->pushName(name);
  ss->pushString(""); //Description
  ss->pushString(""); //Location
  ss->pushByte(index);
  itoa (pin,source,10);
  ss->pushString(source);
  ss->pushByte((uint8_t) type);
  ss->pushByte((uint8_t) subType);
  ss->pushArray(parameter, parameters);

 #if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  // Serial << "=> pin->save " << endl;
  // Serial << "\t name " << name << endl;
  // Serial << "\t index " << index << endl;
  // Serial << "\t pin " << pin << endl;
  // Serial << "\t type " << type << endl;
  // Serial << "\t subType " << subType << endl;
  // Serial << "\t parameters " << parameters << " (" << (uint32_t) parameter << ")" << endl;
#endif
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

  //lastValue = read();
  
  switch(subType)
  {
    case outputLow:
      write(LOW);
      break;
    case outputHigh:
      write(HIGH);
      break;
    case outputToggle:
      write(!lastValue);
      break;
    case outputTimedHigh:
      write(HIGH);
      scheduler->add(new PinSchedule(this, LOW, (parameter[0] << 8 | parameter[1]), false));
      break;
    case outputTimedLow:
      write(LOW);
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
      write(HIGH);
      scheduler->add(new PinSchedule(this, LOW, (parameter[0] << 8 | parameter[1]), false));
      scheduler->add(new PinSchedule(this, HIGH,  (parameter[2] << 8 | parameter[3]), false));
      break;
    case outputPulseHigh:
      write(LOW);
      scheduler->add(new PinSchedule(this, HIGH, (parameter[0] << 8 | parameter[1]), false));
      scheduler->add(new PinSchedule(this, LOW,  (parameter[2] << 8 | parameter[3]), false));
      break;
    //Numeric types
    case outputSetValue:
      write(parameter[0] << 8 | parameter[1]);
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  //Serial << "=> outputSetValue " << name << " pin: " << pin << " = " << lastValue << endl;
#endif
      // break;
    case outputAddValue:
      write(lastValue + (parameter[0] << 8 | parameter[1]));
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  //Serial << "=> outputAddValue " << name << " pin: " << pin << " = " << lastValue << endl;
#endif
      break;
    default:
        //NTD
      break;
  }
 }
//-----------------------------------------------------------
// Deactivate pin according to its type
// Parameters:  none
// Returns: none
//-----------------------------------------------------------
void Pin::deactivate()
{
  scheduler->purge(name);

  //lastValue = read();

  switch(subType)
  {
    case outputLow:
    case outputPulseLow:
    case outputDelayLow:
    case outputTimedLow:
      write(HIGH);
      break;
    case outputHigh:
    case outputToggle:
    case outputPulseHigh:
    case outputDelayHigh:
    case outputDelayToggle:
    case outputTimedHigh:
      write(LOW);
      break;
    //Numeric types
    case outputSetValue:
    case outputAddValue:
      write(0);
      break;
    default:
        //NTD
      break;
  }
// #if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  // Serial << "=> deactivate " << name << " pin: " << pin << " = " << digitalRead(pin) << endl;
// #endif
}
//-----------------------------------------------------------
// Check if pin is active according to its type and subtype
// Parameters:  none
// Returns: true if pin is active
//-----------------------------------------------------------
uint8_t Pin::isActive()
{
  //Update last value
  lastValue = value;
  
  //update current value
  value = read();
  
  //Check conditions
  switch(subType)
  {
    //Boolean conditions
    case pinNone:
      return false;
    case inputLow:
      return value == LOW;
    case inputHigh:
      return value == HIGH;
    case inputHighLow:
      return value == LOW && lastValue == HIGH;
    case inputLowHigh:
      return value == HIGH && lastValue == LOW;
    //Numeric conditions
    case inputBelow:
      return (value < (parameter[0] << 8 | parameter[1]));
    case inputBeyond:
      return (value > (parameter[0] << 8 | parameter[1]));
    case inputBetween:
      return ((parameter[0] << 8 | parameter[1]) <= value && value <= (parameter[2] << 8 | parameter[3]));
    case inputOutside:
      return (value < (parameter[0] << 8 | parameter[1]) || (parameter[2] << 8 | parameter[3]) < value);
    //General conditions
    case inputChanged:
      return lastValue != value;
    case inputEqualTo:
      return value == (parameter[0] << 8 | parameter[1]);
    //Output pin
    case outputLow:
    case outputPulseLow:
        return type == value == 0;
    case outputHigh:
    case outputPulseHigh:
    case outputDelayHigh:
    case outputToggle:
    case outputDelayToggle:
        return type == (value != 0) ;
    case outputSetValue:
        return (value == (parameter[0] << 8 | parameter[1]));
    case outputAddValue:
        return (value != 0);
    default:
      return false;
  }
}
/*
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
*/

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

  // lastValue = read();
  write (!lastValue);
}
//-----------------------------------------------------------
// Set digital output for specific interval
// Parameters: 
//  width: time interval in seconds
//  value: pin value to be set for interval
// Returns:	
//  none
//-----------------------------------------------------------
void Pin::timed(uint8_t interval, uint8_t newvalue)
{
  scheduler->purge(name);
  // lastValue = read();
  write(newvalue);
  scheduler->add(new PinSchedule(this, !newvalue, (uint16_t) (interval*100), false));
}
//-----------------------------------------------------------
// Set digital output after delay
// Parameters: 
//  delay: time interval in seconds before set value
//  value: pin value to be set
// Returns:	
//  none
//-----------------------------------------------------------
void Pin::delayed(uint8_t delay, uint8_t newvalue)
{
  scheduler->purge(name);
  // lastValue = read();
  scheduler->add(new PinSchedule(this, newvalue, (uint16_t) (delay*100), false));
 #if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  Serial << "=> pin->delayed " << endl;
  Serial << "\t value " << newvalue << endl;
  Serial << "\t delay " << delay << endl;
#endif
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
  // lastValue = read();
  scheduler->add(new PinSchedule(this, !lastValue, (uint16_t) (delay*100), false));
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
void Pin::pulsed(uint8_t interval, uint8_t delay, uint8_t newvalue)
{
  scheduler->purge(name);
  // lastValue = read();
  write(!newvalue);
  scheduler->add(new PinSchedule(this, newvalue, (uint16_t) (interval*100), false));
  scheduler->add(new PinSchedule(this, !newvalue, (uint16_t) ((interval+delay)*100), false));
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
  // lastValue = read();

  for(uint8_t i=0;i< cycles;i++) {
    //Start of HIGH pulse
    scheduler->add(new PinSchedule(this, HIGH, time + (uint16_t) (interval*100), false));
    //End of HIGH pulse
    scheduler->add(new PinSchedule(this, LOW, time + (uint16_t) ((interval+delay)*100), false));
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
uint8_t Pin::change(uint32_t newvalue) 
{
  scheduler->purge(name);
  // lastValue = read();
  write(newvalue);
  
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
  // lastValue = read();
  
  write(((uint32_t) high_width * (MAX_PWM - MIN_PWM) / (uint32_t) width ) + MIN_PWM);
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
  // lastValue = read();
  write(lastValue + delta);
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
  // lastValue = read();
  
  //Calculate increment for each step
  uint32_t time = 0;
  uint32_t newvalue = value_start;
  int delta = ((int) value_end - (int) value_start) / (steps - 1);

  //Set actual value
  write(newvalue);

  //Loop for n-1 steps
  for(uint8_t i = 1; i < steps; i++) {
      newvalue += delta;
      if (newvalue < MIN_PWM) newvalue = MIN_PWM;
      if (newvalue > MAX_PWM) newvalue = MAX_PWM;
      time += delay * 100;
      scheduler->add(new PinSchedule(this, newvalue, time, false));
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
  uint8_t new_read;
  uint32_t new_value;
  
  switch(type)
  {
    //Check digital input state
    case pinInput:
      lastTime = millis();
      new_value = digitalRead(pin);
      // //new read
      // new_read = digitalRead(pin);
      
      // if (new_read != lastRead) {
          // lastRead = (uint8_t) new_read;
          // //start anti de-bounce delay
          // lastTime = millis();
          // return lastValue;
      // }
      // else {
        // if ((millis() - lastTime) > DEF_DEBOUNCE) {
          // //store new status
          // new_value = new_read ? 1 : 0;
        // }
      // }
      break;
          
    //Check analog/counter input state
    case pinOutput:
      lastTime = millis();
      new_value = digitalRead(pin);
      break;

      //Check analog/counter input state
    case pinAnalog:
      lastTime = millis();
      new_value = analogRead(pin);
      break;
          
      //Check analog/counter input state
    default:
      //NTD
      lastTime = millis();
      break;
  }

  //Trigger event on pin change
  if (controller && new_value != lastValue) 
    controller->onPinChange(index, name, new_value);
  
  return new_value;
}

void Pin::write(uint32_t newvalue)
{
  value = newvalue;
  
  switch(subType)
  {
    //Digital types
    case outputLow:
    case outputPulseLow:
    case outputDelayLow:
    case outputTimedLow:
    case outputHigh:
    case outputToggle:
    case outputPulseHigh:
    case outputDelayHigh:
    case outputDelayToggle:
    case outputTimedHigh:
      digitalWrite(pin, (uint8_t)(value & 0xff));
      break;
    //Numeric types
    case outputSetValue:
    case outputAddValue:
      if (value < MIN_PWM)  
        value = MIN_PWM;
      if (value > MAX_PWM)
        value = MAX_PWM;
      analogWrite(pin,(uint16_t) value);
      break;
    default:
        //NTD
      break;
  }
  //Trigger event on pin change
  if (controller && value != lastValue)
    controller->onPinChange(index, name, value);
    
  //reset last value
  lastValue = value;
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
 #if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  Serial << "PinSchedule::execute" << endl;
  Serial << "\t pin "   << _pin->name << endl;
  Serial << "\t value " << _value << endl;
#endif

  if (_pin == 0) return;
  
  _pin->lastValue = _pin->read();
  _pin->write(_value);
}


#endif