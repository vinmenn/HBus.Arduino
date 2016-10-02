/*
-------------------------------------------------------------------------------------
   HBus - Home automation/IoT application protocol 
   Pins classes
   Copyright (C) 2014-15-16 Vincenzo Mennella (see license.txt)
   History
    1.0.0 24/12/2014:   First release with current architecture
    1.0.1 25/07/2015:   New wire concept: sends HBus commands when triggered  
    1.0.2 21/10/2015:   Bug fixes and code review
-------------------------------------------------------------------------------------
*/
#ifndef HBUS_WIRE_H
#define HBUS_WIRE_H

//-----------------------------------------------------------------------------
// History:
//-----------------------------------------------------------------------------
#define VERSION_HBUS_WIRE_H   "1.0.2"

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
#include <devices/device.h>

//-----------------------------------------------------------------------------
//Wire handler class
//-----------------------------------------------------------------------------
class Wire
{
  public:
    uint8_t index;
    char input[NAME_LENGTH+1];
    uint8_t command;
    uint8_t data[MAX_BUFFER];
    uint16_t length;
    hb_address_t address;
    uint8_t use_input_value;
    //hb_wire_trigger_t trigger;

    Wire() 
    {
        strcpy_P(input,PSTR("")); 
        command = 0;
        address = NO_ADDRESS;
    }
    //Basic constructor for local wire
    Wire(char* pinput, char* poutput) 
    {
      strcpy(input, pinput); 
      command = CMD_ACTIVATE;
      memcpy(data, poutput, NAME_LENGTH);
      address = 0;
      length = NAME_LENGTH;
      use_input_value = false;
    }
    //Full constructor
    // pinput: input pin name
    // pcommand: HBus command (for remote activations)
    // paddress: HBus address (for remote activations)
    // pdata: HBus command data  (for remote activations)
    // plength: command data length  (for remote activations)
    // puseinputvalue: add input value to command data  (for remote activations)
    Wire(char* pinput, uint8_t pcommand, hb_address_t paddress, char* pdata, uint8_t plength, uint8_t puseinputvalue) 
    {
      strcpy(input, pinput); 
      command = pcommand;
      address = paddress;
      if (pdata!=0) {
        memcpy(data, pdata, plength);
      }
      length = plength;
      use_input_value = puseinputvalue;
       
    }
     //-----------------------------------------------------------
    // Serialized configuration constructor
    //-----------------------------------------------------------
    Wire(uint8_t* data, uint16_t length) 
    {
      SimpleStack ss;
      ss.begin(data, length, (uint8_t) NAME_LENGTH);
      load(&ss);
    }
   Wire(SimpleStack *ss) 
    {
        load(ss);
    }
    //-----------------------------------------------------------
    // Load wire configuration
    // Parameters:  memory stack
    // Returns: none
    //-----------------------------------------------------------
    void load(SimpleStack *ss) 
    {
      index = ss->popByte();
      ss->popName(input);
      command = ss->popByte();
#if DEFAULT_WIDTH == 1
      address = ss->popByte();
#endif
#if DEFAULT_WIDTH == 2
      address = ss->popInteger();
#endif
#if DEFAULT_WIDTH == 4
      address = ss->popLong();
#endif
      use_input_value = ss->popByte();

      ss->popArray(data, &length);
      
    }
        
    //-----------------------------------------------------------
    // save wire configuration
    // Parameters:  memory stack
    // Returns: none
    //-----------------------------------------------------------
    void save(SimpleStack *ss) 
    {
      //Push data
      ss->pushByte(index);
      ss->pushName(input);
      ss->pushByte(command);

#if DEFAULT_WIDTH == 1
      ss->pushByte(address);
#endif
#if DEFAULT_WIDTH == 2
      ss->pushInteger(address);
#endif
#if DEFAULT_WIDTH == 4
      ss->pushLong(address);
#endif
      ss->pushByte(use_input_value);
      
      ss->pushArray(data, length);
      
    }
    //-----------------------------------------------------------
    // Default conversion to string
    //-----------------------------------------------------------
    char* toString() 
    {
      return input; //strcat(strcat(input, " => "), output);
    }
};
#endif