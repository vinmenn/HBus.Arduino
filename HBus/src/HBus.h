/*
  =======================================================
  HBus - Home automation/IoT application protocol 
  
  HBus: Bus controller
  
  Copyright (C) 2014 Vincenzo Mennella (see license.txt)

  History
    1.0.0 31/12/2014:   First code release with last architecture
    1.0.1 16/08/2015:   Review of library with last updates
  =======================================================
*/
#ifndef HBUS_H
#define HBUS_H
#define VERSION_HBUS_H   "1.0.1"

//-------------------------------------------------------
//External includes and global defines
//-------------------------------------------------------
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
#include <Streaming.h> 
#endif
//-------------------------------------------------------
//HBus libraries
//-------------------------------------------------------
#include <HBusDefines.h>
#include <Crc16.h>
#include <SimpleStack.h>
#include <HBusPort.h>

// HBus message
typedef struct {
	uint8_t start;
	uint8_t flags;
	hb_address_t source;
	hb_address_t destination;
	uint8_t command;
	uint8_t data[MAX_BUFFER];
	uint16_t length;
	uint16_t crc;
} hb_message_t;

//Map information
//Used to map node names with network addresses
typedef struct {
	char name[NAME_LENGTH+1];
	hb_address_t address;
	uint8_t port;
} hb_node_map_t;

// HBus command handler virtual class
// Is used to hook events to class that implements this methods 
//(useful for libraries)
class HBusMessageHandler
{
   public:
      virtual uint8_t processCommand(hb_message_t message, uint8_t port) = 0;
      virtual uint8_t processAck(hb_message_t message, uint8_t port) = 0;
};
//Simple function handler
//Is used to hook events directly to Arduino code (ino files)
extern "C" {
	// callback function types
    typedef uint8_t (*messageHandler)(hb_message_t message, uint8_t port);
    typedef uint8_t (*getCommandsHandler)(uint8_t* data, uint16_t* length);
} 

//===========================================================
// HBus communication class
// This class is used to encode/decode HBus protocol
// and message routing 
//===========================================================
class HBus {
	private:
		//Bus configuration
		HBusPort *_port[MAX_PORTS];	      // HBus port : receives and sends bus messages
		uint8_t _ports;						        // number of ports (1 -> MAX_PORTS)
    hb_node_map_t _map[MAX_NODES];    // HBus nodes map
    uint8_t _nodes;                   // Map length

    //Message handling
    SimpleStack _rx;                  // Reception stack
    SimpleStack _tx;                  // Transmission stack
    uint8_t _txMsg[MAX_BUFFER];       // Tx message buffer
    uint8_t _txData[TX_BUFFER];       // Tx data buffer
    uint8_t _rxData[RX_BUFFER];       // Rx data buffer
    //uint16_t _fwdIdx;                 // Rx write index
		hb_message_t _rx_msg;				            // Last received message
		HBusMessageHandler * _externalHandler;	// External handler class
		hb_address_t _ack_source;               // Source of waited ack
		uint16_t _ack_crc;		                  // Crc used as id for waited ack
    uint8_t _last_port;

		uint8_t _lastError;					      // Last occurred error
		uint32_t _errors;					        // Total number of errors
		uint8_t _state;						        // Bus global state
        
#if defined(DEBUG) && DEBUG == 3 //DEBUG on Led
    // -------------------------------------------------------
    // Blink debug led n times
    // Parameters: 
    //  n         number of blinks
    //
    // Returns:	  none
    // -------------------------------------------------------
    void blink(uint8_t n) {
        for(uint8_t i=0;i<n;i++) {
          digitalWrite(LED_PIN, HIGH);
          delay(300);
          digitalWrite(LED_PIN, LOW);
          delay(300);
        }
    }
#endif        
    //===========================================================
    //  Messages routines
    //===========================================================
    // -------------------------------------------------------
    // Send HBus message (command or ack response)
    // Parameters: 
    //  flags         HBus message flags
    //  destination   HBus destination address
    //  command       HBus command
    //  data          HBus data array
    //  dataLength    HBus data array length
    //
    // Returns:	true if message sent
    // -------------------------------------------------------
    uint8_t sendMessage(uint8_t flags, hb_address_t destination, uint8_t command, uint8_t* data, uint16_t dataLength, uint8_t port) 
    {
      uint16_t length;
      uint8_t width;
      uint8_t type;
      uint8_t payload ;
      Crc16 calc;
      uint16_t crc;

      _lastError = 0;

      //Check if bus is already transmitting or receiving (for rs485 bus)
      if (_state == BUS_SEND || _state == BUS_RECEIVE)
          return ERR_BUS_BUSY;
            
      length = dataLength + MESSAGE_LENGTH;
      
      //Check buffer length
      if (length > MAX_BUFFER)
          return ERR_RX_OVERSIZE;

      width = flags & 0x0C;
      type  = flags & 0x03;
      payload = flags & 0xF0;

      //set buffer
      if (payload == PAYLOAD_USER)  
          length += 2;
                
      //uint8_t buffer[MAX_BUFFER];
      uint16_t index = 0;

      _txMsg[index++] = START_BYTE;
      _txMsg[index++] = flags;
            
      //Set optional user data length
      if (payload == PAYLOAD_USER)  
      {
          // set user data length for payload
          _txMsg[index++] = (dataLength >> 8) & 0xff;
          _txMsg[index++] = dataLength & 0xff; 
      }
      
      //Set address destination & source
      if (width == ADDRESS_1) {
        _txMsg[index++] = (uint8_t) destination;
        _txMsg[index++] = (uint8_t) address;
      }
      else if (width == ADDRESS_2) {
        _txMsg[index++] = (uint8_t) (destination >> 8);
        _txMsg[index++] = (uint8_t) (destination & 0xFF);
        _txMsg[index++] = (uint8_t) (address >> 8);
        _txMsg[index++] = (uint8_t) (address & 0xFF);
      }
      else if (width == ADDRESS_4) {
        _txMsg[index++] = (uint8_t) ((destination >> 24) & 0xff);
        _txMsg[index++] = (uint8_t) ((destination >> 16) & 0xff);
        _txMsg[index++] = (uint8_t) ((destination >> 8)  & 0xff);
        _txMsg[index++] = (uint8_t) (destination & 0xFF);
        _txMsg[index++] = (uint8_t) ((address >> 24) & 0xff);
        _txMsg[index++] = (uint8_t) ((address >> 16) & 0xff);
        _txMsg[index++] = (uint8_t) ((address >> 8)  & 0xff);
        _txMsg[index++] = (uint8_t) (address & 0xFF);
      }

      //Set command code
      _txMsg[index++] = (uint8_t) command;
      //set data payload
      for(int i = 0;i < dataLength;i++)
          _txMsg[index + i] = data[i];
      index += dataLength;
      
      //calculate crc     
      crc = calc.XModemCrc(_txMsg, 0, index);
      _txMsg[index++] = (crc >> 8) & 0xff;
      _txMsg[index++] = crc & 0xff;

      //Transmit message to one or all ports
      //Broadcast to all ports if port number is 0xff or 
      //if destination address is BROADCAST_ADDRESS
      if (port < _ports && port != 0xff && destination != BROADCAST_ADDRESS) {
          _port[port]->write(_txMsg, length);
      }
			else {
				//0xff => Broadcast to all ports
				for(uint8_t i=0;i<_ports;i++) {
					_port[i]->write(_txMsg, length);
        }
      }
      
      //if normal message set wait information
      if ((type & 0x03) == NORMAL_MSG) {
        _ack_source = destination;
        _ack_crc = crc;
      }
      else {
        //Reset ack source from port
        _ack_source = 0;
        //Reset crc from port (used as message id)
        _ack_crc = 0;
      }
    }
    
    // -------------------------------------------------------
    // Check messages from selected port
    // Parameters: port number
    //
    // Returns: none
    // -------------------------------------------------------
    void checkPort(uint8_t port) 
    {
      uint8_t c = 0;
      uint8_t width;
      uint8_t type;
      uint8_t payload;
      
      _lastError = 0;
      
      //Check actual bus state
      if (_port[port] == 0)
          return;

      //Check for incoming bytes
      uint8_t avail = _port[port]->dataAvailable();

      //Exit if no data
      if (avail == 0) 
        return;

      clear(port);
      //Get start
      _rx_msg.start = read(port);

      //Exit if no START BYTE
      if (_rx_msg.start != START_BYTE) {
        setError(ERR_MESSAGE_CORRUPTED);
        delay(1);
        return;
      }
      //Get flags
      _rx_msg.flags = read(port);
      
      //Extract flag fields
      width = _rx_msg.flags & 0x0C;
      type  = _rx_msg.flags & 0x03;
      payload = _rx_msg.flags & 0xF0;

      // Get payload data length
      switch(payload)
      {
          case PAYLOAD_16:
              _rx_msg.length = 16;
              break;
          case PAYLOAD_32:
              _rx_msg.length = 32;
              break;
          case PAYLOAD_64:
              _rx_msg.length = 64;
              break;
          case PAYLOAD_128:
              _rx_msg.length = 128;
              break;
          case PAYLOAD_256:
              _rx_msg.length = 256;
              break;
          case PAYLOAD_512:
              _rx_msg.length = 512;
              break;
          case PAYLOAD_USER:
              //read user length from message
              _rx_msg.length = (read(port) << 8) | read(port);
              break;
          default:
              _rx_msg.length = (payload >> 4);
              break;
      }

      //Check for buffer overrun
      if (_rx_msg.length > MAX_BUFFER) {
          _port[port]->clear();
          setError(ERR_RX_OVERSIZE);
          return;
      }

      //Wait number of bytes to be received are available
      uint8_t timeout = 0;
      while (_port[port]->dataAvailable() < _rx_msg.length || timeout == MAX_READ) {
          delay(1);
          timeout++;
      }
            
      //Check if all data is received before timeout
      if (timeout >= MAX_READ) {
          _port[port]->clear();
          setError(ERR_RX_TIMEOUT);
          return;
      }
            
      //receive destination & source
      if (width == ADDRESS_1) {
        _rx_msg.destination = read(port);
        _rx_msg.source = read(port);
      }
      else if (width == ADDRESS_2) {
        _rx_msg.destination = (read(port) << 8) | read(port);
        _rx_msg.source = (read(port) << 8) | read(port);
      }
      else if (width == ADDRESS_4) {
        _rx_msg.destination = (read(port) << 24) | (read(port) << 16) | (read(port) << 8) | read(port);
        _rx_msg.source = (read(port) << 24) | (read(port) << 16) | (read(port) << 8) | read(port);
      }
      
      //receive command
      _rx_msg.command = read(port);
            
      //set payload buffer
      if (_rx_msg.length > 0 && _rx_msg.length < MAX_BUFFER) {
          //Receive message payload
          for(int i=0;i<_rx_msg.length;i++) {
              _rx_msg.data[i] = read(port);
          }
      }
            
      //Receive crc
      _rx_msg.crc = (read(port) << 8) | read(port);

      //Release bus status
      _state = BUS_READY;

      //Calculate crc
      uint16_t rxCrc = _port[port]->crc.getCrc();

      //Check crc
      if (rxCrc!= 0) {
          _port[port]->clear();
          setError(ERR_BAD_CRC);
          sendNack(_rx_msg.command, _rx_msg.source, _rx_msg.crc, _lastError, port);
          return;
      }

      //Message is correct
      //Flag set to true if handler process the command correctly
      //This means that hasn't to be retransmitted to other nodes
      //(If is in broadcast and there are mare than one port)
      uint8_t processed = false;
      //Message without address (point to point connection
      uint8_t no_address = width == 0;
      //Message in broadcast
      uint8_t broadcast = _rx_msg.destination == BROADCAST_ADDRESS;
      //Message is for this node
      uint8_t for_me = (_rx_msg.destination == address) || no_address;
      uint8_t from_me = _rx_msg.source == address;
      //Message no requires ack/nack
      uint8_t immediate = type == IMMEDIATE_MSG;
      //Message is ack/nack response
      uint8_t is_ack = (type == ACK_MSG || type == NACK_MSG);
            
      //Check for destination:
      //Message for the node or in broadcast
      if (for_me || broadcast || no_address)
      {
        if (type == NORMAL_MSG || type != IMMEDIATE_MSG) {
          //COMMAND RECEIVED
          //External command handler
          //used from external c++ libraries
          //that implement virtual class
          if (_externalHandler) {
            processed = _externalHandler->processCommand(_rx_msg, port);
          }
          else if (receivedCommand) {
            //Direct command handler
            //used from sketches
            processed = receivedCommand(_rx_msg, port);
            delay(100);
          }
        }
        else {
          //ACK/NACK RECEIVED
          //***********************************
          //TODO: Implement correct message id
          //***********************************
          //Waiting for ack : check if id is correct
          uint16_t id = _rx_msg.data[0] << 8 | _rx_msg.data[1];
          if (true)//if (id == _ack_crc)
          {
              //Process ack/nack externally
              if (_externalHandler) {
                  processed = _externalHandler->processAck(_rx_msg, port);
              }
              else if (receivedAck)
                  //Direct ack handler
                  processed = receivedAck(_rx_msg, port);
          }
          else
          {
              //Set error for lost waited ack
              _lastError = ERR_ACK_LOST;
          }
          
          //Reset waited ack anyway
          _ack_crc = 0;
        }
      }
      //Send response if needed
      if (processed)
      {
        if (for_me && !broadcast && !immediate && !is_ack)
        {
          if (no_address)
          {
            if (_lastError == 0)
              sendAck(_rx_msg.command, _rx_msg.crc, _txData, _tx.writeIndex(), port);
            else
              sendNack(_rx_msg.command, _rx_msg.crc, _lastError, port);
          }
          else
          {
            if (_lastError == 0)
              sendAck(_rx_msg.command, _rx_msg.source, _rx_msg.crc, _txData, _tx.writeIndex(), port);
            else
              sendNack(_rx_msg.command, _rx_msg.source, _rx_msg.crc, _lastError, port);
          }
        }
      }
      else
      {
        if (broadcast) 
          for(uint8_t i=0;i<_ports;i++) {
            if (i == port) continue; //Skip same port
            
            //Forward message to different ports
            sendMessage(_rx_msg.flags, _rx_msg.destination, _rx_msg.command, _rx_msg.data, _rx_msg.length, i);
        }
      }
      
      //Clear HBus port
      _port[port]->clear();
    }	
    // -------------------------------------------------------
    // Read data from HBus port (serial, tcp, etc)
    // Parameters: port number
    // Returns: none
    // -------------------------------------------------------
    uint8_t read(uint8_t port)
    {
      uint8_t value = _port[port]->read();

      return value;
    }
    // -------------------------------------------------------
    // Clear port to start new message reception
    // Parameters: port number
    // Returns: none
    // -------------------------------------------------------
    void clear(uint8_t port)
    {
      //reset crc calculation
      _port[port]->crc.clearCrc();
    }
        
    //===========================================================
    //  Support routines
    //===========================================================

    // -------------------------------------------------------
    // Reset bus status
    // Parameters: none
    // Returns:	none
    // -------------------------------------------------------
    void reset() 
    {
      _lastError = 0;
      _errors = 0;
      _ack_source = 0;	// Source of waited ack
      _ack_crc = 0;	    // Crc used as id for waited ack
      _state = BUS_RESET;
    }
    // -------------------------------------------------------
    // get payload value for specific data length
    // Parameters: payload length 
    // Returns: payload code
    // -------------------------------------------------------
    uint8_t getPayload(uint16_t length)
    {
        if (length <= 8)
            return (length << 4);
        else if (length == 16)
            return PAYLOAD_16;
        else if (length == 32)
            return PAYLOAD_32;
        else if (length == 64)
            return PAYLOAD_64;
        else if (length == 128)
            return PAYLOAD_128;
        else if (length == 256)
            return PAYLOAD_256;
        else if (length == 512)
            return PAYLOAD_512;
        else  
            return PAYLOAD_USER;
    }
    
	public:
    //===========================================================
		//HBus CONSTRUCTORS
    //===========================================================
    // -------------------------------------------------------
    // Initialize with 1 HBus port
    // Parameters: 
    //  portBus: HBus communication port
    // -------------------------------------------------------
		HBus(HBusPort *portBus) 
    {
      _port[0] = portBus;
      for(uint8_t i=1;i<MAX_PORTS;i++)
        _port[i] = 0;
      _ports = 1;
      
      reset();
    }
    // -------------------------------------------------------
    // Initialize with 2 HBus ports
    // Parameters: 
    //  portBus_0: 1st HBus communication port
    //  portBus_1: 2nd HBus communication port
    // -------------------------------------------------------
		HBus(HBusPort *portBus_0, HBusPort *portBus_1) 
    {
        _port[0] = portBus_0;
        _port[1] = portBus_1;
        for(uint8_t i=2;i<MAX_PORTS;i++)
          _port[i] = 0;
        _ports = 2;
        
        reset();
    }
    // -------------------------------------------------------
    // Initialize with 3 HBus ports
    // Parameters: 
    //  portBus_0: 1st HBus communication port
    //  portBus_1: 2nd HBus communication port
    //  portBus_2: 3rd HBus communication port
    // -------------------------------------------------------
		HBus(HBusPort *portBus_0, HBusPort *portBus_1, HBusPort *portBus_2) 
    {
      _port[0] = portBus_0;
      _port[1] = portBus_1;
      _port[2] = portBus_2;
      for(uint8_t i=3;i<MAX_PORTS;i++)
        _port[i] = 0;
      _ports = 3;
      
      reset();
    }
    //===========================================================
		//HBus ROUTINES
    //===========================================================
    // -------------------------------------------------------
    // Initialize bus controller
    // Parameters: node Address
    // Returns:	none
    // -------------------------------------------------------
    void begin(hb_address_t nodeAddress) 
    {
      address = nodeAddress;

      _tx.begin(_txData, MAX_BUFFER, NAME_LENGTH);
      _rx.begin(_rxData, MAX_BUFFER, NAME_LENGTH);

      for(uint8_t i=0; i<_ports;i++) {
        if (_port[i])
          _port[i]->begin();
      }
      _state = BUS_READY;
    }
    
    // -------------------------------------------------------
    // Check serial lines for incoming messages
    // Parameters: none
    // Returns:	none
    // -------------------------------------------------------
    void checkMessages() 
    {
      for(uint8_t i=0; i<_ports;i++)  
        checkPort(i);
    }
		
    // -------------------------------------------------------
    // send normal command without address (point to point connection)
    // and without payload
    // Parameters: 
    //  command: command code, port number
    // Returns:	true if ack sent correctly
    // -------------------------------------------------------
		uint8_t sendCommand(uint8_t command, uint8_t port) 
		{
      uint16_t length = _tx.writeIndex();
			uint8_t payload = getPayload(length);

			return sendMessage(NORMAL_MSG | NO_ADDRESS | PAYLOAD_0, 0, command, _txData, length, port);
		}
    // -------------------------------------------------------
    // send normal command with address
    // and without payload
    // Parameters: 
    //  command: command code, destination address, port number
    // Returns:	true if ack sent correctly
    // -------------------------------------------------------
		uint8_t sendCommand(uint8_t command, hb_address_t destination, uint8_t port) 
		{
      uint16_t length = _tx.writeIndex();
			uint8_t payload = getPayload(length);

			return sendMessage(NORMAL_MSG | ADDRESS_WIDTH | payload, destination, command, _txData, length, port);
		}
    // -------------------------------------------------------
    // send normal command with destination address and payload
    // Parameters: 
    //  command: command code
    //  destination: destination node address    
    //  data: payload
    //  length: payload length
    //  port: port number
    // Returns:	true if ack sent correctly
    // -------------------------------------------------------
		uint8_t sendCommand(uint8_t command, hb_address_t destination, uint8_t* data, uint16_t length, uint8_t port) 
		{
			uint8_t payload = getPayload(length);
      
			return sendMessage(NORMAL_MSG | ADDRESS_WIDTH | payload, destination, command, data, length, port);
		}
    // -------------------------------------------------------
    // send immediate command without address (point to point connection)
    // and without payload
    // Parameters: 
    //  command: command code, port number
    // Returns:	true if ack sent correctly
    // -------------------------------------------------------
		uint8_t sendImmediate(uint8_t command, uint8_t port) 
		{
      uint16_t length = _tx.writeIndex();
			uint8_t payload = getPayload(length);
      
			return sendMessage(IMMEDIATE_MSG | NO_ADDRESS | payload, 0, command, _txData, length, port);
		}
    // -------------------------------------------------------
    // send immediate command with destination address
    // Parameters: 
    //  command: command code
    //  destination: destination node address    
    //  port: port number
    // Returns:	true if ack sent correctly
    // -------------------------------------------------------
		uint8_t sendImmediate(uint8_t command, hb_address_t destination, uint8_t port) 
		{
      uint16_t length = _tx.writeIndex();
			uint8_t payload = getPayload(length);
      
			return sendMessage(IMMEDIATE_MSG |  ADDRESS_WIDTH | payload, destination, command, _txData, length, port);
		}
    // -------------------------------------------------------
    // send immediate command with destination address and payload
    // Parameters: 
    //  command: command code
    //  destination: destination node address    
    //  data: payload
    //  length: payload length
    //  port: port number
    // Returns:	true if ack sent correctly
    // -------------------------------------------------------
		uint8_t sendImmediate(uint8_t command, hb_address_t destination, uint8_t* data, uint16_t length, uint8_t port) 
		{
			uint8_t payload = getPayload(length);
      
			return sendMessage(IMMEDIATE_MSG |  ADDRESS_WIDTH | payload, destination, command, data, length, port);
		}
    // -------------------------------------------------------
    // send ack response with payload data without address
    // Parameters: 
    // command: command code
    //  destination: destination node address    
    //  crc_id: crc of received command (used as message Id)
    //  data: payload
    //  length: payload length
    //  port: port number
    // Returns:	true if ack sent correctly
    // -------------------------------------------------------
		uint8_t sendAck(uint8_t command, uint16_t crc_id, uint8_t* data, uint16_t length, uint8_t port) 
    {
      data[length++] = (crc_id >> 8) & 0xff;
      data[length++] = (crc_id & 0xff);
			uint8_t payload = getPayload(length);
      
			return sendMessage(ACK_MSG | NO_ADDRESS | PAYLOAD_0, 0, command, data, length, port);
		}
    // -------------------------------------------------------
    // send ack response with payload data
    // Parameters: 
    // command: command code
    //  crc_id: crc of received command (used as message Id)
    //  data: payload
    //  length: payload length
    //  port: port number
    // Returns:	true if ack sent correctly
    // -------------------------------------------------------
		uint8_t sendAck(uint8_t command, hb_address_t destination, uint16_t crc_id, uint8_t* data, uint16_t length, uint8_t port) 
    {
      data[length++] = (crc_id >> 8) & 0xff;
      data[length++] = (crc_id & 0xff);
			uint8_t payload = getPayload(length);
      
			return sendMessage(ACK_MSG | ADDRESS_WIDTH | payload, destination, command, data, length, port);
		}
    // -------------------------------------------------------
    // send nack response with error code without address
    // Parameters: 
    //  command: command code
    //  destination: destination node address    
    //  crc_id: crc of received command (used as message Id)
    //  error_code: error code sent with nack
    //  data: error code
    // Returns:	true if ack sent correctly
    // -------------------------------------------------------
		uint8_t sendNack(uint8_t command, uint16_t crc_id, uint8_t error_code, uint8_t port) 
    {
			uint8_t data[3] = { (uint8_t) (crc_id >> 8) & 0xff, (uint8_t) (crc_id & 0xff), error_code };
      
			return sendMessage(NACK_MSG | NO_ADDRESS | PAYLOAD_3, 0, command, data, 3, port);
		}
    // -------------------------------------------------------
    // send nack response with error code
    // Parameters: 
    //  command: command code
    //  destination: destination node address    
    //  crc_id: crc of received command (used as message Id)
    //  error_code: error code sent with nack
    //  data: error code
    // Returns:	true if ack sent correctly
    // -------------------------------------------------------
		uint8_t sendNack(uint8_t command, hb_address_t destination, uint16_t crc_id, uint8_t error_code, uint8_t port) 
    {
			uint8_t data[3] = { (uint8_t) (crc_id >> 8) & 0xff, (uint8_t) (crc_id & 0xff), error_code };
      
			return sendMessage(NACK_MSG | ADDRESS_WIDTH | PAYLOAD_3, destination, command, data, 3, port);
		}

    //===========================================================
		// External handlers
    //===========================================================
    // -------------------------------------------------------
    // Register a callback class to process HBus commands
    // Parameters: callbackObject: callback class
    // Returns:	none
    // -------------------------------------------------------
    void registerCallback(HBusMessageHandler *callback_object)
    {
        _externalHandler = callback_object;
    }
    
    //===========================================================
		//Errors management
    //===========================================================
    // -------------------------------------------------------
    // Clear all errors
    // Parameters: none
    // Returns: none
    // -------------------------------------------------------
    void clearErrors()
    {
        _lastError = 0;
        _errors = 0;
    }
    // -------------------------------------------------------
    // Set error code
    // Parameters: error code
    // Returns: none
    // -------------------------------------------------------
    void setError(uint8_t error)
    {
        _lastError = error;
        _errors++;
    }
    // -------------------------------------------------------
    // reset last error code
    // Parameters: error code
    // Returns: none
    // -------------------------------------------------------
    void resetError()
    {
        _lastError = 0;
    }
    // -------------------------------------------------------
    // Returns last error code
   
    // Parameters: none
    // Returns: last error code
    // -------------------------------------------------------
    uint8_t getError()
    {
        return _lastError;
    }
    // -------------------------------------------------------
    // Returns total errors from reset
    // Parameters: none
    // Returns: Total errors from reset
    // -------------------------------------------------------
    uint32_t totalErrors()
    {
        return _errors;
    }
        
    //===========================================================
		//General properties
    //===========================================================
		hb_address_t address;				                    // Controller address
    SimpleStack *rx() { return &_rx; }              // Reception stack
    SimpleStack *tx() { return &_tx; }              // Transmission stack
		uint8_t getStatus() { return _state;}           // Current status of bus communications
		messageHandler receivedCommand;                 // Direct callback handler for received commands
		messageHandler receivedAck;                     // Direct callback handler for received acks
		
};

#endif