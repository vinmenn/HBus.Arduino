//-------------------------------------------------------------------------------------
// HBus - Home automation/IoT application protocol 
// Copyright (C) 2014 Vincenzo Mennella (see license.txt)
// History
//  0.1.0 31/12/2014:   First public code release
//
// License
// "MIT Open Source Software License":
// Permission is hereby granted, free of charge, to any person obtaining a copy of
// this software and associated documentation files (the "Software"), to deal in the
// Software without restriction, including without limitation the rights to use, copy,
// modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
// and to permit persons to whom the Software is furnished to do so, subject to
// the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
// FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
// COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
// IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
// WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//-------------------------------------------------------------------------------------
#ifndef HBUS_H
#define HBUS_H
#define LIBRARY_VERSION_HBUS_H   "0.1.0"

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
class HBusMessageHandler
{
   public:
      virtual uint8_t processCommand(hb_message_t message, uint8_t port) = 0;
      virtual uint8_t processAck(hb_message_t message, uint8_t port) = 0;
};
//Simple function handler
//Is used to hook commands event directly to Arduino code
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
		hb_address_t _address;				    // Node address
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
    uint16_t _fwdIdx;                   // Rx write index
		hb_message_t _rx_msg;				      // Last received message
		HBusMessageHandler * _externalHandler;	// External handler class
		hb_address_t _ack_source;               // Source of waited ack
		uint16_t _ack_crc;		            // Crc used as id for waited ack
    uint8_t _last_port;

		uint8_t _lastError;					      // Last occurred error
		uint32_t _errors;					        // Total number of errors
		uint8_t _state;						        // Bus global state
        
#ifdef DEBUG        
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
    //  flags
    //  destination
    //  command
    //  data
    //  dataLength
    // Returns:	true if message sent
    // -------------------------------------------------------
    uint8_t sendMessage(uint8_t flags, hb_address_t destination, uint8_t command, uint8_t* data, uint16_t dataLength, uint8_t port) 
    {

      _lastError = 0;

      //Check if bus is already transmitting or receiving (for rs485 bus)
      if (_state == BUS_SEND || _state == BUS_RECEIVE)
      {
          return ERR_BUS_BUSY;
      }
            
      uint16_t length = dataLength + MESSAGE_LENGTH;
      //Check buffer length
      if (length > MAX_BUFFER)
          return ERR_RX_OVERSIZE;

      uint8_t width = flags & 0x0C;
      uint8_t type  = flags & 0x03;
      uint8_t payload = flags & 0xF0;

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
        _txMsg[index++] = (uint8_t) _address;
      }
      else if (width == ADDRESS_2) {
        _txMsg[index++] = (uint8_t) (destination >> 8);
        _txMsg[index++] = (uint8_t) (destination & 0xFF);
        _txMsg[index++] = (uint8_t) (_address >> 8);
        _txMsg[index++] = (uint8_t) (_address & 0xFF);
      }
      else if (width == ADDRESS_4) {
        _txMsg[index++] = (uint8_t) ((destination >> 24) & 0xff);
        _txMsg[index++] = (uint8_t) ((destination >> 16) & 0xff);
        _txMsg[index++] = (uint8_t) ((destination >> 8)  & 0xff);
        _txMsg[index++] = (uint8_t) (destination & 0xFF);
        _txMsg[index++] = (uint8_t) ((_address >> 24) & 0xff);
        _txMsg[index++] = (uint8_t) ((_address >> 16) & 0xff);
        _txMsg[index++] = (uint8_t) ((_address >> 8)  & 0xff);
        _txMsg[index++] = (uint8_t) (_address & 0xFF);
      }

      //Set command code
      _txMsg[index++] = (uint8_t) command;
      //set data payload
      for(int i = 0;i < dataLength;i++)
          _txMsg[index + i] = data[i];
      index += dataLength;
      //calculate crc     
      Crc16 calc;
      unsigned int crc = calc.XModemCrc(_txMsg, 0, index);
      _txMsg[index++] = (crc >> 8) & 0xff;
      _txMsg[index++] = crc & 0xff;
          
      if (port < _ports && port != 0xff) {
          _port[port]->write(_txMsg, length);
      }
			else {
				//Broadcast to all ports
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
    // Check messages from selected serial line
    // Parameters: port number
    // Returns: none
    // -------------------------------------------------------
    void checkPort(uint8_t port) 
    {
      uint8_t c = 0;
      
      //Check actual bus state
      if (_port[port] == 0)
          return;

 // #if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
  // Serial << "dataAvailable => " << endl;
  // delay(200);
// #endif

      //Check for incoming bytes
      uint8_t avail = _port[port]->dataAvailable();

      // if (avail > 0) 
        // delay(100);
      
      while (_port[port]->dataAvailable() && c != START_BYTE) {
        //reset forward index & crc
        clear(port);
        c = read(port);
        delay(20);
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
      Serial << c << " " <<_HEX(c) << ": " << (char) c << endl;
#endif
      }
      
      if ( c!= START_BYTE) return;
      
      // avail = _port[port]->dataAvailable();
      // if(avail < MESSAGE_LENGTH )  {// buffer up a min length message
          // return;
      // }
      _lastError = 0;

#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
//      Serial << "rx from port" << port << endl;
#endif
      
      //get start & flags
      _rx_msg.start = c;
      _rx_msg.flags = read(port);
      
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
      // Serial << "rx start" << _rx_msg.start << endl;
      // Serial << "rx flags" << _rx_msg.flags << endl;
#endif
      //Check for message header
      if (_rx_msg.start != START_BYTE) {
        _port[port]->clear();
        setError(ERR_MESSAGE_CORRUPTED);
        return;
      }

#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
      // Serial << "\t start" << endl;
#endif
      //Extract flag fields
      uint8_t width = _rx_msg.flags & 0x0C;
      uint8_t type  = _rx_msg.flags & 0x03;
      uint8_t payload = _rx_msg.flags & 0xF0;

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

#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
      // Serial << "\t length "<<  _rx_msg.length << endl;
#endif
      //Wait number of bytes to be received are available
      uint8_t timeout = 0;
      while (_port[port]->dataAvailable() < _rx_msg.length || timeout == MAX_READ) {
          delay(20);
          timeout++;
      }
            
      //Check if all data is received before timeout
      if (timeout == MAX_READ) {
          _port[port]->clear();
          setError(ERR_RX_TIMEOUT);
          return;
      }
            
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
      //Serial << "\t dest-src" << endl;
#endif
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
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
      // Serial << "\t ok" << endl;
#endif

      //Message ok
      //Flag set to true if handler process the command correctly
      //This means that hasn't to be retransmitted to other nodes
      //(If is in broadcast and there are mare than one port)
      uint8_t processed = false;
            
      //Check for destination:
      //Message for the node or in broadcast
      if (_rx_msg.destination == BROADCAST_ADDRESS || _rx_msg.destination == _address)
      {
        if (type == ACK_MSG || type == NACK_MSG) {
          if (_rx_msg.destination == _address) { // && _ack_crc != 0) {

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
        else {
          if (!processed) {
            //External command handler
            //used from external c++ libraries
            //that implement virtual class
            if (_externalHandler) {
              processed = _externalHandler->processCommand(_rx_msg, port);
            }
            //Direct command handler
            //used from sketches
            else if (receivedCommand) {
              processed = receivedCommand(_rx_msg, port);
            }
          }
        }
      }
      //Send response if needed
      if (_rx_msg.destination == _address && type == NORMAL_MSG) {
        if (processed) {
          if (_lastError == 0) {
            sendAck(_rx_msg.command, _rx_msg.source, _rx_msg.crc, _txData, _tx.writeIndex(), port);
          }
          else {
            sendNack(_rx_msg.command, _rx_msg.source, _rx_msg.crc, _lastError, port);
          }
        }
      }
#if defined(LED0_PIN) && LED0_PIN > 0 //DEBUG on Serial
      digitalWrite(LED1_PIN, LOW);
      digitalWrite(LED0_PIN, LOW);
#endif
      if (!processed && _rx_msg.destination != _address && _ports > 1)  {
        //Retransmit message if not processed
        //(In case of more than one port)
        //Retransmit to the other lines
        //***********************************
        //TODO network architecture STAR/RING
        //***********************************
        for(uint8_t i=0;i<_ports;i++) {
          if (i == port) continue; //Skip same port
#if defined(LED0_PIN) && LED0_PIN > 0 //DEBUG on Serial
          if (i == 0) {
            digitalWrite(LED0_PIN, HIGH);
            digitalWrite(LED1_PIN, LOW);
          } else {
            digitalWrite(LED1_PIN, HIGH);
            digitalWrite(LED0_PIN, LOW);
          }
#endif
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
      // Serial << "fwd from " << port << " to " << i << endl;
      // Serial << "\t length " << _fwdIdx << endl;
#endif
          
          _state = BUS_SEND;
          _port[i]->write(_txMsg, _fwdIdx);
          _state = BUS_READY;
          blink(1);
        }    
      }
            
      //Clear serial port
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
      
      //Store received byte to forward messages
      _txMsg[_fwdIdx++] = value;
      
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
      _fwdIdx = 0;
    }
    // // -------------------------------------------------------
    // // forward message to specific port
    // // Doesn't recalculate crc & flags
    // // Parameters:  port number
    // // Returns: none
    // // -------------------------------------------------------
    // void forwardMessage(uint8_t port)
    // {
      // if (_fwdIdx < MESSAGE_LENGTH) return;
      
      // _state = BUS_SEND;
      // _port[port]->write(_txMsg, _fwdIdx);
      // _state = BUS_READY;
      
    // }
    // -------------------------------------------------------
    // Internal commands processor
    // Used for specific bus commands 
    // Parameters:  received message, pot number
    // Returns: true/false
    // -------------------------------------------------------
    uint8_t processCommand(hb_message_t command, uint8_t port)
    {
      //By default command is processed
      uint8_t processed = true;
      uint16_t length = 0;
      //Command type
      uint8_t cmd_type  = command.flags & 0x03;
            
      switch(command.command)
      {
        /*    
        case CMD_PING:
            //OK
            break;
        case CMD_DISCOVER_MAP:
            updateMap(command.data, command.length, data, &length);
            break;
        case CMD_SET_MAP:
            setMap(command.data, command.length, port);
            break;
        case CMD_GET_MAP:
            getMap(data, &length, port);
            break;
        case CMD_GET_STATUS:
            data[0] = _state;
            length = 1;
            break;
        case CMD_GET_ERROR:
            data[0] = _lastError;
            data[1] = (_errors >> 24);
            data[2] = (_errors >> 16);
            data[3] = (_errors >> 8);
            data[4] = (_errors & 0xff);
            length = 5;
            break;
        case CMD_GET_COMMANDS:
            if (_externalHandler)
                processed = _externalHandler->getCommands(data, &length);
            else if (supportedCommands!=0)
                processed = supportedCommands(data, &length);
            //Add bus commands
            processed = busCommands(data, &length);
            break;
        */            
        default:
          processed = false;
          break;
      }
      return processed;
    }
		
    //===========================================================
    //  Map routines
    //===========================================================
    // -------------------------------------------------------
    //Add new node information to node map
    // Parameters:  node address
    // Returns: none
    // -------------------------------------------------------
    // void addNode(hb_address_t source)
    // {
      // char name[NAME_LENGTH+1];              // Fixed length name

      // //Receive node name
      // //_rx.popFixedString(name, NAME_LENGTH);
        
      // //Search if node already mapped
      // for(uint8_t i=0;i<_nodes;i++)
          // if (strcmp(_map[i].name, name)==0)
             // return;
             
      // //Map node name with address
      // if (_nodes < MAX_NODES) {
          // strcpy(_map[_nodes].name, name);
          // _map[_nodes].address = source;
      // }
    // }
    
    // -------------------------------------------------------
    // Update transmitted map with this node information
    // Parameters:  node name
    // Returns: none
    // -------------------------------------------------------
    // void updateMap(char *name)
    // {   
        // char name2[NAME_LENGTH+1];              // Fixed length name
        // uint8_t length = _rx.popByte();
        
        // _tx.pushByte(length+1);
        // //Check if this node is already mapped
        // for(uint8_t i=0;i<length;i++) {
            // _rx.popName(name2);
            // _tx.pushName(name2);
            // _tx.pushAddress( _rx.popAddress() );
        // }
        // //Add this node information
        // _tx.pushName(name);
        // _tx.pushAddress(_address);
    // }
    // -------------------------------------------------------
    // Store node full map
    // Parameters:  none
    // Returns: none
    // -------------------------------------------------------
    // void setMap()
    // {
      // _nodes = _rx.popByte();
      // for(uint8_t i=0;i<_nodes;i++) {
          
          // _rx.popName(_map[i].name);
          // _map[i].address = _rx.popAddress();
      // }
    // }
    // -------------------------------------------------------
    // Transmit node full map
    // Parameters:  none
    // Returns: none
    // -------------------------------------------------------
    // void getMap()
    // {
      // _tx.pushByte(_nodes);
      // for(uint8_t i=0;i<_nodes;i++) {
          // _tx.pushName(_map[i].name);
          // _tx.pushAddress(_map[i].address);
      // }
    // }
    
    // -------------------------------------------------------
    // get address distance from node in network "hops"
    // Parameters: target node address
    // Returns: 0 if map is not defined or message is broadacast
    //  0xffff id address not found
    //  <value> = hops to destinations
    // -------------------------------------------------------
    uint16_t getDistance(hb_address_t address) {

      uint16_t s = 0xffff, e = 0xffff;
            
      //If message is broadcast or map not defined return 0
      // to use selected port
      if (address == BROADCAST_ADDRESS || _nodes == 0)
          return 0;
          
      //Error if address is node itself
      if (address == _address)
          return 0xffff;
          
      //Find this node index to start counting hops
      for(uint8_t i =0;i < _nodes;i++)
      {
        //Start of path
        if (_map[i].address == _address)
        {
          s = i;
          break;
        }
        //End of path
        if (_map[i].address == address)
        {
          e = i;
          break;
        }
        //Return number of hops
        if (s < 0xffff && e < 0xffff)
        {
          if (s < e) 
            return (e - s);
          else
            return (s - e);
        }
      }

      //Destination not found
      return 0xffff;
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
      _ack_crc = 0;	// Crc used as id for waited ack
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
    // Init with 1 HBus port
    // Parameters: 
    //  portBus: HBus communication port
    // -------------------------------------------------------
		HBus(HBusPort *portBus) {
      _port[0] = portBus;
      for(uint8_t i=1;i<MAX_PORTS;i++)
        _port[i] = 0;
      _ports = 1;
      
      reset();
    }
    // -------------------------------------------------------
    // Init with 2 HBus ports
    // Parameters: 
    //  portBus_0: 1st HBus communication port
    //  portBus_1: 2nd HBus communication port
    // -------------------------------------------------------
		HBus(HBusPort *portBus_0, HBusPort *portBus_1) {
        _port[0] = portBus_0;
        _port[1] = portBus_1;
        for(uint8_t i=2;i<MAX_PORTS;i++)
          _port[i] = 0;
        _ports = 2;
        
        reset();
    }
    // -------------------------------------------------------
    // Init with 3 HBus ports
    // Parameters: 
    //  portBus_0: 1st HBus communication port
    //  portBus_1: 2nd HBus communication port
    //  portBus_2: 3rd HBus communication port
    // -------------------------------------------------------
		HBus(HBusPort *portBus_0, HBusPort *portBus_1, HBusPort *portBus_2) {
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
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
      Serial << "start bus" << endl;
      //delay(500);
#endif
      _address = nodeAddress;

      _tx.begin(_txData, MAX_BUFFER, NAME_LENGTH);
      _rx.begin(_rxData, MAX_BUFFER, NAME_LENGTH);

#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
      //Serial << "init ports" << endl;
      //delay(500);
#endif
      for(uint8_t i=0; i<_ports;i++) {
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
      //Serial << "port[" << i << "]" << endl;
      //delay(500);
#endif
        if (_port[i])
          _port[i]->begin();
      }
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
      //Serial << "bus done" << endl;
      //delay(500);
#endif
      _state = BUS_READY;
    }
    
    // -------------------------------------------------------
    // Initialize bus controller
    
    // Parameters: node Address
    //  nodes map
    // Returns:	none
    // -------------------------------------------------------
    // void begin(hb_address_t nodeAddress, hb_node_map_t map[], uint8_t length) 
    // {

      // _nodes = length;
      
      // for(uint8_t i=0;i<_nodes;i++) {
        // strcpy(map[i].name, _map[i].name);
        // _map[i].address = map[i].address;
      // }
            
      // begin(nodeAddress);
    // }

    // -------------------------------------------------------
    // Check serial lines for incoming messages
    // Parameters: none
    // Returns:	none
    // -------------------------------------------------------
    void checkMessages() 
    {
      for(uint8_t i=0; i<_ports;i++)  checkPort(i);
                
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
#if defined(DEBUG) && DEBUG == 1 //DEBUG on Serial
      // Serial << "sendCommand " << length << endl;
#endif
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
    // send ack response with payload data
    // Parameters: 
    // command: command code
    //  destination: destination node address    
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
			uint8_t data[3] = { (uint8_t) (crc_id >> 8) & 0xff, (uint8_t) crc_id & 0xff, error_code };
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
    
		messageHandler receivedCommand;                 // Direct callback handler for received commands
		messageHandler receivedAck;                     // Direct callback handler for received acks

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
    SimpleStack *rx() { return &_rx; }              // Reception stack
    SimpleStack *tx() { return &_tx; }              // Transmission stack
		hb_address_t getAddress() { return _address; }  // Address assigned to communications
		uint8_t getStatus() { return _state;}           // Current status of bus communications
		
};

#endif