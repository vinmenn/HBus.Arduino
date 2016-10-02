# HBus an home automation / Internet of things protocol

## Description
HBus is a multi-platform library for home automation and ioT purposes.
It’s architecture is around a network of simple independent devices (nodes) that communicates each other commands triggered from different input events.

HBus is composed of several parts:

+ A simple communication protocol
+ A base communication library implementend on different platforms from simple device to more powerful platforms.
+ A companion library **HNode** that uses HBus to handle home automation / IoT features

## Usage
If you want to use complete suite with node functions please follow instructions in HNode library.
Otherwise if you want to use only HBus protocol you can follow these guidelines.

main class is HBus that is the communication controller that sends and receives messages through communication ports. 
Implemented are serial port with TXEnable pin for RS485 communications and TCP port with ethernet shield.

Define bus controller with its port:

```c++
  HBusPort *port = new HBusSerialPort(Serial, 0);
  HBus *bus = new HBus(port);
```

Add handler routine for received commands
```c++
  bus->receivedCommand = cmdReceived;
```

Initialize bus with HBus address
```c++
  bus->begin(NODE_ADDRESS); //HBus Address
```

Add check routine in the main loop:
```c++
  //Check hbus commands
  bus->checkMessages();
```

Use this signature for handler routine:

```c++
uint8_t cmdReceived(hb_message_t message, uint8_t port) {

  if ((message.flags & 0x03) == ACK_MSG)
    ... ACK processing
  else if ((message.flags & 0x03) == NORMAL_MSG || (message.flags & 0x03) == IMMEDIATE_MSG)
    ... command processing
  
}
```
### HBus Commands
+ Standard HBus commands are defined into HBusDefines.h with range 0x00 - 0x6F
If you want to extend standard commands please send me a note and I'll integrate into documentation (if command is not in contrast with already defined commands)
+ User commands that not fall into predefined categories could be defined in range 0x70 - 0xDF
+ Command in range 0xE0 - 0xFF are reserved for future uses.
