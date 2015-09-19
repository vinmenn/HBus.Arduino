# HBus.Arduino
HBus project Low cost, multi-platform, IoT enabled, home automation project - Arduino libraries

##Prerequisites
 - Arduino ide >= 1.6.3
 - Arduino mega + FTDI adapter (for debug messages)
 - Arduino uno / mini / micro
 - Sensor DHT11/22 ( other sensors are possible of course)

##Getting started
- Connect Arduinos with this schema
![Example schema](https://raw.github.com/vinmenn/HBus.Arduino/master/images/sensor-hub example.png)
- Open and upload "test_node.ino" into Arduino mega with first Arduino ide instance
- Open and upload "sensor_node.ino" into Arduino uno/micro with another Arduino ide instance
Open terminal on first instance
Press '1' to read sensor temperature other node
Press '2' to read sensor humidity from other node
Press 't' to toggle Led on other node
Press 's' to subscribe to 20 temperature readings from other node
Press 'u' to unsubscribe from other node before subscribe expiration
