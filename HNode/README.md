# HBus.Arduino
HBus project Low cost, multi-platform, IoT enabled, home automation project - Arduino libraries

##Prerequisites
 - Arduino ide >= 1.6.3
 - Arduino mega (hub node)
 - Arduino Uno (sensor node)
 - Sensor LM35 for sensor node
 - Sensor DHT11/22 for hub node

##Getting started
###Two board connection example
- Open and upload "*hub_node.ino*" into Arduino Mega with first Arduino ide instance
- Open and upload "*sensor_node.ino*" into Arduino Uno with another Arduino ide instance
- Disconnect from usb and connect Arduinos with this 
![schema](https://github.com/vinmenn/HBus.Arduino/blob/master/HNode/images/sensor-hub%20example.png)

Open terminal on first instance
- Press 'l' to toggle Led on sensor node
- Press 'r' to read remote sensor temperature from LM35
- Press 's' to subscribe to 20 temperature readings from sensor node
- Press 'u' to unsubscribe from sensor node before subscribe expiration
- Press 't' to read local temperature from DHT11/22
- Press 'h' to read local humidity from DHT11/22

You should see sensors values showed into terminal window of Arduino Mega.
Pressing l will toggle led on Arduino Uno
