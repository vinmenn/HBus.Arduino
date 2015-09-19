# HBus.Arduino
HBus project Low cost, multi-platform, IoT enabled, home automation project - Arduino libraries

##Prerequisites
 - Arduino ide >= 1.6.3
 - Arduino mega
 - Arduino Uno
 - Sensor LM35 ( other sensors are possible of course)

##Getting started
###Two board connection example
- Open and upload "*hub_node.ino*" into Arduino Mega with first Arduino ide instance
- Open and upload "*sensor_node.ino*" into Arduino Uno with another Arduino ide instance
- Disconnect from usb and connect Arduinos with this 
![schema](https://github.com/vinmenn/HBus.Arduino/blob/master/HNode/images/sensor-hub%20example.png)

Open terminal on first instance
Press '1' to read sensor temperature other node
Press '2' to read sensor humidity from other node
Press 't' to toggle Led on other node
Press 's' to subscribe to 20 temperature readings from other node
Press 'u' to unsubscribe from other node before subscribe expiration
You should see sensor temperatures from Arduino Uno showed into terminal window of Arduino Mega
