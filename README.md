
# AGROSBUS

The AGROSBUS project wants to deal with the following main challenges:

-   Send/Acquire data from the Implement Messages Application layer by using a ROS bridge to enable the communication of tractor/implement with ROS nodes;
-   Re-use existing ROS packages to automate the navigation of a tractor or to map the environment (e.g. SLAM) to make Agriculture 4.0 scenarios possibile;
-   Make easier the development, deploy, testing and application of advanced applications in Agriculture 5.0 scenarios.

### Basic Architecture

The Bridge Node allows to get messages from the CAN and send back to ROS host using a serial connection. The message encoding includes all the relevant data as the msg id, msg length, message payload and timestamp. The Bridge node is developed using the Arduino and makes use of ros.h  and a custom message definition to represent the CAN-bus frame relevant data and metadata (ID, message length, message payload, timestamp).

![enter image description here](https://s3.amazonaws.com/vrai.univpm/agrosbus/ArchitectureAgrosbus.png)

### ROS CAN Message format
In the actual version the ROS message format we used to encode the CAN message consider the following mandatory data in the CAN frame:

 - message ID
 - message length
 - message payload (max 8 byte)

We also added a timestamp field in order to keep the receiving time of that frame.

    uint32 id
    uint8 lenght
    uint8[] value
    uint64 timestamp

id is able to manage CAN2.0A and CAN2.0B frames.
In order to generate the header file to be used in Arduino it is necessary to run the following command:

### Arduino CAN Bridge to/from ROS
We developed two Arduino sketches.

 - [CANBridge.ino](https://github.com/vrai-group/agrosbus/blob/main/arduino/CANBridge/CANBridge.ino "CANBridge.ino")
 - [CANSender.ino](https://github.com/vrai-group/agrosbus/blob/main/arduino/CANSender/CANSender.ino)
#### CANBridge.ino
[CANBridge.ino](https://github.com/vrai-group/agrosbus/blob/main/arduino/CANBridge/CANBridge.ino) must run on a MKR Arduino eqipped with [CAN Shield](https://www.arduino.cc/en/Guide/MKRCANShield). This shield integrated the [MCP2515](http://ww1.microchip.com/downloads/en/DeviceDoc/20001801H.pdf) chip by Microchip. It is necessary to connect the shield to MKR board through the dual line header connectors. CAN requires CAN_H and CAN_L. In case of long networks or in case of noisy channel we recommend also to connect the GND.
![enter image description here](https://s3.amazonaws.com/vrai.univpm/agrosbus/bridge.png =400x)

The CANBridge.ino requires the arduino-CAN library. Code could be found [here](https://github.com/sandeepmistry/arduino-CAN).
The CANBridge enables the translation from ROS messages into CAN bust messages. The actual release of code is able to manage also a timestamp that should be set by the host computer before starting the ros nodes.
Here is the code to set-up current time on Arduino from the host computer:

    stty -F <serial_port> <baudrate>
    echo "T"$(date +%s) > <serial_port>

In this way the Arduino node is initialized with the current timestamp of the host machine. Next version will also interface the PPS signal from a GPS receiver in order to have a precise sync with an absolute time reference.

Bridge Node that is able to convert ROS messages into  CAN messages and 
viceversa. We tested the code on SAMD devices equipped with CAN Shield.

Messages from / to CAN-bus are buffered in order to compensate different speed of message producer/consumers.

> **Topic naming.** 
> In the actual version the Arduino bridge subscribes to "*fromros*" topic
> and publishes data (in our case CANFrameData) using "*fromarduino*"
> topic.

#### CanSender.ino
This sketch has been designed in order to simulate a set of sensors installed on a device that are able to measure distance (using ultrasonic sensors), temperature and humidity. Here is the list of used HW:

 - Ultrasound distance sensor (HC-RS04)
 - Temperature and Humidity (DHT11)
Here is wiring scheme using the above mentioned sensors and MKR Arduino.
![wiring scheme for sender node](https://s3.amazonaws.com/vrai.univpm/agrosbus/sender.png)

Sensors data are properly encoded in a binary format. For example the humidity sensors is encoded using a UINT16 representation and a union is used to map uint16 into an array of bytes (unsigned char). On the host side  a proper set of [classes](https://github.com/vrai-group/agrosbus/blob/main/agrosbus/scripts/humsensor.py) must be implemented in order to decode the message, encode in a new ROS message that will be published and the consumed by subscribed ROS nodes . 

## ROS Side to Decode / Encode message from/to CAN-bus bridge

### Dependencies
The AGROSBUS relies on [rosserial_python](http://wiki.ros.org/action/fullsearch/rosserial_python?action=fullsearch&context=180&value=linkto%3A%22rosserial_python%22 "Fare clic qui per effettuare una ricerca su tutto il testo per questo titolo") based on [rosserial](http://wiki.ros.org/rosserial). The [rosserial_python](http://wiki.ros.org/action/fullsearch/rosserial_python?action=fullsearch&context=180&value=linkto%3A%22rosserial_python%22 "Fare clic qui per effettuare una ricerca su tutto il testo per questo titolo") is used to interface ROS with Arduino CAN bridge over a serial bus (in particular the Arduino MKR hardware is connected to host computer using USB). 

The following command enables the communication with the Arduino HW.

    rosrun rosserial_python serial_node.py port:=<serial_port> baud:=<baud rate>
Usually Arduino MKR hardware is available as ttyACMx. It is possible to retrieve the list of available USB based serial ports through the following commands:

     $ dmesg
     $ dmesg | grep -i serial
     $ dmesg | grep -i ACM
     $ dmesg | grep -i FTDI

FTDI only in the case you use a FTDI serial - USB converted connected to the UART on Arduino.

### Decode / Encode messages from/to CAN-Bus 

The [listener.py](https://github.com/vrai-group/agrosbus/blob/main/agrosbus/scripts/listener.py) script is responsible to process the incoming messages from the serial bus that contains the received CAN-bus frames. User could customize this script by adding the set of CAN-IDs that should be processed by dedicated classes (e.g. [humsensor.py](https://github.com/vrai-group/agrosbus/blob/main/agrosbus/scripts/humsensor.py),  [tempsensor.py](https://github.com/vrai-group/agrosbus/blob/main/agrosbus/scripts/tempsensor.py), ...). When a new message from the serial bus (with *fromarduino* topic name) is received than based od the ID the proper method is invoked to decode the received frame and then publish the decoded messages to subscribed nodes. 

> **Current limitation**. Actually we assume to have a single network. Next release will be able to manage different networks that could be the case of a complex AG systems where multiple networks are available.

A ROS node that wants to communicate with a CAN-bus device through the  CAN-bridge could publish a [CANFrameData](https://github.com/vrai-group/agrosbus/blob/main/agrosbus/msg/CANFrameData.msg "CANFrameData.msg") message by setting the ID, length, timestamp and payload. This message will be then forwarded through rosserial to CAN-bus bridge. In this case ROS sender should use the "*fromros*" topic name
Run the following command to start listening data from CAN-bus:

    rosrun agrosbuspkg listener.py

 

### Acknowledgement
AGROSBUS for Robot Operating System (ROS) project is developed by UNIVPM.

***
<!-- 
    ROSIN acknowledgement from the ROSIN press kit
    @ https://github.com/rosin-project/press_kit
-->

<a href="http://rosin-project.eu">
  <img src="http://rosin-project.eu/wp-content/uploads/rosin_ack_logo_wide.png" 
       alt="rosin_logo" height="60" >
</a>

Supported by ROSIN - ROS-Industrial Quality-Assured Robot Software Components.  
More information: <a href="http://rosin-project.eu">rosin-project.eu</a>

<img src="http://rosin-project.eu/wp-content/uploads/rosin_eu_flag.jpg" 
     alt="eu_flag" height="45" align="left" >  

This project has received funding from the European Union’s Horizon 2020  
research and innovation programme under grant agreement no. 732287.

