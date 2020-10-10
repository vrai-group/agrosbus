
# AGROSBUS

The AGROSBUS project wants to deal with the following main challenges:

-   Send/Acquire data from the Implement Messages Application layer by using a ROS bridge to enable the communication of tractor/implement with ROS nodes;
-   Re-use existing ROS packages to automate the navigation of a tractor or to map the environment (e.g. SLAM) to make Agriculture 4.0 scenarios possibile;
-   Make easier the development, deploy, testing and application of advanced applications in Agriculture 5.0 scenarios.

### For ROS
The code to send/receive message from/to CAN network
### For Arduino
Bridge Node that is able to convert ROS messages into  CAN messages and viceversa. We tested the code on SAMD devices equipped with CAN Shield.

### Basic Architecture

The Bridge Node allows to get messages from the CAN and send back to ROS host using a serial connection. The message encoding includes all the relevant data as the msg id, msg length, message payload and timestamp. The Bridge node is developed using the Arduino and makes use of ros.h  and a custom message.

![enter image description here](https://s3.amazonaws.com/vrai.univpm/agrosbus/ArchitectureAgrosbus.png)

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
