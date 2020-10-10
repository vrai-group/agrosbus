#!/usr/bin/env python3
#Licensed under the Apache License, Version 2.0  license. See LICENSE file in the project root for full license information.
# maintainer: Luigi Di Marcantonio agrosbus Project UNIVPM

import sys
import rospy
from agrosbus.msg import CANFrameData #info about packet from arduino
import datetime

#settings for topic to subscribe and Publish
rospy.init_node('ROSNODE', anonymous=True) #node initialising
pub = rospy.Publisher('fromros', CANFrameData, queue_size=10) #for outgoing packets

global log_lvl #affect logging level visble on the cmd-line

#library import + instance creation
#RADAR HCSR04
from radar import Radar
radarHCSR04 = Radar("hcsr04_topic",2,10,pub)

#Temp/HUM DHT11
from tempsensor import Temperature
DHT11 = Temperature("dht11_topic",1,10)

from humsensor import Humidity
DHT11_hum = Temperature("dht11_hum_topic",1,10)


def logfunction(id, int_timestamp, len, payload, dir):
    s = int_timestamp / 1000
    s_timestamp = datetime.datetime.fromtimestamp(s)
    #dir = 1 -> outgoing packet dir=0 -> ingoing packet
    if dir == 0:
        dir = "Incoming packet "
    elif dir == 1:
        dir = "Outgoing packet "
    else:
        dir = "Unknow direction.. Probably losing packets"
        pass
    #print a different level of detail depending on log_lvl
    switcher = {
        0: dir + "ID: " + str(id), #just id and direction

        1: dir + "ID: " + str(id) + " LEN: " + str(len),

        2: dir + "ID: " + str(id) + " LEN: " + str(len) + " T_stamp: "
        + str(s_timestamp.strftime(" %Y-%m-%d %H:%M:%S.%f")[:-3]),

        #up until fully detailed (case: 3)
        3: dir + "ID: " + str(id) + " LEN: " + str(len) + " T_stamp: " +
         str(s_timestamp.strftime(" %Y-%m-%d %H:%M:%S.%f")[:-3]) + " PAYLOAD: " + str(payload)
    }
    #Nothing will be printed out if no log level is set
    rospy.loginfo(switcher.get(log_lvl, "No log level is set.."))



#Callback for incoming packets
def callback(data):
    #printing info about packet that has just arrived
    logfunction(data.id, data.timestamp, data.lenght, data.value,0)

    #call appropriate instace depending on IDs
    if data.id == 20:
        Radar.sendpacket(radarHCSR04, data.value)

    elif data.id == 21:
        Humidity.sendpacket(DHT11_hum, data.value)

    elif data.id == 18:
        Temperature.sendpacket(DHT11, data.value)

    elif data.id == 19:
        Humidity.sendpacket(DHT11_hum, data.value)

#where ros keep sensing packets in both directions
def main_loop():
    #start to listen for coming packets
    while not rospy.is_shutdown():
        pass

rospy.Subscriber('fromarduino', CANFrameData, callback) #for incoming packets

if __name__ == '__main__':
    log_lvl=3 #starts with a default log_lvl of 3 = fully detailed
    try:
        main_loop()
    #if Ros can't start erease an exception
    except rospy.ROSInterruptException:
        print ("ROSInterruptException occured..exit")
        sys.exit()
