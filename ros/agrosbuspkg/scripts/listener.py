#!/usr/bin/env python3
#Licensed under the Apache License, Version 2.0  license.
# See LICENSE file in the project root for full license information.
# maintainer: Luigi Di Marcantonio agrosbus Project UNIVPM
"""Main script to interface CAN-bus Arduino bridge and ROS"""
import sys
import datetime
import rospy
from agrosbuspkg.msg import CANFrameData #info about packet from arduino
from humsensor import Humidity
from radar import Radar
from tempsensor import Temperature
#settings for topic to subscribe and Publish
rospy.init_node('ROSNODE', anonymous=True) #node initialising
#setting publisher
PUB = rospy.Publisher('fromros', CANFrameData, queue_size=10) #for outgoing packets

#log level
LOG_LEVEL = 3 #affect logging level visble on the cmd-line

#RADAR HCSR04
RADARHCSR04 = Radar("hcsr04_topic", 2, 10, PUB)

#Temp/HUM DHT11
DHT11 = Temperature("dht11_topic", 1, 10)

#HUM DHT11
DHT11_HUM = Temperature("dht11_hum_topic", 1, 10)

def logfunction(msg_id, int_timestamp, msg_len, msg_payload, direction):
    """this function logs event with different levels"""
    s_timestamp = int_timestamp / 1000
    s_timestamp = datetime.datetime.fromtimestamp(s_timestamp)
    #direction = 1 -> outgoing packet direction=0 -> ingoing packet
    if direction == 0:
        direction = "Incoming packet "
    elif direction == 1:
        direction = "Outgoing packet "
    else:
        direction = "Unknow direction.. Probably losing packets"

    #print a different level of detail depending on LOG_LEVEL
    switcher = {
        0: direction + "ID: " + str(msg_id), #just id and direction

        1: direction + "ID: " + str(msg_id) + " LEN: " + str(msg_len),

        2: direction + "ID: " + str(msg_id) + " LEN: " + str(msg_len) + " T_stamp: " +
           str(s_timestamp.strftime(" %Y-%m-%d %H:%M:%S.%f")[:-3]),

        #up until fully detailed (case: 3)
        3: direction + "ID: " + str(msg_id) + " LEN: " + str(msg_len) + " T_stamp: " +
           str(s_timestamp.strftime(" %Y-%m-%d %H:%M:%S.%f")[:-3]) + " PAYLOAD: " + str(msg_payload)
    }
    #Nothing will be printed out if no log level is set
    rospy.loginfo(switcher.get(LOG_LEVEL, "No log level is set.."))



#Callback for incoming packets
def callback(data):
    """This callback function reacts to messages from CAN-bus bridge using rosserial"""
    #printing info about packet that has just arrived
    logfunction(data.id, data.timestamp, data.lenght, data.value, 0)

    #call appropriate instace depending on IDs
    if data.id == 20:
        Radar.sendpacket(RADARHCSR04, data.value)

    elif data.id == 21:
        Humidity.sendpacket(DHT11_HUM, data.value)

    elif data.id == 18:
        Temperature.sendpacket(DHT11, data.value)

    elif data.id == 19:
        Humidity.sendpacket(DHT11_HUM, data.value)

#where ros keep sensing packets in both directions
def main_loop():
    """main loop"""
    #start to listen for coming packets
    while not rospy.is_shutdown():
        pass

rospy.Subscriber('fromarduino', CANFrameData, callback) #for incoming packets

if __name__ == '__main__':
    try:
        main_loop()
    #if Ros can't start erease an exception
    except rospy.ROSInterruptException:
        print("ROSInterruptException occured..exit")
        sys.exit()
