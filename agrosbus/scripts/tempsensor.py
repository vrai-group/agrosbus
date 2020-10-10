#!/usr/bin/env python3
import rospy
from array import array
import struct
from std_msgs.msg import Int16

class Temperature:
    def __init__(self, topic_name, pow, q_size):
        self.topic_name = topic_name
        self.pow_value = pow
        self.queue_size = q_size
        self.pub_radar = rospy.Publisher(topic_name, Int16 ,queue_size=self.queue_size) #for outgoing packets

    def sendpacket(self, sensor_value_tuple):
        value_to_send = int(self.convert(sensor_value_tuple))
        self.pub_radar.publish(value_to_send)

    def convert(self,tup):
        bytes = array('b', tup)
        value = struct.unpack('<h', bytes)
        #return the right value with the right pow conversion
        return (value[0] / pow(10, self.pow_value))
