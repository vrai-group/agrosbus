#!/usr/bin/env python3
"""Module to manage Humidity Sensor"""
import struct
from array import array
import rospy
from std_msgs.msg import Int16
class Humidity:
    """Class to manage Humidity Sensor"""
    def __init__(self, topic_name, pow_value, q_size):
        self.topic_name = topic_name
        self.pow_value = pow_value
        self.queue_size = q_size
        self.pub = rospy.Publisher(topic_name, Int16, queue_size=self.queue_size)

    def sendpacket(self, sensor_value_tuple):
        """Method to send converted value to listeners"""
        value_to_send = int(self.convert(sensor_value_tuple))
        self.pub.publish(value_to_send)

    def convert(self, tup):
        """Method to convert payload into a value with a well-known ROS datatype"""
        data = array('b', tup)
        value = struct.unpack('<h', data)
        #return the right value with the right pow conversion
        return value[0] / pow(10, self.pow_value)
