"""Module to manage distance Sensor"""
#!/usr/bin/env python3
#for conversions
from array import array
import struct
import rospy
#data types definition
from std_msgs.msg import Float32
from agrosbuspkg.msg import CANFrameData #info about packet from arduino

class Radar:
    """Class to manage distance Sensor"""
    def __init__(self, topic_name, pow_data, q_size, pub_to_can):
        self.topic_name = topic_name
        self.pow_value = pow_data
        self.queue_size = q_size
        #outgoing messages
        self.pub = rospy.Publisher(topic_name, Float32, queue_size=self.queue_size)
        self.pub_to_can = pub_to_can
        #depending on witch data type should turn back from sensor logic
        rospy.Subscriber(topic_name + "_back", Float32, self.radar_callback) #for outgoing packets
    def radar_callback(self, data):
        """Method to manage data from other ROS nodes"""
        #build CANFrameData and then send to CAN-bus bridge
        #self.pub_to_can.publish(data)

    def sendpacket(self, sensor_value_tuple):
        """Method to send converted value to listeners"""
        value_to_send = self.convert(sensor_value_tuple)
        self.pub.publish(value_to_send)

    #each sensor have to provide a proper conversion function
    def convert(self, tup):
        """Method to convert payload into a value with a well-known ROS datatype"""
        data = array('b', tup)
        value = struct.unpack('<h', data)
        #return the right value with the right pow conversion
        return value[0] / pow(10, self.pow_value)
