#!/usr/bin/env python3
import rospy
#for conversions
from array import array
import struct
#data types definition
from std_msgs.msg import Float32
from agrosbuspkg.msg import RosInfo

#what to do with data coming back
def radar_callback(data):
    #when the radar logic send back something than
    #this callback decide how to deal with it and
    #eventually it can create a RosInfo() to send
    #a packet back to the Ros Network
    packet_backToCan = RosInfo()
    pub.publish(pacchetto_backtoCan)
    pass

class Radar:
    #defining attributes for each sensor, a RosInfo to send things back
    #to arduino and two pointers (topic) to communicate with sensor logic
    #script
    def __init__(self, topic_name, pow, q_size, fromros_pointer):
        self.topic_name = topic_name
        self.pow_value = pow
        self.queue_size = q_size
        self.radar_ToCANpkt = RosInfo() #packet to fill and sand back to arduino from topic "fromros

        self.pub_radar = rospy.Publisher(topic_name, Float32,queue_size=self.queue_size) #for outgoing packets
        #depending on witch data type should turn back from sensor logic
        rospy.Subscriber(topic_name + "_back", Float32, radar_callback) #for outgoing packets

    #when a packet for the sensor arrive from the topic "fromarduino"
    #this method stars and send immediatly to the sensor logic
    def sendpacket(self, sensor_value_tuple):
        value_to_send = self.convert(sensor_value_tuple)
        self.pub_radar.publish(value_to_send)

    #each sensor have to provide a proper conversion function
    def convert(self,tup):
        bytes = array('b', tup)
        value = struct.unpack('<h', bytes)
        #return the right value with the right pow conversion
        return (value[0] / pow(10, self.pow_value))
