'''
Luigi Di Marcantonio
bus_spi.py
Università Politecnica delle Marche

Works in combo with the bus_spam.py script. This one continuosly sniff
can frame from the CAN network vcan 0 and then send info about them to the
serial port where arduino is attached. Before that it send information
about time for the timestamp processing (see reading_sketch.ino for that)

19-03-2019
'''
# import librerie
import can
import struct
import serial
import datetime

# create bus comunication with the network
can_interface = 'vcan0'
bus = can.interface.Bus(can_interface, bustype='socketcan')

if __name__ == '__main__':

    ser = serial.Serial('/dev/ttyUSB0', 9600)

    #get sys current time
    now = datetime.datetime.now()

    #send that to arduino sketch trough serial
    ser.write(struct.pack('>B', now.second))
    ser.write(struct.pack('>B', now.minute))
    ser.write(struct.pack('>B', now.hour))
    ser.write(struct.pack('>B', now.day))
    ser.write(struct.pack('>B', now.month))

    while True:
        message = bus.recv() #è bloccante

        # we don't need to get timestamp there
        print (message) # print it only for debug purpose
        ser.write(b'n') # new frame it's going to be sended
        ser.write(struct.pack('i', message.arbitration_id)) # 4 bytes
        ser.write(struct.pack('>B', message.dlc)) # 1 byte

        #tipo bytearray quindi a posto così (non serve il struct.pack)
        ser.write(message.data)
