'''
Luigi Di Marcantonio
bus_spam.py
Università Politecnica delle Marche

generate random CAN frame and send them one by one on a socketcan
the socketcan is binded to a virtual CAN network created in commandline
before executing this script. Run the file start.sh (shell script) in order
to start the vcan0

then check created vcan0 using 'ifconfig' command

16-09-2019
'''
# import librerie
from time import sleep
import random
import struct
import sys
import socket

from collections import namedtuple
Constants = namedtuple('Constants', ['rate', 'device_excursion', 'total','group_dim','fmt'])
constants = Constants(1.00, 65535, 2048, 256, "<IB3x8s")



def produce_randomID(count):
    # simuliamo l'id del mittente del frame
    #2^11 - 1 perchè CAN2.0A 11 bit for arbitration field
    rand_arb_id = random.randrange(0, constants.total)
    n, dev = (rand_arb_id// constants.group_dim, rand_arb_id% constants.group_dim)

    # inizio a stampare informazioni
    print(count,': Message from: arbid=',rand_arb_id,', network:',n,' , device:',dev,)

    return rand_arb_id




if __name__ == '__main__':
    #informazioni sul passaggio di parametri [per il packing]
    if (sys.argv[1] == 'help'):
        print ("""Use h/H (2 BYTES) \n
		i/I/l/L/f (4 BYTES) \n
		q/Q/d (8 BYTES) \n
		like first argument for this script
		for data field formatting -> affecting payload""")
        sys.exit(0)



    # dopo aver creato il virtual CAN network [sul terminale] creo una socket per comunicare
    sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW) # creo la socket
    interface = "vcan0" # nome dell'interfaccia
    try:
        sock.bind((interface,)) # la collego alla CAN network
    except OSError:
        # problemi nel collegamento... esco
        sys.stderr.write("Could not bind to interface '%s'\n" % interface)
        sys.exit(-1)



    #collegamento effettuato..inizio a inviare frame
    count = 0
    print ("Starting generating CAN frames...\n\n")
    sleep(2.00)

    while (1):

        value = random.randint(0, constants.device_excursion) # emulo il valore di un sensore (ANALOG)
        encoded_value = bytearray(struct.pack(str(sys.argv[1]), value))
        # lo impacchetto [dipendentemente da sys.argv[1] e lo trasformo in un array di byte 

        # genero un id per l'arbitration_field del pacchetto
        # count lo uso per contare il numero di pacchetti dall'inizio dello stream
        r_id = produce_randomID(count)

        # continuo a stampare informazioni sul pacchetto
        print ('val: ',value,'payload: ',len(encoded_value))

        # costruisco effettivamente il pacchetto
        can_pkt = struct.pack(constants.fmt, r_id, len(encoded_value), encoded_value)
        sock.send(can_pkt)

        count += 1
        sleep(constants.rate)
