#!/usr/bin/env python3
import rospy
import serial.tools.list_ports
import time
from collections import deque
import numpy as np
from std_msgs.msg import String, Float64MultiArray
import sys, select, os, serial

# Custom Modules
from motor_util import Hermes
if os.name == 'nt':
    import msvcrt
else:
    import tty, termios

class bp_communicator_read():
    def __init__(self):
        # Publishers and subscribers
        self.odom_pub = rospy.Publisher('odom', Float64MultiArray, queue_size=1)
        self.odom = Float64MultiArray()

        # For writing back to BP
        self.port = None
        while not self.port:
            for port in serial.tools.list_ports.comports():
                if port[1] == "STM32 Virtual ComPort":
                    self.port = port[0]
            if not self.port:
                print("ERROR: Waiting for port.")
        self.reading = Hermes(self.port)

        # Other attributes
        self.start = 0
        self.data = None
        self.freq = 10
        self.rate = rospy.Rate(self.freq)
        rospy.sleep(1)

    def read(self):
        
        self.start = time.time()
        self.data = self.reading.extract()

        # Sends length 3 array: [Encoder Left, Encoder Right, Delta]
        self.odom.data = [to_rad(self.data['encoderleft']), to_rad(self.data['encoderright']), time.time()-self.start]
        self.odom_pub.publish(self.odom)
        return True

def to_rad(counts_per_message):
    '''
    convert the counts per message to radiand per message
    '''
    return counts_per_message*(np.pi/120)    

if __name__ == "__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('MIE438_Odom')
    bp_comm = bp_communicator_read()
    try:
        bp_comm.reading.send_command('n')
        while (1):
           bp_comm.read()
    except rospy.ROSInterruptException:
        print("comm failed")
