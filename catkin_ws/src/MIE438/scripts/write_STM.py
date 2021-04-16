#!/usr/bin/env python3
import rospy
import serial.tools.list_ports
import time
from collections import deque
import numpy as np
from std_msgs.msg import String, UInt16MultiArray
import sys, select, os, serial

# Custom Modules
from motor_util import Hermes
if os.name == 'nt':
    import msvcrt
else:
    import tty, termios

class bp_communicator_write():
    def __init__(self):
        # Publishers and subscribers
        self.command_sub = rospy.Subscriber('command', UInt16MultiArray, self.command_callback, queue_size=1)

        # For writing back to BP
        self.port = None
        while not self.port:
            for port in serial.tools.list_ports.comports():
                if port[1] == "STM32 Virtual ComPort":
                    self.port = port[0]
            if not self.port:
                print("ERROR: Waiting for port.")
        print(self.port)
        self.command = Hermes(self.port)

        # For negotiating speed:
        self.l_dir = -1
        self.r_dir = -1
        self.l_speed = 0
        self.r_speed = 0

        # Other attributes
        self.freq = 10
        self.rate = rospy.Rate(self.freq)
        rospy.sleep(1)

    def command_callback(self, data):
        # Receives length 4 array: [L Dir, R Dir, Left PWM Speed, Right PWM Speed]
        self.l_dir = data.data[0]
        self.r_dir = data.data[1]
        self.l_speed = data.data[2]
        self.r_speed = data.data[3]
        self.write()                # on new message we update mconfig
        return True

    def write(self):
        self.command.write_mconfig(self.l_dir, self.r_dir, self.l_speed, self.r_speed)
        return True

if __name__ == "__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('MIE438_Actuation')
    bp_comm = bp_communicator_write()
    try:
        while (1):
           bp_comm.write()
    except rospy.ROSInterruptException:
        print("comm failed")
