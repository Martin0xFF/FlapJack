#!/usr/bin/env python3
import rospy
import serial.tools.list_ports
import time
from collections import deque
import numpy as np
from std_msgs.msg import String
import sys, select, os, serial

# Custom Modules
from motor_util import Hermes

if os.name == 'nt':
    import msvcrt
else:
    import tty, termios

e = """
Communications Failed
"""

def getKey():
    if os.name == 'nt':
        return ord(msvcrt.getch())

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = ord(sys.stdin.read(1))
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

"""
OPERATOR KEYS:

W - Forward
A - Turn Left
S - Backward
D - Turn Right
E - Stop

I - Speed up
K - Slow Down

"""

class bp_communicator_write():
    def __init__(self):

        # For writing back to BP
        self.port = None
        while not self.port:
            for port in serial.tools.list_ports.comports():
                if port[1] == "STM32 Virtual ComPort":
                    self.port = port[0]
            if not self.port:
                print("ERROR: Waiting for port.")
        self.key = None
        self.command = Hermes(self.port)

        # For negotiating speed:
        self.l_dir = 1
        self.r_dir = 1
        self.l_speed = 0
        self.r_speed = 0

        # Other attributes
        self.freq = 10
        self.rate = rospy.Rate(self.freq)
        rospy.sleep(1)

    def write(self):
        print("Enter a command.")
        while self.key is None or self.key=='':
            self.key = getKey()    
        if self.key == 87 or self.key == 119: # W (Forward)
            print("Moving forward...")
            self.l_dir = 1
            self.r_dir = 1
            #self.l_speed = 20000
            #self.r_speed = 20000
        elif self.key == 65 or self.key == 97: # A (Turn Counterclockwise)
            print("Turning counterclockwise...")
            self.l_dir = 0
            self.r_dir = 1
            #self.l_speed = 20000
            #self.r_speed = 20000
        elif self.key == 83 or self.key == 115: # S (Backward)
            print("Moving backwards...")
            self.l_dir = 0
            self.r_dir = 0
            #self.l_speed = 20000
            #self.r_speed = 20000
        elif self.key == 69 or self.key == 101: # E (Stop)
            print("Stopping...")
            self.l_dir = 1
            self.r_dir = 1
            self.l_speed = 0
            self.r_speed = 0
        elif self.key == 68 or self.key == 100: # D (Turn Clockwise)
            print("Turning clockwise...")
            self.l_dir = 1
            self.r_dir = 0
            #self.l_speed = 20000
            #self.r_speed = 20000
        elif self.key == 73 or self.key == 105: # I (Speed Up)
            print("Speeding up...")
            self.l_speed += 5000
            self.r_speed += 5000
            if self.l_speed >= 65000:
                self.l_speed = 65000
            elif self.r_speed >= 65000:
                self.r_speed = 65000
        elif self.key == 75 or self.key == 107: # K (Slow Down)
            print("Slowing down...")
            self.l_speed -= 5000
            self.r_speed -= 5000
            if self.l_speed <= 0:
                self.l_speed = 2500
            elif self.r_speed <= 0:
                self.r_speed = 2500
        elif self.key == ord('\x03'):
            return False
        else:
            True
        self.command.write_mconfig(self.l_dir, self.r_dir, self.l_speed, self.r_speed)
        self.key = None
        return True

if __name__ == "__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('MIE438_Teleop')
    bp_comm = bp_communicator_write()
    try:
        print(" FLAPJACK TELE-OPERATION MODE \n")
        print(" Controls: \n")
        print(" W - Forward \n")
        print(" A - Turn Counterclockwise \n")
        print(" S - Backward \n")
        print(" D - Turn Clockwise \n")
        print(" E - Stop \n\n")
        print(" I - Speed Up \n")
        print(" K - Slow Down \n")
        while (1):
           run = bp_comm.write()
           if not run:
                break
    except rospy.ROSInterruptException:
        print("comm failed")
