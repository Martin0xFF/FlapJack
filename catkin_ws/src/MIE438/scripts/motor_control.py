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

# Feeds control back to write node
class motor_control():
    def __init__(self):
        # Subscriber
        self.cmd_pub = rospy.Publisher('motor_ctrl', String, queue_size=1)
        self.command = String()

    def write(self):
        self.command.data =
        self.cmd_pub.publish(self.command)

# Main code
if __name__ == "__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('MIE438_MotorCtrl')
    motor_ctrl = motor_control()
    try:
        while (1):
           run = motor_ctrl.write()
    except rospy.ROSInterruptException:
        print("comm failed")