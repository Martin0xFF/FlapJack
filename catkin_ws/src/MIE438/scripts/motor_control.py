#!/usr/bin/env python3
import rospy
import serial.tools.list_ports
import time
from collections import deque
import numpy as np
from std_msgs.msg import String, UInt16MultiArray, Float32MultiArray
import sys, select, os, serial
import math

# Custom Modules
from motor_util import convert_duty_to_counts
if os.name == 'nt':
    import msvcrt
else:
    import tty, termios

# Feeds control back to write node
class motor_control():
    def __init__(self):
        # Publishers and subscribers
        self.target_speed = rospy.Subscriber('target_speed', UInt16MultiArray, self.target_speed_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber('odom', Float32MultiArray, self.odom_callback, queue_size=1)
        self.command_pub = rospy.Publisher('command', UInt16MultiArray, queue_size=1)
        self.command = UInt16MultiArray()

        # Subscribed data
        self.l_dir = -1
        self.r_dir = -1
        self.target_speed_l = 0
        self.target_speed_r = 0
        self.odom_l = 0
        self.odom_r = 0
        self.delta = 0.000001

        # Other inits
        self.start = 0

        self.kpl = 50
        self.kil = 0
        self.kdl = 0.5

        self.kpr = 50
        self.kir = 0
        self.kdr = 0.5

        self.integral_cutoff = 10000
        self.eprevl = 0
        self.eprevr = 0
        self.elacum = 0
        self.eracum = 0
        self.lefterror = 0
        self.righterror = 0

        self.target_counts_l = None
        self.target_counts_r = None
        self.actual_speed_l = None
        self.actual_speed_r = None


    def target_speed_callback(self, data):
        # Receives length 4 array: [L Dir, R Dir, Left Target Speed, Right Target Speed]
        self.l_dir = data.data[0]
        self.r_dir = data.data[1]
        self.target_speed_l = data.data[2]
        self.target_speed_r = data.data[3]
        return True

    def odom_callback(self, data):
        # Receives length 3 array: [Encoder Left, Encoder Right, Delta]
        self.odom_l = data.data[0]
        self.odom_r = data.data[1]
        self.delta = data.data[2]
        return True

    def write(self):
        if self.target_speed_l == 0 and self.target_speed_r == 0:
            self.command.data = [self.l_dir, self.r_dir, self.target_speed_l, self.target_speed_r]
            print(self.command.data)
            self.command_pub.publish(self.command)

        self.start = time.time()

        self.eprevl = self.lefterror
        self.eprevr = self.righterror

        self.lefterror = self.target_speed_l - (self.odom_l/self.delta)
        self.righterror = self.target_speed_r - (self.odom_r/self.delta)

        self.elacum += self.delta * self.lefterror

        if self.elacum > 0:
            self.elacum = min(self.elacum, self.integral_cutoff)
        else:
            self.elacum = max(self.elacum, -self.integral_cutoff)

        self.eracum += self.delta * self.righterror

        if self.eracum > 0:
            self.eracum = min(self.eracum, self.integral_cutoff)
        else:
            self.eracum = max(self.eracum, -self.integral_cutoff)

        self.actual_speed_l = max(min(int(self.kpl * self.lefterror + self.kil * self.elacum + self.kdl *
            ((self.eprevl - self.lefterror) / self.delta)), 65000), 0)
        self.actual_speed_r = max(min(int(self.kpr * self.righterror + self.kir * self.eracum + self.kdr *
            ((self.eprevr - self.righterror) / self.delta)), 65000), 0)

        # Sends length 4 array: [L Dir, R Dir, Left PWM Speed, Right PWM Speed]
        self.command.data = [self.l_dir, self.r_dir, self.actual_speed_l, self.actual_speed_r]
        print(f"Current speed | L: {((self.odom_l/self.delta)*math.pi*0.08)/240} | R: {((self.odom_r/self.delta)*math.pi*0.08)/240}")
        self.command_pub.publish(self.command)

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
