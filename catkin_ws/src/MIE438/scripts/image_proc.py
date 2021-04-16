#!/usr/bin/env python3
import rospy
import serial.tools.list_ports
import time
from collections import deque
import numpy as np
from std_msgs.msg import String, Float64MultiArray, UInt16MultiArray
from sensor_msgs.msg import Image, CameraInfo
import sys, select, os, serial
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Custom Modules
from motor_util import Hermes

if os.name == 'nt':
    import msvcrt
else:
    import tty, termios


class image_proc():
    def __init__(self):
        # Publishers and subscribers
        self.command_pub = rospy.Publisher('command', UInt16MultiArray, queue_size=1)
        # self.caminfo_sub = rospy.Subscriber('camera_info', CameraInfo, self.caminfo_callback, queue_size=1)
        self.image_sub = rospy.Subscriber('cv_camera/image_raw', Image, self.image_callback, queue_size=1)
        self.command = UInt16MultiArray()

        # Other attributes
        self.image = None
        self.gray = None
        self.circles = None
        self.speed = None
        self.processed = None
        self.freq = 10
        self.rate = rospy.Rate(self.freq)
        rospy.sleep(1)

    def image_callback(self, data1):
        try:
            self.image = CvBridge().imgmsg_to_cv2(data1, "bgr8")
        except CvBridgeError as e:
            print(e)

    def basic(self):
        # Hough Circles requires grayscale input
        print(self.image.shape)
        self.gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        self.circles = cv2.HoughCircles(self.gray, cv2.HOUGH_GRADIENT, 1.2, 100)
        if self.circles is not None:
            print("I see a circle!! :)")
            out = self.circles[0, 0, :]
            print(out)
            if out[2] > 200:
                print("Too close!! :o")
                self.command.data = [0, 0, 0, 0]
            else:
                self.speed = int(3000000 / out[2])
                if self.speed > 30000:
                    self.speed = 30000
                elif self.speed < 0:
                    self.speed = 0
                print(self.speed)
                self.command.data = [0, 0, self.speed, self.speed]
        else:
            print("No circle :(")
            self.command.data = [1, 1, 0, 0]
        self.command_pub.publish(self.command)
        return True

    def tracking(self):
        # Hough Circles requires grayscale input
        print(self.image.shape)
        self.gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        self.circles = cv2.HoughCircles(self.gray, cv2.HOUGH_GRADIENT, 1.2, 100)
        if self.circles is not None:
            print("I see a circle!! :)")
            out = self.circles[0, 0, :]
            print(out)
            if out[2] > 200:
                print("Too close!! :o")
                self.command.data = [0, 0, 0, 0]
            else:
                self.speed = int(1500000 / out[2])
                if self.speed > 20000:
                    self.speed = 20000
                elif self.speed < 0:
                    self.speed = 0

                wheel_diff, dir = self.find_diff(self.image, out)

                print(self.speed+wheel_diff, self.speed-wheel_diff)

                wheel_diff/=2

                if dir == 'left':
                    self.command.data = [0, 0, int(self.speed+wheel_diff), int(self.speed-wheel_diff)]
                elif dir == 'right':
                    self.command.data = [0, 0, int(self.speed-wheel_diff), int(self.speed+wheel_diff)]
                else:
                    self.command.data = [0, 0, self.speed, self.speed]
        else:
            print("No circle :(")
            self.command.data = [1, 1, 0, 0]
        self.command_pub.publish(self.command)
        return True

    def find_diff(self, image, circle):
        # Find relative horizontal position of circle:
        diff = (image.shape[1]/2) - circle[0]

        # Find the relative speeds of the wheels
        wheel_diff = abs(diff)*50

        # Cap the diff
        if wheel_diff > 5000:
            wheel_diff = 5000

        # Find direction
        if diff < 0:
            dir = 'right'
        elif diff == 0:
            dir = 'center'
        else:
            dir = 'left'

        return wheel_diff, dir


if __name__ == "__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('MIE438_ImageProcessing')
    processor = image_proc()
    try:
        while (1):
            processor.tracking()
    except rospy.ROSInterruptException:
        print("comm failed")
