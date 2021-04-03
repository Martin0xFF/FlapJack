#!/usr/bin/env python3
import rospy
import serial.tools.list_ports
import time
from collections import deque
import numpy as np
from std_msgs.msg import String
import sys, select, os, serial

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

class Hermes():
    '''
    Messager between computer and STM32 via serial
    '''

    def __init__(self, port=None, baudrate=115200, sensor_frame_size=20):
        self.frames = deque(maxlen=5)  # we would only like topical data
        if port is None:
            raise ("Must Provide Serial port for Hermes object")
        self.ser = serial.Serial(port, baudrate, timeout=None)  # attempt to open serial comm
        self.sfs = sensor_frame_size

    def send_command(self, specifier=None, byte_arr=b''):
        if specifier is None:
            raise ("Specifier for send_command not provided")
        self.ser.write(bytes(specifier, 'utf-8') + byte_arr)

    def _read_into_queue(self, ):
        '''
        Read latest sensor reading into frames queue
        We utilize a queue here just in case
        '''
        self.frames.appendleft(self._parse_sensor_frame(
            self.ser.read(20)))  # multithread this part so we just readinto a queue that other code will access

    def _extract_from_queue(self, ):
        '''
        Pull the latest value from the frames queue
        '''
        return self.frames.pop()

    def _parse_sensor_frame(self, read_bytes):
        to_parse = ('ax', 'ay', 'az', 'temperature',
                    'gx', 'gy', 'gz', 'range', 'encoderleft', 'encoderright')
        out = {}
        for i, element in enumerate(to_parse):
            out[element] = int.from_bytes(read_bytes[i * 2:i * 2 + 2], 'big', signed=True)
        return out

    def flush_input(self, ):
        self.ser.reset_input_buffer()

    def extract(self, n=1):
        '''
        try to read n and extract
        '''
        self._read_into_queue()
        return self._extract_from_queue()

    def display(self, sensor_data):
        for key in sensor_data:
            print(f"{key}:{sensor_data[key]}", end='\r\n')
        print('', end='\r', flush=True)

    def write_mconfig(self, l_dir, r_dir, l_speed=0, r_speed=0):
            '''
            l_dir - direction of the left motor, 1 for forward, 0 for back, anything else is stop
            r_dir - direction of the right motor, 1 for forward, 0 for back, anything else is stop

            l_speed - desired angular speed of the motor (as duty cycle)
            r_speed - desired angular speed of the motor (as duty cycle) (0-65535)
            '''

            ctrl_byte = 0x00 

            if l_dir == 0:
                ctrl_byte = 0b00000000 # Left Motor is on and going backwards        
            elif l_dir == 1:
                ctrl_byte = 0b00000010 # Left Motor is on and going forwards
            else:
                ctrl_byte = 0b00001000 # Left Motor is off DGAF about direction

            if r_dir == 0:
                ctrl_byte |= 0b00000000 # Right Motor is on and going backwards        
            elif r_dir == 1:
                ctrl_byte |= 0b00000001 # Right Motor is on and going forwards
            else:
                ctrl_byte |= 0b00000100 # Right Motor is off DGAF about direction
            self.send_command('m', bytearray([ctrl_byte, l_speed>>8&0xff, l_speed&0xff,r_speed>>8&0xff, r_speed&0xff]))

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
            self.l_speed = 20000
            self.r_speed = 20000
        elif self.key == 65 or self.key == 97: # A (Turn Counterclockwise)
            print("Turning counterclockwise...")
            self.l_dir = 0
            self.r_dir = 1
            self.l_speed = 20000
            self.r_speed = 20000
        elif self.key == 83 or self.key == 115: # S (Backward)
            print("Moving backwards...")
            self.l_dir = 0
            self.r_dir = 0
            self.l_speed = 20000
            self.r_speed = 20000
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
            self.l_speed = 20000
            self.r_speed = 20000
        elif self.key == 73 or self.key == 105: # I (Speed Up)
            print("Speeding up...")
            self.l_speed += 5000
            self.r_speed += 5000
            if self.l_speed >= 65535:
                self.l_speed = 65000
            elif self.r_speed >= 65535:
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
