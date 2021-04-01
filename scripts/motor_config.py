
import serial
from collections import deque

class Hermes():
    '''
    Messager between computer and STM32 via serial
    '''
    def __init__(self, port=None, baudrate=115200, sensor_frame_size=20):
        self.frames = deque(maxlen=5)                          # we would only like topical data
        if port is None:
            raise("Must Provide Serial port for Hermes object")
        self.ser = serial.Serial(port, baudrate, timeout=None)    # attempt to open serial comm
        self.sfs = sensor_frame_size

    def send_command(self, specifier=None, byte_arr=b''):
            if specifier is None:
                raise("Specifier for send_command not provided")
            self.ser.write(bytes(specifier, 'utf-8') + byte_arr)
    
    def _read_into_queue(self,): 
        '''
        Read latest sensor reading into frames queue
        We utilize a queue here just in case 
        '''
        self.frames.appendleft(self._parse_sensor_frame(self.ser.read(20))) # multithread this part so we just readinto a queue that other code will access

    def _extract_from_queue(self,):
        '''
        Pull the latest value from the frames queue
        '''
        return self.frames.pop()

    def _parse_sensor_frame(self, read_bytes):
        to_parse = ('ax', 'ay', 'az', 'temperature', 
                    'gx', 'gy', 'gz','range','encoderleft', 'encoderright')
        out = {}  
        for i, element in enumerate(to_parse):
            out[element] = int.from_bytes(read_bytes[i*2:i*2+2],'big', signed=True)
        return out

    def flush_input(self,):
        self.ser.reset_input_buffer()

    def extract(self, n=1):
        '''
        try to read n and extract
        '''
        self._read_into_queue()
        return self._extract_from_queue()

    def display(self, sensor_data):
        for key in sensor_data:
            print(f"{key}:{sensor_data[key]}",end='\r\n')
        print('', end='\r',flush=True)
           

            
        
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

          
        
    
    def __del__(self):
        '''
        when we are done with the serial com, want to free it
        '''
        if self.ser:
            self.ser.close()


def sensor_test(port):
    aprime = Hermes(port)
    while True:
        aprime.send_command('n')
        aprime.display(aprime.extract())

def motor_test(port):
    import time
    aprime = Hermes(port) 
    while True:
        s = time.time()
        while(time.time()-s< 3):
            aprime.write_mconfig(1,1, 65535, 15000)
        s = time.time()
        while(time.time()-s< 3):
            aprime.write_mconfig(1,0, 5000, 10000)       

def convert_duty_to_counts(duty):
    '''
    Duty cycle, uint16_t max 65535, min 10000
    '''
    return int((duty - 10000)/462.8 + 14)

def motor_control(port):
    import time
    aprime = Hermes(port)
    target_counts_per_sec = 20
    kpl = 7500
    kpr = 7500
    kil = 1
    kir = 1
    eprevl = 0
    eprevr = 0
    elacum = 0
    eracum = 0 
    lefterror=0
    righterror=0
    while True:
        s = time.time()
        while(True):
            datum = aprime.extract()
            lefterror = datum['encoderleft'] - 20
            righterror = datum['encoderright'] - 20
            print(f"{datum['encoderleft']}:{datum['encoderright']}")
            elacum += lefterror
            aprime.write_mconfig(1,1, 65535, 10000)
        s = time.time()
        while(time.time()-s< 3):
            datum = aprime.extract()
            lefterror = datum['encoderleft'] - 1
            righterror = datum['encoderright'] - 1
            print(f"{datum['encoderleft']}:{datum['encoderright']}")
            aprime.write_mconfig(1,1, kpl*lefterror, kpr*righterror)      

   
def control_test(port):
    """
    Tilt Test
    Front of bot is Pos x
    Right side of bot is Pox y
    Bottom is Pox Z


    Can create angle about wheel using ax and az   
    """
    import numpy as np
    aprime = Hermes(port, 256000)
    ref = -0.02
    kp = 2*65535
    ki = 0#200
    kd =0# 65535# 65535//2
    prev_error  = 0;
    acum_error = 0;
    error = 0;
    while True:
        theta = 0
        for i in range(10):
            datum = aprime.extract()
            theta += datum['ax']/datum['az']
        theta = theta/10
        prev_error = error
        error = theta - ref
        acum_error += error
        print(f"angle:{error}", end="\r", flush=True)
        control = round(kp*error + min(ki*(acum_error), 30000) + kd*(error - prev_error)) 
        if control > 0:
            aprime.write_mconfig(1,1, control, control);
        else:
            aprime.write_mconfig(0,0, -control, -control);                  
    
    

if __name__ == "__main__":
    import argparse as ap
    parser = ap.ArgumentParser()
    parser.add_argument('--port','-p', help= "Name of the comport we are trying to establish a serial connection with (i.e. /dev/ttyACM0)", required=True)
    parser.add_argument('--motor_test', '-m', action='store_true')
    parser.add_argument('--sensor_test', '-s', action='store_true')
    parser.add_argument('--control_test', '-c', action='store_true')
    parser.add_argument('--speed_test', '-sp', action='store_true')
 


    args = parser.parse_args()

    '''
    For now we just assume standard serial config
    no parity check
    1 stop bit
    8 bit size data
    '''
    if args.motor_test:
        motor_test(args.port)
    if args.sensor_test:
        sensor_test(args.port)
    if args.control_test:
        control_test(args.port)
    if args.speed_test:
        motor_control(args.port)
