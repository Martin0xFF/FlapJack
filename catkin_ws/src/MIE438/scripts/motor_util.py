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
        self.ser = serial.Serial(port, baudrate, timeout=5)    # attempt to open serial comm
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
        self.frames.appendleft(self._parse_sensor_frame(self.ser.read(20)))

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
        self.write_mconfig(-1,-1, 0, 0)


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

def convert_counts_to_duty(counts):
    '''
    counts, counts Pos for forward Neg for back
    '''
    
    return abs(int((counts - 14)*462.8 + 10000))

def motor_control(port, target):
    import time
    aprime = Hermes(port)
    target_counts_per_sec = target
    print(target)
    kpl = 600
    kil = 1250
    kdl = 25
      
    kpr = 600
    kir = 1250
    kdr = 25

    integral_cutoff =1000
    eprevl = 0
    eprevr = 0
    elacum = 0
    eracum = 0 
    lefterror=0
    righterror=0
    aprime.send_command('n')
    aprime.ser.flushInput()
    while True:
        s = time.time()
        
        while(True):
            start = time.time()
            datum = aprime.extract()
            delta = time.time() - start

            eprevl = lefterror
            eprevr = righterror
            
            lefterror =  target_counts_per_sec - datum['encoderleft']
            righterror = target_counts_per_sec - datum['encoderright']
            
            
            elacum += delta*lefterror
            if elacum > 0:
                elacum = min(elacum, integral_cutoff)
            else:
                elacum = max(elacum, -integral_cutoff)
            
            eracum += delta*righterror
            if eracum > 0:
                eracum = min(eracum, integral_cutoff)
            else:
                eracum = max(eracum, -integral_cutoff)
            print(f"{datum['encoderleft']}:{datum['encoderright']}: Left control {int( kpl*lefterror + kil*elacum + kdl*((eprevl - lefterror)/delta)) }")
            lcontrol = min(convert_counts_to_duty(target_counts_per_sec) + int( kpl*lefterror + kil*elacum + kdl*((eprevl - lefterror)/delta)),65000)
            rcontrol = min(convert_counts_to_duty(target_counts_per_sec) + int( kpr*righterror + kir*eracum + kdr*((eprevr - righterror)/delta)),65000)

            aprime.write_mconfig(1,1, lcontrol, rcontrol)
        s = time.time()
        while(time.time()-s< 3):
            datum = aprime.extract()
            lefterror = datum['encoderleft'] - 1
            righterror = datum['encoderright'] - 1
            print(f"{datum['encoderleft']}:{datum['encoderright']}")
            aprime.write_mconfig(1,1, kpl*lefterror, kpr*righterror)      

   
def mpu_convert(gyro_val, scale=0):
    '''
    assume 250 deg/s
    '''
    if abs(gyro_val)< 200:
        gyro_val = 0
    else:
        gyro_val = gyro_val - 300
    return (gyro_val - 32767.5)/131.07 + 250  
 
def reset(port):
    aprime = Hermes(port)
    while True:
        aprime.send_command('r')


def control_test(port):
    """
    Tilt Test
    Front of bot is Pos x
    Right side of bot is Pox y
    Bottom is Pox Z


    Can create angle about wheel using ax and az   
    """
    import numpy as np
    import time as t
    
    aprime = Hermes(port)
    ref = 0
    kp = 4000
    ki = 100
    kd = 200#200# 65535# 65535//2
    prev_angle = 0;
    current_angle = 0;
    
    prev_error  = 0;
    acum_error = 0;
    error = 0;
    alpha=0.80
    aprime.send_command('n')
    datum = aprime.extract()
        
    xoffset = datum['ax']
    while True:
        start = t.time()
        datum = aprime.extract()
        delta = t.time() - start 
        prev_angle = current_angle
        current_angle =  alpha*(prev_angle + delta*mpu_convert(datum["gy"])) + (1-alpha)*(-np.rad2deg(np.arctan((datum['ax'] - xoffset)/datum['az'])))
        
        prev_error = error
        error = current_angle - ref
        acum_error += error
        if abs(acum_error) > 1000:
            acum_error = 1000 if acum_error>0 else -1000 
        control = int(kp*error + ki*(acum_error) + kd*(error - prev_error)/delta) 
        sign = control < 0
        print(f"angle:{error:6.1f}: control:{control:09d}", end="\r", flush=True)
        control = min(abs(control), 60000)
        if sign:
            aprime.write_mconfig(1,1, abs(control), abs(control));
        else:
            aprime.write_mconfig(0,0, abs(control), abs(control));                  
    
    

if __name__ == "__main__":
    import argparse as ap
    parser = ap.ArgumentParser()
    parser.add_argument('--port','-p', help= "Name of the comport we are trying to establish a serial connection with (i.e. /dev/ttyACM0)", required=True)
    parser.add_argument('--motor_test', '-m', action='store_true')
    parser.add_argument('--sensor_test', '-s', action='store_true')
    parser.add_argument('--control_test', '-c', action='store_true')
    parser.add_argument('--speed_test', '-sp', action='store_true')
    parser.add_argument('--target','-t') 
    parser.add_argument('--reset', '-r', action='store_true')
 

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
    if args.reset:
        reset(args.port)
 
    if args.control_test:
        control_test(args.port)
    if args.speed_test:
        motor_control(args.port,int(args.target))
