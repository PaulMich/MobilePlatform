import os
import time
import sys
import serial
import re
import smbus
import math
import pigpio

#																													#
#										  	COMMUNICATION 		  							#
#																													#

# UART
bt_serial = serial.Serial(
  port='/dev/rfcomm0',
  baudrate = 38400,
  parity=serial.PARITY_NONE,
  stopbits=serial.STOPBITS_ONE,
  bytesize=serial.EIGHTBITS,
  timeout=1
)

while not bt_serial.isOpen():
  time.sleep(1)
  
# BLUETOOTH
f_bluetooth_mode = False
def BluetoothConnect():
  global f_bluetooth_mode
  check_ans = ''
  while check_ans[0:4] != 'BTOK':
    bt_serial.write('BTCK'.encode())
    check_ans = bt_serial.readline().decode()
    f_bluetooth_mode = True
    print(check_ans)

# I2C
I2C_CMD_START = 0xFF      # start command, indicates the begining of a new set of instructions
I2C_ADDR = 0x08               # bus address
i2c_bus = smbus.SMBus(1)      # indicates /dev/ic2-1

#																													#
#										  	MANIPULATOR       								#
#																													#

MAN_MAX_ANGLE_A = 135
MAN_MIN_ANGLE_A = 45
MAN_MID_ANGLE_A = (MAN_MAX_ANGLE_A - MAN_MIN_ANGLE_A) / 2

MAN_MAX_ANGLE_B = 140
MAN_MIN_ANGLE_B = 70
MAN_MID_ANGLE_B = (MAN_MAX_ANGLE_B - MAN_MIN_ANGLE_B) / 2

MAN_MAX_ANGLE_C = 130
MAN_MIN_ANGLE_C = 30
MAN_MID_ANGLE_C = (MAN_MAX_ANGLE_C - MAN_MIN_ANGLE_C) / 2

MAN_MAX_ANGLE_E = 158
MAN_MIN_ANGLE_E = 0
MAN_MID_ANGLE_E = (MAN_MAX_ANGLE_E - MAN_MIN_ANGLE_E) / 2

MAN_MAX_X = 160
MAN_MIN_X = -160

MAN_MAX_Y = 300
MAN_MIN_Y = 50

MAN_MAX_Z = 250
MAN_MIN_Z = 100

# UART 
man_serial = serial.Serial(
  port='/dev/ttyACM0',
  baudrate = 38400,
  parity=serial.PARITY_NONE,
  stopbits=serial.STOPBITS_ONE,
  bytesize=serial.EIGHTBITS,
  timeout=1
)
time.sleep(2)
    
    
class Manipulator:
  def __init__(self):
    self.angle_base = 90
    self.angle_arm1_base = 90
    self.angle_arm2_arm1 = 90
    self.angle_effector = 90
    
    self.pos_x = 0
    self.pos_y = 216
    self.pos_z = 230
    
Man = Manipulator()
 
man_angles_data = [0,0,0,0]
man_xyz_data = [0,0,0,0]
def ControllManipulator(mode):
  if mode == 'MA':
    
    Man.angle_base += man_angles_data[0]
    Man.angle_arm1_base += man_angles_data[1]
    Man.angle_arm2_arm1 += man_angles_data[2]
    
    if man_angles_data[3] == 0:
      Man.angle_effector = 0
    elif man_angles_data[3] == 1:
      Man.angle_effector = 170
      
    if Man.angle_base > MAN_MAX_ANGLE_A:
      Man.angle_base = MAN_MAX_ANGLE_A
    elif Man.angle_base < MAN_MIN_ANGLE_A:
      Man.angle_base = MAN_MIN_ANGLE_A
      
    if Man.angle_arm1_base > MAN_MAX_ANGLE_B:
      Man.angle_arm1_base = MAN_MAX_ANGLE_B
    elif Man.angle_arm1_base < MAN_MIN_ANGLE_B:
      Man.angle_arm1_base = MAN_MIN_ANGLE_B
      
    if Man.angle_arm2_arm1 > MAN_MAX_ANGLE_C:
      Man.angle_arm2_arm1 = MAN_MAX_ANGLE_C
    elif Man.angle_arm2_arm1 < MAN_MIN_ANGLE_C:
      Man.angle_arm2_arm1 = MAN_MIN_ANGLE_C
      
    man_cmd_str = 'CA'
    man_cmd_str += 'A'+str(Man.angle_base)
    man_cmd_str += 'B'+str(Man.angle_arm1_base)
    man_cmd_str += 'C'+str(Man.angle_arm2_arm1)
    man_cmd_str += 'E'+str(Man.angle_effector)
    man_cmd_str += '\n'
    man_serial.write(man_cmd_str.encode())
    print(man_cmd_str)
      
    
  elif mode == 'MC':    
    Man.pos_x += man_xyz_data[0]
    Man.pos_y += man_xyz_data[1]
    Man.pos_z += man_xyz_data[2]
    
    if man_angles_data[3] == 0:
      Man.angle_effector = 0
    elif man_angles_data[3] == 1:
      Man.angle_effector = 170
      
    if Man.pos_x > MAN_MAX_X:
      Man.pos_x = MAN_MAX_X
    elif Man.pos_x < MAN_MIN_X:
      Man.pos_x = MAN_MIN_X
      
    if Man.pos_y > MAN_MAX_Y:
      Man.pos_y = MAN_MAX_Y
    elif Man.pos_y < MAN_MIN_Y:
      Man.pos_y = MAN_MIN_Y
      
    if Man.pos_z > MAN_MAX_Z:
      Man.pos_z = MAN_MAX_Z
    elif Man.pos_z < MAN_MIN_Z:
      Man.pos_z = MAN_MIN_Z
     
    man_cmd_str = 'CC'
    man_cmd_str += 'X'+str(Man.pos_x)
    man_cmd_str += 'Y'+str(Man.pos_y)
    man_cmd_str += 'Z'+str(Man.pos_z)
    man_cmd_str += 'E'+str(Man.angle_effector)
    man_cmd_str += '\n'
    man_serial.write(man_cmd_str.encode())
    print(man_cmd_str)


#                                                         #
#                       SHARP SENSORS                     #
#                                                         #

# SPI library 
try:
  import Adafruit_GPIO.SPI as SPI
except ImportError:
  class SPI(object):
    MSBFIRST = 1
    def SpiDev(a, b, max_speed_hz): pass
    def transfer(a): pass
    def set_mode(a): pass
    def set_bit_order(a): pass

# Class representing MCP3208 - 12bit, 8 channel external ADC
class MCP3208(object):
  def __init__(self, device):
    self.spi = SPI.SpiDev(0, device, max_speed_hz=1000000)
    self.spi.set_mode(0)
    self.spi.set_bit_order(SPI.MSBFIRST)

  def __del__(self):
    self.spi.close()

  def read(self, ch):
    if 7 <= ch <= 0:
      raise Exception('MCP3208 channel must be 0-7: ' + str(ch))

    cmd = 128  # 1000 0000
    cmd += 64  # 1100 0000
    cmd += ((ch & 0x07) << 3)
    ret = self.spi.transfer([cmd, 0x0, 0x0])

    # get the 12b out of the return
    val = (ret[0] & 0x01) << 11  # only B11 is here
    val |= ret[1] << 3           # B10:B3
    val |= ret[2] >> 5           # MSB has B2:B0 ... need to move down to LSB

    return (val & 0x0FFF)  # ensure we are only sending 12b
    

NUMBER_OF_SENSORS = 12
MAX_DISTANCE = 80
MIN_DISTANCE = 10

adc0 = MCP3208(0)
adc1 = MCP3208(1)

adc0_raw_data = []
adc1_raw_data = []

for i in range(int(NUMBER_OF_SENSORS/2)):
  adc0_raw_data.append(0)
  adc1_raw_data.append(0)

sharp0_masured_distances = []
sharp1_masured_distances = []

for i in range(int(NUMBER_OF_SENSORS/2)):
  sharp0_masured_distances.append(0)
  sharp1_masured_distances.append(0)

def CheckSharpSensors():
  global sharp0_masured_distances
  global sharp1_masured_distances
  global adc0_raw_data
  global adc1_raw_data
  
  global NUMBER_OF_SENSORS 
  global MAX_DISTANCE
  global MIN_DISTANCE

  result = ''
  
  for i in range(int(NUMBER_OF_SENSORS/2)):
    adc0_raw_data[i] = adc0.read(i)
    
    if adc0_raw_data[i] != 0:
      sharp0_masured_distances[i] = 27.726 * pow(5.0*adc0_raw_data[i]/4095.0, -1.2045)
  
    if sharp0_masured_distances[i] > MAX_DISTANCE:
      sharp0_masured_distances[i] = MAX_DISTANCE
    elif sharp0_masured_distances[i] < MIN_DISTANCE:
      sharp0_masured_distances[i] = MIN_DISTANCE
    
    #FORMAT: SLC,SRC,SLR,SRF,RL,FR,RC,FC,RR,FL,SRR,SLF
    result += format(int(sharp0_masured_distances[i]), '02d') + ' '
  
  for i in range(int(NUMBER_OF_SENSORS/2)):
    adc1_raw_data[i] = adc1.read(i)
    
    if adc1_raw_data[i] != 0:
      sharp1_masured_distances[i] = 27.726 * pow(5.0*adc1_raw_data[i]/4095.0, -1.2045)
  
    if sharp1_masured_distances[i] > MAX_DISTANCE:
      sharp1_masured_distances[i] = MAX_DISTANCE
    elif sharp1_masured_distances[i] < MIN_DISTANCE:
      sharp1_masured_distances[i] = MIN_DISTANCE
    
    #FORMAT: SLC,SRC,SLR,SRF,RL,FR,RC,FC,RR,FL,SRR,SLF
    result += format(int(sharp1_masured_distances[i]), '02d') + ' '
   
  print(result)


#                                                         #
#                         ENCODERS                        #
#                                                         #

ENC_FL_PIN_A = 36
ENC_FL_PIN_B = 35

ENC_FR_PIN_A = 31
ENC_FR_PIN_B = 29

ENC_RL_PIN_A = 13
ENC_RL_PIN_B = 11

ENC_RR_PIN_A = 15
ENC_RR_PIN_B = 16

ENC_FL_counter = 0
ENC_FR_counter = 0
ENC_RL_counter = 0
ENC_RR_counter = 0

ENC_COUNTS_PER_ROTATION = 2100

class decoder:

   """Class to decode mechanical rotary encoder pulses."""

   def __init__(self, pi, gpioA, gpioB, callback):

      self.pi = pi
      self.gpioA = gpioA
      self.gpioB = gpioB
      self.callback = callback

      self.levA = 0
      self.levB = 0

      self.lastGpio = None

      self.pi.set_mode(gpioA, pigpio.INPUT)
      self.pi.set_mode(gpioB, pigpio.INPUT)

      self.pi.set_pull_up_down(gpioA, pigpio.PUD_UP)
      self.pi.set_pull_up_down(gpioB, pigpio.PUD_UP)

      self.cbA = self.pi.callback(gpioA, pigpio.EITHER_EDGE, self._pulse)
      self.cbB = self.pi.callback(gpioB, pigpio.EITHER_EDGE, self._pulse)

   def _pulse(self, gpio, level, tick):

      """
      Decode the rotary encoder pulse.

                   +---------+         +---------+      0
                   |         |         |         |
         A         |         |         |         |
                   |         |         |         |
         +---------+         +---------+         +----- 1

             +---------+         +---------+            0
             |         |         |         |
         B   |         |         |         |
             |         |         |         |
         ----+         +---------+         +---------+  1
      """

      if gpio == self.gpioA:
         self.levA = level
      else:
         self.levB = level;

      if gpio != self.lastGpio: # debounce
         self.lastGpio = gpio

         if   gpio == self.gpioA and level == 1:
            if self.levB == 1:
               self.callback(1)
         elif gpio == self.gpioB and level == 1:
            if self.levA == 1:
               self.callback(-1)

   def cancel(self):
      """
      Cancel the rotary encoder decoder.
      """
      self.cbA.cancel()
      self.cbB.cancel()
      

def ReadEnc_FL(way):
  global ENC_FL_counter
  ENC_FL_counter += way

def ReadEnc_FR(way):
  global ENC_FR_counter
  ENC_FR_counter += way

def ReadEnc_RL(way):
  global ENC_RL_counter
  ENC_RL_counter += way
  
def ReadEnc_RR(way):
  global ENC_RR_counter
  ENC_RR_counter += way
  

def checkEncodersBoundaries():
  global ENC_FL_counter
  global ENC_FR_counter
  global ENC_RL_counter
  global ENC_RR_counter
  
  if ENC_FL_counter > ENC_COUNTS_PER_ROTATION:
    ENC_FL_counter = 0
  elif ENC_FL_counter < 0:
    ENC_FL_counter = ENC_COUNTS_PER_ROTATION
    
  if ENC_FR_counter > ENC_COUNTS_PER_ROTATION:
    ENC_FR_counter = 0
  elif ENC_FR_counter < 0:
    ENC_FR_counter = ENC_COUNTS_PER_ROTATION
    
  if ENC_RL_counter > ENC_COUNTS_PER_ROTATION:
    ENC_RL_counter = 0
  elif ENC_RL_counter < 0:
    ENC_RL_counter = ENC_COUNTS_PER_ROTATION

  if ENC_RR_counter > ENC_COUNTS_PER_ROTATION:
    ENC_RR_counter = 0
  elif ENC_RR_counter < 0:
    ENC_RR_counter = ENC_COUNTS_PER_ROTATION

def PrintEncodersCounters():
  result = ''
  result += format(int(ENC_FL_counter), '07d') + ' ' 
  result += format(int(ENC_FR_counter), '07d') + ' ' 
  result += format(int(ENC_RL_counter), '07d') + ' ' 
  result += format(int(ENC_RR_counter), '07d') + ' ' 
  print(result)
    
    
#                                                         #
#                       PLATFORM                          #
#                                                         #

# Sends set of intructions to PWM_board via I2C
# <xx>_duty - pwm signal duty of <xx> engine [0-100]
# <xx>_dir - direction of rotiation of <xx> engine [0-1]
def updatePWM(fl_duty, fl_dir, fr_duty, fr_dir, rl_duty, rl_dir, rr_duty, rr_dir):
  rotation_directions_mask = 0
  if fl_dir:
    rotation_directions_mask = rotation_directions_mask | 0x01
  if fr_dir:
    rotation_directions_mask = rotation_directions_mask | 0x02
  if rl_dir:
    rotation_directions_mask = rotation_directions_mask | 0x04
  if rr_dir:
    rotation_directions_mask = rotation_directions_mask | 0x08
  try:
    i2c_bus.write_i2c_block_data(I2C_ADDR, I2C_CMD_START, [fl_duty, fr_duty, rl_duty, rr_duty, rotation_directions_mask])
  except OSError:
    print('I2C connection lost')
  return


# Parameters for measuring angular velocities of wheels
ENC_previous_time = 0.0
base_time_unit = 0.2 #seconds

WHEEL_FL_angular_velocity = 0.0
WHEEL_FR_angular_velocity = 0.0
WHEEL_RL_angular_velocity = 0.0
WHEEL_RR_angular_velocity = 0.0

WHEEL_FL_rotation_direction = 1
WHEEL_FR_rotation_direction = 1
WHEEL_RL_rotation_direction = 1
WHEEL_RR_rotation_direction = 1

ENC_FL_previous_counter_val = 0
ENC_FR_previous_counter_val = 0
ENC_RL_previous_counter_val = 0
ENC_RR_previous_counter_val = 0



# Takes current values of encoders counters and by comparing them 
# to previously saved, computes angular velocities of wheels
def measureAngularVelocities():
  global ENC_FL_counter
  global ENC_FR_counter
  global ENC_RL_counter
  global ENC_RR_counter
  
  global ENC_FL_previous_counter_val 
  global ENC_FR_previous_counter_val 
  global ENC_RL_previous_counter_val 
  global ENC_RR_previous_counter_val 
  
  global WHEEL_FL_angular_velocity
  global WHEEL_FR_angular_velocity
  global WHEEL_RL_angular_velocity
  global WHEEL_RR_angular_velocity
  
  counts_FL = float(abs(ENC_FL_counter - ENC_FL_previous_counter_val))
  counts_FR = float(abs(ENC_FR_counter - ENC_FR_previous_counter_val))
  counts_RL = float(abs(ENC_RL_counter - ENC_RL_previous_counter_val))
  counts_RR = float(abs(ENC_RR_counter - ENC_RR_previous_counter_val))
      
  WHEEL_FL_angular_velocity = (counts_FL*360.0) / (2100.0*base_time_unit) #degrees per second
  WHEEL_FR_angular_velocity = (counts_FR*360.0) / (2100.0*base_time_unit) 
  WHEEL_RL_angular_velocity = (counts_RL*360.0) / (2100.0*base_time_unit) 
  WHEEL_RR_angular_velocity = (counts_RR*360.0) / (2100.0*base_time_unit) 
      
  ENC_FL_previous_counter_val = ENC_FL_counter
  ENC_FR_previous_counter_val = ENC_FR_counter
  ENC_RL_previous_counter_val = ENC_RL_counter
  ENC_RR_previous_counter_val = ENC_RR_counter


# Variables storing data received from remote controller
angular_velocity_of_rotation = 0
direction_angle = 0
velocity = 0

# flag set when an obstacle is detected by any of the sharp sensors
f_obstacle = False
plat_data = [0,0,0]

def ControllPlatform(mode):
  
  global WHEEL_FL_angular_velocity
  global WHEEL_FR_angular_velocity
  global WHEEL_RL_angular_velocity
  global WHEEL_RR_angular_velocity
  
  global plat_data
  
  global angular_velocity_of_rotation
  global direction_angle
  global velocity
  
  global ENC_previous_time
  
  global sharp0_masured_distances
  global sharp0_masured_distances
  global f_obstacle
  
  if mode == 'PB':
    if time.time() - ENC_previous_time > base_time_unit:
      measureAngularVelocities()
      ENC_previous_time = time.time() 
      
    angular_velocity_of_rotation = plat_data[0] 
    direction_angle = plat_data[1]*math.pi/180.0
    velocity = plat_data[2] 
    
    vel_x = velocity*math.cos(direction_angle)
    vel_y = velocity*math.sin(direction_angle)
    
    W = 12.2
    H = 12.2
    R = 9.8066921 / 2
    
    new_WHEEL_FR_angular_velocity = (-1/R * (-vel_x + vel_y + angular_velocity_of_rotation * (W+H)))*180.0 / math.pi    
    new_WHEEL_FL_angular_velocity = (-1/R * (vel_x + vel_y - angular_velocity_of_rotation * (W+H)))*180.0 / math.pi
    new_WHEEL_RL_angular_velocity = (-1/R * (-vel_x + vel_y - angular_velocity_of_rotation * (W+H)))*180.0 / math.pi
    new_WHEEL_RR_angular_velocity = (-1/R * (vel_x + vel_y + angular_velocity_of_rotation * (W+H)))*180.0 / math.pi
    
    if new_WHEEL_FL_angular_velocity < 0:
      WHEEL_FL_rotation_direction = 1
    else:
      WHEEL_FL_rotation_direction = 0
      
    if new_WHEEL_FR_angular_velocity < 0:
      WHEEL_FR_rotation_direction = 1
    else:
      WHEEL_FR_rotation_direction = 0
      
    if new_WHEEL_RL_angular_velocity < 0:
      WHEEL_RL_rotation_direction = 1
    else:
      WHEEL_RL_rotation_direction = 0
      
    if new_WHEEL_RR_angular_velocity < 0:
      WHEEL_RR_rotation_direction = 1
    else:
      WHEEL_RR_rotation_direction = 0
    
    control_FL = abs(new_WHEEL_FL_angular_velocity*100/456)
    control_FR = abs(new_WHEEL_FR_angular_velocity*100/456)
    control_RL = abs(new_WHEEL_RL_angular_velocity*100/456)
    control_RR = abs(new_WHEEL_RR_angular_velocity*100/456)
    
    for i in sharp0_masured_distances:
      if i < 20:
        f_obstacle = True
        print(i)
    for i in sharp1_masured_distances:
      if i < 20:
        f_obstacle = True
    if f_obstacle:
      updatePWM(0,0,0,0,0,0,0,0)
    else:
      updatePWM(int(control_FL), int(WHEEL_FL_rotation_direction), int(control_FR), int(WHEEL_FR_rotation_direction), int(control_RL), int(WHEEL_RL_rotation_direction), int(control_RR), int(WHEEL_RR_rotation_direction))
    f_obstacle = False
        
#                                                         #
#                         MAIN                            #
#                                                         #
                                           
BluetoothConnect()
print(f_bluetooth_mode)

while f_bluetooth_mode == True: 
    
  pi = pigpio.pi()
  FL_decoder = decoder(pi, 16, 19, callback_FL)
  FR_decoder = decoder(pi, 6, 12, callback_FR)
  RL_decoder = decoder(pi, 27, 17, callback_RL)
  RR_decoder = decoder(pi, 22, 23, callback_RR)
  
  #PrintEncodersCounters()
  CheckSharpSensors()
  in_remote_data = bt_serial.readline().decode()
      
  if in_remote_data[0:4] == 'CMA ':
    temp = re.findall(r'-?\d+\.?\d*', in_remote_data[4:]) 
    man_angles_data = list(map(int, temp)) 
    if len(man_angles_data) >= 4: 
      ControllManipulator('MA')
        
  elif in_remote_data[0:4] == 'CMC ':
    temp = re.findall(r'-?\d+\.?\d*', in_remote_data[4:]) 
    man_xyz_data = list(map(int, temp)) 
    if len(man_xyz_data) >= 4: 
      ControllManipulator('MC')
        
  elif in_remote_data[0:4] == 'CPB ':
    temp = re.findall(r'-?\d+\.?\d*', in_remote_data[4:]) 
    plat_data = list(map(int, temp)) 
    if len(plat_data) >= 3: 
      ControllPlatform('PB')
        
  
FL_decoder.cancel()
FR_decoder.cancel()
RL_decoder.cancel()
RR_decoder.cancel()

pi.stop()
    
      
