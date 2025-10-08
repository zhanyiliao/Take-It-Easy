from collections import deque
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250
from scipy.spatial.distance import euclidean
# import dronekit
import math
import numpy as np
import serial
import smbus
import sys
import time

# --------------------------------------------------

def angle_normalize(angle):

    angle %= 360
    return angle if angle < 180 else (angle - 360)

# --------------------------------------------------

class Locator():

    def __init__(self, anchors, steps=50, epsilon=1e-5, port='/dev/ttyUSB0'):

        self._anchors = np.array(anchors)
        self._steps = steps
        self._epsilon = epsilon
        self._prev_steps = deque(maxlen=5)
        
        self._ser = serial.Serial(port=port,
                                  baudrate=115200,
                                  bytesize=serial.EIGHTBITS,
                                  parity=serial.PARITY_NONE,
                                  timeout=0.5,
                                  xonxoff=False,
                                  rtscts=False,
                                  dsrdtr=False)
        self._ser.close()
        
        try:
            self._ser.open()
            self._ser.flushInput()
            self._ser.flushOutput()
        except Exception as ex:
            pass
            # print(ex)

    def locate(self, method='Rtls', clear=False):

        if clear:
            self._prev_steps.clear()

        while True:

            data = self._ser.read(self._ser.inWaiting()).decode()
            data = [_ for _ in data.split('\n') if _.startswith(method)]
            result = list()

            # if data:
            for d in range(min(10, len(data))):
                xy = data[d].split(' ')
                if method == 'Rtls':
                    if len(xy) >= 9:
                        try:
                            location = [int(xy[_]) // 10 for _ in (2, 7)]
                        except:
                            continue
                elif method == 'Dist':
                    if len(xy) >= 15:
                        try:
                            distances = [int(xy[_]) // 10 for i in (1, 5, 8, 11)]
                            location = self._coordinate(distances)
                        except:
                            continue
                else:
                    raise ValueError('Unknown locating method: %s' % method)
                result.append(location)

            if result:
                break

        result = np.mean(result, axis=0).tolist()
        self._prev_steps.append(result)
        return result

    def get_angle(self):

        angle_now = np.subtract(self._prev_steps[-1], self._prev_steps[0])
        angle_now = math.degrees(np.arctan2(*angle_now[::-1]))
        
        return angle_now

    def _coordinate(self, distances):

        location = np.mean(self._anchors, axis=0)
        ds = np.vstack(self.the_lenet())

        for i in range(self._steps):

            vectors = location - self._anchors
            distances = np.sqrt(np.sum(vectors ** 2, axis=1, keepdims=True))
            movements = np.sum(vectors * ((ds / distances) - 1), axis=0)
            location += movements * (1 - i / self._steps)
            
            if all(np.abs(movements) < self._epsilon):
                break

        return location

    def __del__(self):

        self._ser.close()


class MPU():

    def __init__(self, bus=1):

        self._mpu = MPU9250(address_ak=AK8963_ADDRESS,
                            address_mpu_master=MPU9050_ADDRESS_68,
                            address_mpu_slave=None,
                            bus=bus,
                            gfs=GFS_1000,
                            afs=AFS_8G,
                            mfs=AK8963_BIT_16,
                            mode=AK8963_MODE_C100HZ)
        
        self._reset()

    def _reset(self):

        self._mpu.reset()
        self._mpu.configure()

    def get_angle(self):

        degrees = 0.0

        while degrees == 0.0:

            # from Dave X
            x, y, _ = self._mpu.readMagnetometerMaster()

            if x == 0.0 and y == 0.0:
                print('re-connect')
                self._reset()
                continue
            elif x > 0:
                radians = math.atan(y / x)
            elif x < 0:
                radians = math.atan(y / x) + math.copysign(math.pi, y)
            else:
                radians = math.copysign(math.pi, y) / 2

            degrees = angle_normalize(90 - math.degrees(radians))
        
        return degrees


class PCA():

    def __init__(self, address=0x40, bus=1, write_delay=0.1):

        self._address = address
        self._bus = smbus.SMBus(bus)
        self._write_delay = write_delay

        time.sleep(self._write_delay)
        self._bus.write_i2c_block_data(self._address, 0xFE, [0x79])
        time.sleep(self._write_delay)
        self._bus.write_i2c_block_data(self._address, 0x00, [0x01])

        self._reset()

    def _control(self, channel, value, set_on=False):

        low_byte_value = value & 0x00FF
        high_byte_value = (value & 0x0F00) >> 8
        reg_low_byte = (0x06 if set_on else 0x08) + 4 * channel

        time.sleep(self._write_delay)
        self._bus.write_i2c_block_data(self._address, reg_low_byte, [low_byte_value])
        time.sleep(self._write_delay)
        self._bus.write_i2c_block_data(self._address, reg_low_byte + 1, [high_byte_value])

    def move(self, l_speed, r_speed):

        self._control(1, l_speed)
        self._control(4, r_speed)

    def turn(self, l_forward, r_forward):

        self._control(2, 0 if l_forward else 4095)
        self._control(5, 4095 if r_forward else 0)

    def brake(self, engage):

        value = 4095 if engage else 0
        self._control(3, value)
        self._control(6, value)

    def _reset(self):

        self.brake(True)
        self.turn(True, True)
        self.move(0, 0)

    def __del__(self):

        self._reset()


class Pixhawk():

    def __init__(self, port):

        self._port = port
        self._vehicle = None

    def connect(self, wait_ready=True, standby_time=0):

        # 127.0.0.1:14551 /dev/ttyUSB0
        self._vehicle = dronekit.connect(self._port, wait_ready=wait_ready)
        time.sleep(standby_time)

    def get_yaw(self):

        if self._vehicle is not None:
            degrees = (math.degrees(self._vehicle.attitude.yaw) + 360) % 360
            degrees = (degrees + 360) % 360
            return degrees
        else:
            raise OSError('Pixhawk device isn\'t connected.')
