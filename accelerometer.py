'''
* accelerometer.py : accelerometer data
*
*          Copyright (C) 2018 by XiaoqiangChen, All rights reserved.
* author  : XiaoqiangChen
* mail    : 309905109@qq.com
* history : 2018/09/26  1.0  new
'''
import math
from typing import NoReturn

class accelerometer():
    """docstring for accelerometer"""
    def __init__(self):
        self._accel_x = 0
        self._accel_y = 0
        self._accel_z = 0

    def __init__(self, ax: float, ay: float, az: float):
    	self._accel_x = ax
    	self._accel_y = ay
    	self._accel_z = az

    def is_valid(self) -> bool:
        if math.isclose(self._accel_x, 0.0) and math.isclose(self._accel_y, 0.0) and math.isclose(self._accel_z, 0.0):
            return False
        else:
            return True

    def reverse(self) -> NoReturn:
        self._accel_x = -self._accel_x
        self._accel_y = -self._accel_y
        self._accel_z = -self._accel_z

    def normalised(self) -> (float, float, float):
        reciprocal_norm = 1.0/math.sqrt(self._accel_x**2 + self._accel_y**2 + self._accel_z**2)
        d_x = self._accel_x*reciprocal_norm
        d_y = self._accel_y*reciprocal_norm
        d_z = self._accel_z*reciprocal_norm
        return d_x, d_y, d_z

    def acc_att(self) -> tuple:
        '''use body accelemeters to caculate pitch and roll'''
        norm = math.sqrt(self._accel_x**2 + self._accel_z**2)
        pitch = math.atan2(self._accel_y, norm)
        roll = math.atan2(self._accel_x, -self._accel_z)
        return pitch, roll

def main() -> NoReturn:
    '''test'''
    accel = accelerometer(0, 0, -9.8)
    print(accel.acc_att())

if __name__ == '__main__':
    main()

