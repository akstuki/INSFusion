'''
* magnetometer.py : magnetometer data
*
*          Copyright (C) 2018 by XiaoqiangChen, All rights reserved.
* author  : XiaoqiangChen
* mail    : 309905109@qq.com
* history : 2018/09/26  1.0  new
'''
import math
class magnetometer():
    """docstring for magnetometer"""
    def __init__(self):
        self._magn_x = 0
        self._magn_y = 0
        self._magn_z = 0
    def __init__(self, m_x: float, m_y: float, m_z: float):
        self._magn_x = m_x
        self._magn_y = m_y
        self._magn_z = m_z

    def to_list(self) -> list:
        return [self._magn_x, self._magn_y, self._magn_z]

    def is_valid(self):
        if math.isclose(self._magn_x, 0.0) and math.isclose(self._magn_y, 0.0) and math.isclose(self._magn_z, 0.0):
            return False
        else:
            return True

    def normalised(self) -> (float, float, float):
        reciprocal_norm = 1.0/math.sqrt(self._magn_x**2 + self._magn_y**2 + self._magn_z**2)
        d_x = self._magn_x*reciprocal_norm
        d_y = self._magn_y*reciprocal_norm
        d_z = self._magn_z*reciprocal_norm
        return d_x, d_y, d_z

    def mag_heading(self, roll: float, pitch: float) -> float:
        '''use body mag data to caculate yaw, roll pitch known'''
        mn_x = self._magn_y*math.cos(pitch)-self._magn_z*math.sin(pitch)
        mn_y = self._magn_x*math.cos(roll)+ \
        self._magn_y*math.sin(roll)*math.sin(pitch)+self._magn_z*math.sin(roll)*math.cos(pitch)
        return -math.atan2(mn_x, mn_y)
