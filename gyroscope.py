'''
* gyroscope.py : gyroscope data
*
*          Copyright (C) 2018 by XiaoqiangChen, All rights reserved.
* author  : XiaoqiangChen
* mail    : 309905109@qq.com
* history : 2018/09/26  1.0  new
'''

class gyroscope():
    """docstring for gyroscope"""
    def __init__(self):
        self._gyro_x = 0
        self._gyro_y = 0
        self._gyro_z = 0

    def __init__(self, g_x: float, g_y: float, g_z: float):
        self._gyro_x = g_x
        self._gyro_y = g_y
        self._gyro_z = g_z
