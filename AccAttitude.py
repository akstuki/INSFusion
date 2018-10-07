'''
* main.py : attitude caculator using accelmeter data
*
*          Copyright (C) 2018 by XiaoqiangChen, All rights reserved.
* author  : XiaoqiangChen
* mail    : 309905109@qq.com
* history : 2018/09/26  1.0  new
'''

from attitude import attitude
from attitude import acc_att
from attitude import mag_heading

class AccAttitude(attitude):
    """docstring for ClassName"""
    def __init__(self):
        super(AccAttitude, self).__init__()
        self._strategy = "accelerometer method"

    def calculate_att(self):
        '''accel method main cycle'''
        imu_data = self._data_set.get_sensors_imu()
        for imu in imu_data:
            accel = imu[2]
            magn = imu[3]
            pitch, roll = accel.acc_att()
            yaw = magn.mag_heading(roll, pitch)
            self.add_pitch_roll_yaw(pitch, roll, yaw)

def main():
    '''test main'''
    att = AccAttitude()
    att.test()

if __name__ == '__main__':
    main()
