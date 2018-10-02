'''
* main.py : Attitude caculator using accelmeter data
*
*          Copyright (C) 2018 by XiaoqiangChen, All rights reserved.
* author  : XiaoqiangChen
* mail    : 309905109@qq.com
* history : 2018/09/26  1.0  new
'''

from attitude import Attitude
from attitude import acc_att
from attitude import mag_heading

class AccAttitude(Attitude):
    """docstring for ClassName"""
    def __init__(self):
        super(AccAttitude, self).__init__()
        self._strategy = "accelerometer method"

    def calculate_att(self):
        imu_data = self._data_set.get_imu_data()
        for imu in imu_data:
            pitch, roll = acc_att(imu[1])
            self._ls_pitch.append(pitch)
            self._ls_roll.append(roll)
            self._ls_yaw.append(mag_heading(imu[2], roll, pitch))

def main():
    '''test main'''
    sensorfile = r'test\09_26_14_sensor_combined_0.csv'
    attfile = r'test\09_26_14_vehicle_attitude_0.csv'
    att = AccAttitude()
    att.load_data(sensorfile, attfile)
    att.calculate_att()
    att.show_fig()

if __name__ == '__main__':
    main()
