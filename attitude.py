'''
* main.py : Attitude caculator parent class
*
*          Copyright (C) 2018 by XiaoqiangChen, All rights reserved.
* author  : XiaoqiangChen
* mail    : 309905109@qq.com
* history : 2018/09/26  1.0  new
'''
import math
import matplotlib.pyplot as plt
from data_set import DataSet

class Attitude():
    """docstring for Attitude"""
    def __init__(self):
        self._strategy = "none"
        self._data_set = None
        self._ls_pitch = []
        self._ls_roll = []
        self._ls_yaw = []

    def load_data(self, imu_file_name: str, att_file_name: str):
        '''read pixhawk log file'''
        self._data_set = DataSet(imu_file_name, att_file_name)
        self._data_set.load_imu_data()
        self._data_set.load_px4_att()

    def calculate_att(self):
        '''implement in subclass'''
        pass

    def show_fig(self):
        '''show fig of calculated attitude and pixhawk attitude'''
        if not self._ls_pitch:
            print('no result')
            return
        ekf_times, ekf_pitchs, ekf_rolls, ekf_yaws = self._data_set.get_ekf_attitude()
        imu_times = self._data_set.get_imu_times()

        plt.figure(1)
        plt.subplot(311)
        plt.plot(imu_times, self._ls_pitch, label="pitch")
        plt.plot(ekf_times, ekf_pitchs, label="EKF2")
        plt.ylabel('pitch(rad)')
        plt.title(self._strategy)
        plt.legend()
        plt.grid(True)

        plt.subplot(312)
        plt.plot(imu_times, self._ls_roll, label="roll")
        plt.plot(ekf_times, ekf_rolls, label="EKF2")
        plt.ylabel('roll(rad)')
        plt.xlabel('time(s)')
        plt.legend()
        plt.grid(True)

        plt.subplot(313)
        plt.plot(imu_times, self._ls_yaw, label="yaw")
        plt.plot(ekf_times, ekf_yaws, label="EKF2")
        plt.ylabel('yaw(rad)')
        plt.xlabel('time(s)')
        plt.legend()
        plt.grid(True)

        plt.show()

def acc_att(acc: list) -> tuple:
    '''use body accelemeters to caculate pitch and roll'''
    norm = math.sqrt(acc[0]**2 + acc[2]**2)
    pitch = math.atan2(acc[1], norm)
    roll = math.atan2(acc[0], -acc[2])
    return pitch, roll

def mag_heading(mag: list, roll: float, pitch: float) -> float:
    '''use body mag data to caculate yaw, roll pitch known'''
    mn_x = mag[1]*math.cos(pitch)-mag[2]*math.sin(pitch)
    mn_y = mag[0]*math.cos(roll)+ \
    mag[1]*math.sin(roll)*math.sin(pitch)+mag[2]*math.sin(roll)*math.cos(pitch)
    return -math.atan2(mn_x, mn_y)

def main():
    '''test'''
    # att = Attitude()
    pass

if __name__ == '__main__':
    main()
