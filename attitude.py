'''
* main.py : attitude caculator parent class
*
*          Copyright (C) 2018 by XiaoqiangChen, All rights reserved.
* author  : XiaoqiangChen
* mail    : 309905109@qq.com
* history : 2018/09/26  1.0  new
'''
import math
import matplotlib.pyplot as plt
from data_set import DataSet

class attitude():
    """docstring for attitude"""
    def __init__(self):
        self._strategy = "none"
        self._data_set = None
        self._ls_pitch = []
        self._ls_roll = []
        self._ls_yaw = []

    def load_data(self, imu_file_name: str, att_file_name: str):
        '''read pixhawk log file'''
        self._data_set = DataSet(imu_file_name, att_file_name)
        # self._data_set.load_imu_data()
        # self._data_set.load_px4_att()
        self._data_set.load_open_imu_data()

    def remove_allresults(self):
        self._ls_pitch.clear()
        self._ls_roll.clear()
        self._ls_yaw.clear()

    def calculate_att(self):
        '''implement in subclass'''
        self.remove_allresults()
    
    def add_pitch_roll_yaw(self, pitch: float, roll: float, yaw: float):
        ''' '''
        self._ls_pitch.append(pitch)
        self._ls_roll.append(roll)
        self._ls_yaw.append(yaw)

    def show_fig(self):
        '''show fig of calculated attitude and pixhawk attitude'''
        if not self._ls_pitch:
            print('no result')
            return
        # ekf_times, ekf_pitchs, ekf_rolls, ekf_yaws = self._data_set.get_ekf_attitude()
        imu_times = self._data_set.get_imu_times()

        plt.figure(self._strategy)
        plt.subplot(311)
        pitch_deg = [c*57.2957795 for c in self._ls_pitch]
        plt.plot(imu_times, pitch_deg, label="pitch")
        plt.ylabel('pitch(deg)')
        plt.title(self._strategy)
        plt.legend()
        plt.grid(True)
        plt.grid(linestyle='--')

        plt.subplot(312)
        roll_deg = [c*57.2957795 for c in self._ls_roll]
        plt.plot(imu_times, roll_deg, label="roll")
        plt.ylabel('roll(deg)')
        plt.xlabel('time(s)')
        plt.legend()
        plt.grid(True)
        plt.grid(linestyle='--')

        plt.subplot(313)
        yaw_deg = [c*57.2957795 for c in self._ls_yaw]
        plt.plot(imu_times, yaw_deg, label="yaw")
        plt.ylabel('yaw(deg)')
        plt.xlabel('time(s)')
        plt.legend()
        plt.grid(True)
        plt.grid(linestyle='--')

        plt.show()

    def test(self):
        '''main test'''
        sensorfile = r'test\09_26_14_sensor_combined_0.csv'
        attfile = r'test\09_26_14_vehicle_attitude_0.csv'
        self.load_data(sensorfile, attfile)
        self.calculate_att()
        self.show_fig()

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
    # att = attitude()
    pass

if __name__ == '__main__':
    main()
