'''
* main.py : data manager
*
*          Copyright (C) 2018 by XiaoqiangChen, All rights reserved.
* author  : XiaoqiangChen
* mail    : 309905109@qq.com
* history : 2018/09/26  1.0  new
'''

import os
from lib.quaternion import quat2euler
from accelerometer import accelerometer
from gyroscope import gyroscope
from magnetometer import magnetometer

class DataSet():
    """docstring for DataSet"""
    _filename = ""
    _lsTimes = []
    _lsDeltT = []

    _attitude_filename = ""
    _lsEkfTimes = []
    _lsEkfPitch = []
    _lsEkfRoll = []
    _lsEkfYaw = []
    def __init__(self, filename: str, attFileName: str):
        self._filename = filename
        self._attitude_filename = attFileName
        self._ls_accel = []
        self._ls_gyro = []
        self._ls_mage = []

    def load_imu_data(self):
        '''read data from px4 vehicle attitude.csv log file'''
        # (0-4)   timestamp   gyro_rad[0] gyro_rad[1] gyro_rad[2] gyro_integral_dt
        # (5-7)   accelerometer_timestamp_relative    accelerometer_m_s2[0]   accelerometer_m_s2[1]
        # (8-9)   accelerometer_m_s2[2]   accelerometer_integral_dt
        # (10-12) magnetometer_timestamp_relative magnetometer_ga[0]  magnetometer_ga[1]
        # (13-16) magnetometer_ga[2]  baro_timestamp_relative baro_alt_meter  baro_temp_celcius
        if not os.path.exists(self._filename):
            print("sensors file [{filename}] not exist".format(filename=self._filename))
            return
        with open(self._filename, 'rt') as fp_imu:
            for line in fp_imu:
                ls_fields = line.strip('\n').split(',')
                if len(ls_fields) < 17:
                    continue
                if self.is_header(ls_fields):
                    continue
                self._lsTimes.append(1e-6*float(ls_fields[0]))
                self._lsDeltT.append(float(ls_fields[4]))
                accel = accelerometer(float(ls_fields[6]), float(ls_fields[7]), float(ls_fields[8]))
                gyro = gyroscope(float(ls_fields[1]), float(ls_fields[2]), float(ls_fields[3]))
                magn = magnetometer(float(ls_fields[11]), float(ls_fields[12]), float(ls_fields[13]))
                self._ls_accel.append(accel)
                self._ls_gyro.append(gyro)
                self._ls_mage.append(magn)

    def is_header(self, ls_fields: list) -> bool:
        if ls_fields[0] == 'timestamp':
            return True
        else:
            return False

    def get_sensors_imu(self) -> zip:
        '''return imu data'''
        return zip(self._lsDeltT, self._ls_gyro, self._ls_accel, self._ls_mage)

    def get_ekf_attitude(self) -> (list, list, list, list):
        '''pixhawk attitude: time,pitch,roll,yaw'''
        return self._lsEkfTimes, self._lsEkfPitch, self._lsEkfRoll, self._lsEkfYaw

    def get_imu_times(self) -> list:
        '''return sensors timestamp'''
        return self._lsTimes

    def load_px4_att(self):
        '''' timestamp rollspeed   pitchspeed  yawspeed    q[0]    q[1]    q[2]    q[3]'''
        if not os.path.exists(self._attitude_filename):
            return
        with open(self._attitude_filename, 'rt') as fp_att:
            for line in fp_att:
                ls_fields = line.strip('\n').split(',')
                if len(ls_fields) < 8:
                    continue
                if self.is_header(ls_fields):
                    continue
                self._lsEkfTimes.append(1e-6*float(ls_fields[0]))
                q_0, q_1, q_2, q_3 = ls_fields[4:8]
                phi, theta, psi = quat2euler(float(q_0), float(q_1), float(q_2), float(q_3))
                self._lsEkfPitch.append(phi)
                self._lsEkfRoll.append(theta)
                self._lsEkfYaw.append(psi)

if __name__ == '__main__':
    IMU_FILE = r'test\09_26_14_sensor_combined_0.csv'
    ATT_FILE = r'test\09_26_14_vehicle_attitude_0.csv'
    DATA_MGR = DataSet(IMU_FILE, ATT_FILE)
    DATA_MGR.load_px4_att()
    DATA_MGR.load_imu_data()
