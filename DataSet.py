'''
* main.py : data manager
*
*          Copyright (C) 2018 by XiaoqiangChen, All rights reserved.
* author  : XiaoqiangChen
* mail    : 309905109@qq.com
* history : 2018/09/26  1.0  new
'''

import os
# from Quaternion import Quat2Euler
from lib.Quaternion import Quat2Euler
class DataSet(object):
    """docstring for DataSet"""
    _filename = "";
    _lsTimes  = [];
    _lsDeltT  = [];
    _lsAcclX  = [];
    _lsAcclY  = [];
    _lsAcclZ  = [];
    _lsGyroX  = [];
    _lsGyroY  = [];
    _lsGyroZ  = [];
    _lsMageX  = [];
    _lsMageY  = [];
    _lsMageZ  = [];

    _attitude_filename = "";
    _lsEkfTimes  = [];
    _lsEkfPitch = [];
    _lsEkfRoll = [];
    _lsEkfYaw = [];
    def __init__(self,filename:str,attFileName:str):
        self._filename = filename;
        self._attitude_filename = attFileName;

    def LoadIMUData(self):
    # (0-4)   timestamp   gyro_rad[0] gyro_rad[1] gyro_rad[2] gyro_integral_dt   
    # (5-7)   accelerometer_timestamp_relative    accelerometer_m_s2[0]   accelerometer_m_s2[1]  
    # (8-9)   accelerometer_m_s2[2]   accelerometer_integral_dt 
    # (10-12) magnetometer_timestamp_relative magnetometer_ga[0]  magnetometer_ga[1] 
    # (13-16) magnetometer_ga[2]  baro_timestamp_relative baro_alt_meter  baro_temp_celcius
        if os.path.exists(self._filename) == False:
            print("sensors file [{filename}] not exist".format(filename=self._filename));
            return;
        with open(self._filename,'rt') as f:
            for line in f:
                lsField = line.strip('\n').split(',');
                if len(lsField)<17:
                    continue;
                if lsField[0] == 'timestamp':
                    continue;
                self._lsTimes.append(1e-6*float(lsField[0]));
                self._lsDeltT.append(float(lsField[4]));
                self._lsAcclX.append(float(lsField[6]));
                self._lsAcclY.append(float(lsField[7]));
                self._lsAcclZ.append(float(lsField[8]));
                self._lsGyroX.append(float(lsField[1]));
                self._lsGyroY.append(float(lsField[2]));
                self._lsGyroZ.append(float(lsField[3]));
                self._lsMageX.append(float(lsField[11]));
                self._lsMageY.append(float(lsField[12]));
                self._lsMageZ.append(float(lsField[13]));

    def loadEkfAtt(self):
    # timestamp rollspeed   pitchspeed  yawspeed    q[0]    q[1]    q[2]    q[3]
        if os.path.exists(self._attitude_filename) == False:
            return;
        with open(self._attitude_filename,'rt') as f:
            for line in f:
                lsField = line.strip('\n').split(',');
                if len(lsField)<8:
                    continue;
                if lsField[0] == 'timestamp':
                    continue;
                self._lsEkfTimes.append(1e-6*float(lsField[0]));
                q0,q1,q2,q3 = lsField[4:8];
                phi,theta,psi = Quat2Euler(float(q0),float(q1),float(q2),float(q3));
                self._lsEkfPitch.append(phi);
                self._lsEkfRoll.append(theta);
                self._lsEkfYaw.append(psi);