'''
* main.py : Attitude caculator using complementary filter
*
*          Copyright (C) 2018 by XiaoqiangChen, All rights reserved.
* author  : XiaoqiangChen
* mail    : 309905109@qq.com
* history : 2018/09/26  1.0  new
'''

from CAttitude import Attitude
class CompAttitude(Attitude):
    """docstring for ClassName"""
    _phi0 = 0;
    _theta0 = 0;
    _psi0 = 0;
    _a = 0.7;

    def __init__(self):
        super(CompAttitude, self).__init__()
        self._strategy = "comp method";

    def calculateAtt(self):
        lsGyros = zip(self._dataSet._lsDeltT,self._dataSet._lsGyroX,self._dataSet._lsGyroY,self._dataSet._lsGyroZ);
        lsAccs = zip(self._dataSet._lsAcclX,self._dataSet._lsAcclY,self._dataSet._lsAcclZ);
        for imu in zip(lsGyros,lsAccs):
            pitch,roll = self.accAtt(imu[1]);
            dt = imu[0][0];
            self._phi0   = self._a*(self._phi0   + dt*imu[0][1])+(1-self._a)*pitch;
            self._theta0 = self._a*(self._theta0 + dt*imu[0][2])+(1-self._a)*roll;
            # self._psi0   = self._a*(self._psi0   + 0.004*imu[0][2])+(1-self._a)*imu[1][2];

            self._lsPitch.append(self._phi0);
            self._lsRoll.append(self._theta0);