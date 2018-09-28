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

    def __init__(self,a:float):
        super(CompAttitude, self).__init__()
        self._strategy = "comp method";
        self._a = a;

    def calculateAtt(self):
        lsGyros = zip(self._dataSet._lsDeltT,self._dataSet._lsGyroX,self._dataSet._lsGyroY,self._dataSet._lsGyroZ);
        lsAccs = zip(self._dataSet._lsAcclX,self._dataSet._lsAcclY,self._dataSet._lsAcclZ);
        lsmag = zip(self._dataSet._lsMageX,self._dataSet._lsMageY,self._dataSet._lsMageZ);
        for imu in zip(lsGyros,lsAccs,lsmag):
            pitch,roll = self.accAtt(imu[1]);
            dt = imu[0][0];
            self._theta0   = self._a*(self._theta0   + dt*imu[0][0])+(1-self._a)*pitch;
            self._phi0 = self._a*(self._phi0 + dt*imu[0][1])+(1-self._a)*roll;
            yaw = self.magHeading(imu[2],self._theta0,self._phi0)
            self._psi0   = self._a*(self._psi0   + dt*imu[0][2])+(1-self._a)*yaw;

            self._lsPitch.append(self._theta0);
            self._lsRoll.append(self._phi0);
            self._lsYaw.append(self._psi0)

def main():
    sensorfile = r'test\09_26_14_sensor_combined_0.csv';
    attfile = r'test\09_26_14_vehicle_attitude_0.csv'
    att = CompAttitude(0.7);
    att.loadData(sensorfile,attfile);
    att.calculateAtt();
    att.showFig();

if __name__ == '__main__':
    main()