'''
* main.py : Attitude caculator using pure gyro data integration
*
*          Copyright (C) 2018 by XiaoqiangChen, All rights reserved.
* author  : XiaoqiangChen
* mail    : 309905109@qq.com
* history : 2018/09/26  1.0  new
'''

from attitude import Attitude
class IntAttitude(Attitude):
    """docstring for ClassName"""
    _phi0 = 0;
    _theta0 = 0;
    _psi0 = 0;
    def __init__(self):
        super(IntAttitude, self).__init__()
        self._strategy = "Int method";
        
    def calculate_att(self):
        lsGyros = zip(self._dataSet._lsGyroX,self._dataSet._lsGyroY,self._dataSet._lsGyroZ,self._dataSet._lsDeltT);
        for gyro in lsGyros:
            dt = gyro[3];
            self._theta0 = self._theta0 + dt*gyro[0];
            self._phi0   = self._phi0   + dt*gyro[1];
            self._psi0   = self._psi0   + dt*gyro[2];

            self._ls_pitch.append(self._theta0);
            self._ls_roll.append(self._phi0);
            self._ls_yaw.append(self._psi0);

def main():
    sensorfile = r'test\09_26_14_sensor_combined_0.csv';
    attfile = r'test\09_26_14_vehicle_attitude_0.csv'
    att = IntAttitude();
    att.load_data(sensorfile,attfile);
    att.calculate_att();
    att.showFig();

if __name__ == '__main__':
    main()