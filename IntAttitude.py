'''
* main.py : Attitude caculator using pure gyro data integration
*
*          Copyright (C) 2018 by XiaoqiangChen, All rights reserved.
* author  : XiaoqiangChen
* mail    : 309905109@qq.com
* history : 2018/09/26  1.0  new
'''

from CAttitude import Attitude
class IntAttitude(Attitude):
    """docstring for ClassName"""
    _phi0 = 0;
    _theta0 = 0;
    _psi0 = 0;
    def __init__(self):
        super(IntAttitude, self).__init__()
        self._strategy = "Int method";
        
    def calculateAtt(self):
        lsGyros = zip(self._dataSet._lsGyroX,self._dataSet._lsGyroY,self._dataSet._lsGyroZ);
        for gyro in lsGyros:
            self._phi0   = self._phi0 + gyro[0];
            self._theta0 = self._theta0 + gyro[1];
            self._psi0   = self._psi0 + gyro[2];

            self._lsPitch.append(self._phi0);
            self._lsRoll.append(self._psi0);

def main():
    att = IntAttitude('09_26_14_sensor_combined_0.csv');
    att.calculateAtt();
    att.showFig();

if __name__ == '__main__':
    main()