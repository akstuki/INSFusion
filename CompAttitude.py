'''
* main.py : Attitude caculator using complementary filter
*
*          Copyright (C) 2018 by XiaoqiangChen, All rights reserved.
* author  : XiaoqiangChen
* mail    : 309905109@qq.com
* history : 2018/09/26  1.0  new
'''

from attitude import Attitude
from attitude import acc_att
from attitude import mag_heading

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

    def calculate_att(self):
        imu_data = self._data_set.get_imu_data()
        for imu in imu_data:
            pitch,roll = acc_att(imu[1]);
            dt = imu[0][0];
            self._theta0   = self._a*(self._theta0   + dt*imu[0][0])+(1-self._a)*pitch;
            self._phi0 = self._a*(self._phi0 + dt*imu[0][1])+(1-self._a)*roll;
            yaw = mag_heading(imu[2],self._theta0,self._phi0)
            self._psi0   = self._a*(self._psi0   + dt*imu[0][2])+(1-self._a)*yaw;

            self._ls_pitch.append(self._theta0);
            self._ls_roll.append(self._phi0);
            self._ls_yaw.append(self._psi0)

def main():
    sensorfile = r'test\09_26_14_sensor_combined_0.csv';
    attfile = r'test\09_26_14_vehicle_attitude_0.csv'
    att = CompAttitude(0.7);
    att.load_data(sensorfile,attfile);
    att.calculate_att();
    att.show_fig();

if __name__ == '__main__':
    main()