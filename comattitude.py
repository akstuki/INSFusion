'''
* main.py : attitude caculator using complementary filter
*
*          Copyright (C) 2018 by XiaoqiangChen, All rights reserved.
* author  : XiaoqiangChen
* mail    : 309905109@qq.com
* history : 2018/09/26  1.0  new
'''

from attitude import attitude
from attitude import acc_att
from attitude import mag_heading

class CompAttitude(attitude):
    """docstring for ClassName"""
    _phi0 = 0
    _theta0 = 0
    _psi0 = 0
    _a = 0.7

    def __init__(self, a: float):
        super(CompAttitude, self).__init__()
        self._strategy = "comp method"
        self._a = a

    def calculate_att(self):
        '''complementary filter main cycle'''
        imu_data = self._data_set.get_imu_data()
        for imu in imu_data:
            self.filter_pitch_roll_yaw(imu)
            self.add_pitch_roll_yaw(self._theta0, self._phi0, self._psi0)

    def filter_pitch_roll_yaw(self, imu:tuple):
        pitch, roll = acc_att(imu[1])
        d_t = imu[0][0]
        self._theta0 = self.siso_filter(self._theta0, pitch, d_t, imu[0][1])
        self._phi0 = self.siso_filter(self._phi0, roll, d_t, imu[0][2])
        yaw = mag_heading(imu[2], self._theta0, self._phi0)
        self._psi0 = self.siso_filter(self._psi0, yaw, d_t, imu[0][3])

    def siso_filter(self, last_hat: float, obs: float, dt: float, gyro: float) -> float:
        """ """
        x_hat = self._a*(last_hat + dt*gyro)+(1-self._a)*obs
        return x_hat

def main():
    '''test main'''
    att = CompAttitude(0.7)
    att.test()

if __name__ == '__main__':
    main()
