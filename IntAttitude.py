'''
* main.py : attitude caculator using pure gyro data integration
*
*          Copyright (C) 2018 by XiaoqiangChen, All rights reserved.
* author  : XiaoqiangChen
* mail    : 309905109@qq.com
* history : 2018/09/26  1.0  new
'''

from attitude import attitude
class IntAttitude(attitude):
    """docstring for ClassName"""
    _phi0 = 0
    _theta0 = 0
    _psi0 = 0
    def __init__(self):
        super(IntAttitude, self).__init__()
        self._strategy = "Int method"

    def calculate_att(self):
        '''integration main cycle'''
        imu_data = self._data_set.get_imu_data()
        for imu in imu_data:
            self.integrate(imu)
            self.add_pitch_roll_yaw(self._theta0, self._phi0, self._psi0)

    def integrate(self, imu: tuple):
        d_t = imu[0][0]
        self._theta0 = self._theta0 + d_t*imu[0][1]
        self._phi0 = self._phi0 + d_t*imu[0][2]
        self._psi0 = self._psi0 + d_t*imu[0][3]

def main():
    '''test main'''
    att = IntAttitude()
    att.test()

if __name__ == '__main__':
    main()
