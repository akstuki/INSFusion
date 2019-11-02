'''
* main.py : attitude caculator using pure gyro data integration
*
*          Copyright (C) 2018 by XiaoqiangChen, All rights reserved.
* author  : XiaoqiangChen
* mail    : 309905109@qq.com
* history : 2018/09/26  1.0  new
'''
from typing import NoReturn
from attitude import attitude

class IntAttitude(attitude):
    """docstring for ClassName"""
    _phi0 = 0
    _theta0 = 0
    _psi0 = 0
    def __init__(self):
        super(IntAttitude, self).__init__()
        self._strategy = "Int method"

    def calculate_att(self) -> NoReturn:
        '''integration main cycle'''
        imu_data = self._data_set.get_sensors_imu()
        for imu in imu_data:
            self.integrate(imu)
            self.add_pitch_roll_yaw(self._theta0, self._phi0, self._psi0)

    def integrate(self, imu: tuple):
        d_t = imu[0]
        gyros = imu[1]
        self._theta0 = self._theta0 + d_t*gyros._gyro_x
        self._phi0 = self._phi0 + d_t*gyros._gyro_y
        self._psi0 = self._psi0 + d_t*gyros._gyro_z

def main() -> NoReturn:
    '''test main'''
    att = IntAttitude()
    att.test()

if __name__ == '__main__':
    main()
