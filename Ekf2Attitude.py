'''
* main.py : Attitude caculator using ekf
*
*          Copyright (C) 2018 by XiaoqiangChen, All rights reserved.
* author  : XiaoqiangChen
* mail    : 309905109@qq.com
* history : 2018/09/26  1.0  new
'''

from CAttitude import Attitude

class EkfAttitude(Attitude):
    """docstring for EkfAttitude"""
    def __init__(self):
        super(EkfAttitude, self).__init__()
        self._strategy = "EKF";
