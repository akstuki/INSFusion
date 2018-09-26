'''
* main.py : Attitude caculator using accelmeter data
*
*          Copyright (C) 2018 by XiaoqiangChen, All rights reserved.
* author  : XiaoqiangChen
* mail    : 309905109@qq.com
* history : 2018/09/26  1.0  new
'''

from CAttitude import Attitude

class AccAttitude(Attitude):
    """docstring for ClassName"""
    def __init__(self):
        super(AccAttitude, self).__init__()
        self._strategy = "accelerometer method";

    def calculateAtt(self):
        lsacclemeter = zip(self._dataSet._lsAcclX,self._dataSet._lsAcclY,self._dataSet._lsAcclZ);
        for acc in lsacclemeter:
            pitch,roll = self.accAtt(acc);
            self._lsPitch.append(pitch);
            self._lsRoll.append(roll);