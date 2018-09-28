'''
* main.py : Attitude caculator parent class
*
*          Copyright (C) 2018 by XiaoqiangChen, All rights reserved.
* author  : XiaoqiangChen
* mail    : 309905109@qq.com
* history : 2018/09/26  1.0  new
'''

from DataSet import DataSet
import math
import matplotlib.pyplot as plt

class Attitude(object):
    """docstring for Attitude"""
    _strategy = "none";
    _dataSet = None;

    _lsPitch = [];
    _lsRoll = [];
    _lsYaw = [];
    def __init__(self):
        super(Attitude, self).__init__()
    
    def loadData(self,filename:str,attFileName:str):
        self._dataSet = DataSet(filename,attFileName);
        self._dataSet.LoadIMUData();
        self._dataSet.loadEkfAtt();

    def accAtt(self,acc:list) -> tuple:
        norm = math.sqrt(acc[0]**2 +acc[2]**2);
        pitch = math.atan2(acc[1],norm);
        roll = math.atan2(acc[0],-acc[2]);
        return pitch,roll;

    def magHeading(self,mag:list,r:float,p:float) -> float:
        Mn_x = mag[1]*math.cos(p)-mag[2]*math.sin(p);
        Mn_y = mag[0]*math.cos(r)+mag[1]*math.sin(r)*math.sin(p)+mag[2]*math.sin(r)*math.cos(p);
        return -math.atan2(Mn_x,Mn_y);


    def calculateAtt(self):
        pass

    def showFig(self):
        if len(self._lsPitch) == 0:
            print('no result');
            return;

        plt.figure(1);
        plt.subplot(311);
        plt.plot(self._dataSet._lsTimes,self._lsPitch,label="pitch");
        plt.plot(self._dataSet._lsEkfTimes ,self._dataSet._lsEkfPitch,label="EKF2");
        plt.ylabel('pitch(rad)')
        plt.title(self._strategy);
        plt.legend()
        plt.grid(True);

        plt.subplot(312);
        plt.plot(self._dataSet._lsTimes,self._lsRoll,label="roll");
        plt.plot(self._dataSet._lsEkfTimes,self._dataSet._lsEkfRoll,label="EKF2");
        plt.ylabel('roll(rad)')
        plt.xlabel('time(s)')
        plt.legend()
        plt.grid(True);

        plt.subplot(313);
        plt.plot(self._dataSet._lsTimes,self._lsYaw,label="yaw");
        plt.plot(self._dataSet._lsEkfTimes,self._dataSet._lsEkfYaw,label="EKF2");
        plt.ylabel('yaw(rad)')
        plt.xlabel('time(s)')
        plt.legend()
        plt.grid(True);

        plt.show();

def main():
    att = Attitude();

if __name__ == '__main__':
    main()