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
    def __init__(self):
        super(Attitude, self).__init__()
    
    def loadData(self,filename:str,attFileName:str):
        self._dataSet = DataSet(filename,attFileName);
        self._dataSet.LoadIMUData();
        self._dataSet.loadEkfAtt();

    def accAtt(self,acc:list):
        norm = math.fabs(acc[2]);
        pitch = math.atan2(acc[1],norm);
        roll = math.atan2(acc[0],norm);
        return pitch,roll;
    def calculateAtt(self):
        pass

    def showFig(self):
        if len(self._lsPitch) == 0:
            print('no result');
            return;

        plt.figure(1);
        plt.subplot(211);
        plt.plot(self._dataSet._lsTimes,self._lsPitch,label="pitch");
        plt.plot(self._dataSet._lsEkfTimes ,self._dataSet._lsEkfPitch,label="EKFpitch");
        plt.ylabel('attitude(rad)')
        plt.title(self._strategy);
        plt.legend()
        plt.grid(True);

        plt.subplot(212);
        plt.plot(self._dataSet._lsTimes,self._lsRoll,label="roll");
        plt.plot(self._dataSet._lsEkfTimes,self._dataSet._lsEkfRoll,label="EKFroll");
        plt.xlabel('time(s)')
        plt.grid(True);
        plt.show();

def main():
    att = Attitude('09_26_14_sensor_combined_0.csv');

if __name__ == '__main__':
    main()