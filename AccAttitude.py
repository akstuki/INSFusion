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
        lsmag = zip(self._dataSet._lsMageX,self._dataSet._lsMageY,self._dataSet._lsMageZ);
        for sensordata in zip(lsacclemeter,lsmag):
            pitch,roll = self.accAtt(sensordata[0]);
            self._lsPitch.append(pitch);
            self._lsRoll.append(roll);
            self._lsYaw.append(self.magHeading(sensordata[1],roll,pitch));

def main():
    sensorfile = r'test\09_26_14_sensor_combined_0.csv';
    attfile = r'test\09_26_14_vehicle_attitude_0.csv'
    att = AccAttitude();
    att.loadData(sensorfile,attfile);
    att.calculateAtt();
    att.showFig();

if __name__ == '__main__':
    main()