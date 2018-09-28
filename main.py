'''
* main.py : Attitude caculator caller
*
*          Copyright (C) 2018 by XiaoqiangChen, All rights reserved.
* author  : XiaoqiangChen
* mail    : 309905109@qq.com
* history : 2018/09/26  1.0  new
'''
import sys, getopt
import math
import matplotlib.pyplot as plt
from CAttitude import Attitude
from AccAttitude import AccAttitude
from IntAttitude import IntAttitude
from CompAttitude import CompAttitude
from Ekf2Attitude import EkfAttitude
from SO3Attitude import SO3Attitude

def printHelp():
    print ('main.py -s <sensorfile> -a <attfile> -m <method>')
    print('\t-m:')
    print('\t\ti:integration')
    print('\t\ta:accelameter')
    print('\t\tc:complementary')
    print('\t\te:ekf')
   
def AttitudeFactory(method:str) -> Attitude:
    if method == "i":
        return IntAttitude();
    elif method == "a":
        return AccAttitude();
    elif method == "c":
        return CompAttitude(0.6);
    elif method == "s":
        return SO3Attitude();
    elif method == "e":
        return EkfAttitude();

def main(argv):
    sensorfile = r'test\09_26_14_sensor_combined_0.csv';
    attfile = r'test\09_26_14_vehicle_attitude_0.csv'
    method = "c";

    try:
        opts, args = getopt.getopt(argv,"hs:a:m:",["sensorf=","attf=","method="])
    except getopt.GetoptError:
        print ('main.py -s <sensorfile> -a <attfile>')
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print ('main.py -s <sensorfile> -a <attfile>')
            sys.exit()
        elif opt in ("-s", "--sensorf"):
            sensorfile = arg
        elif opt in ("-a", "--attf"):
            attfile = arg
        elif opt in ("-m","--method"):
            method = arg

    att = AttitudeFactory(method);
    att.loadData(sensorfile,attfile);
    att.calculateAtt();
    att.showFig();

if __name__ == '__main__':
    main(sys.argv[1:]);