'''
* main.py : attitude caculator caller
*
*          Copyright (C) 2018 by XiaoqiangChen, All rights reserved.
* author  : XiaoqiangChen
* mail    : 309905109@qq.com
* history : 2018/09/26  1.0  new
*           2018/09/29  1.0  changed to google python style
'''
import sys
import getopt
from attitude import attitude
from accattitude import AccAttitude
from intattitude import IntAttitude
from comattitude import CompAttitude
from ekfattitude import EkfAttitude
from so3attitude import SO3Attitude
from typing import NoReturn

def print_help() -> NoReturn:
    '''
    print cmd help doc
    '''
    print('main.py -s <sensorfile> -a <attfile> -m <method>')
    print('\t-m:')
    print('\t\ti:integration')
    print('\t\ta:accelameter')
    print('\t\tc:complementary')
    print('\t\ts:so3')
    print('\t\te:ekf')

def attitude_factory(method: str) -> attitude:
    '''
    construct the attitude caculator object
    '''
    att = None
    if method == "i":
        att = IntAttitude()
    elif method == "a":
        att = AccAttitude()
    elif method == "c":
        att = CompAttitude(0.6)
    elif method == "s":
        att = SO3Attitude()
    elif method == "e":
        att = EkfAttitude()
    else:
        pass
    return att

def main(argv: list) -> NoReturn:
    '''main.'''
    sensorfile = r'test\09_26_14_sensor_combined_0.csv'
    attfile = r'test\09_26_14_vehicle_attitude_0.csv'
    method = "i"

    try:
        opts, _ = getopt.getopt(argv, "hs:a:m:", ["sensorf=", "attf=", "method="])
    except getopt.GetoptError:
        print_help()
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print_help()
            sys.exit()
        elif opt in ("-s", "--sensorf"):
            sensorfile = arg
        elif opt in ("-a", "--attf"):
            attfile = arg
        elif opt in ("-m", "--method"):
            method = arg

    att = attitude_factory(method)
    att.load_data(sensorfile, attfile)
    att.calculate_att()
    att.show_fig()

if __name__ == '__main__':
    main(sys.argv[1:])
