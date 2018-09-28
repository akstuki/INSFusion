'''
* main.py : Attitude caculator math lib
*
*          Copyright (C) 2018 by XiaoqiangChen, All rights reserved.
* author  : XiaoqiangChen
* mail    : 309905109@qq.com
* history : 2018/09/26  1.0  new
'''

import math

M_PI = 3.14159265358979323846;

def Quat2DCM(q0:float,q1:float,q2:float,q3:float)->list:
    '''
    DCM=
    lsDCM[0] lsDCM[1] lsDCM[2]
    lsDCM[3] lsDCM[4] lsDCM[5]
    lsDCM[6] lsDCM[7] lsDCM[8]
    '''
    lsDCM = [];
    a = q0;
    b = q1;
    c = q2;
    d = q3;
    aSq = a*a;
    bSq = b*b;
    cSq = c*c;
    dSq = d*d;
    lsDCM.append(aSq + bSq - cSq - dSq);
    lsDCM.append(2 * (b * c - a * d));
    lsDCM.append(2 * (a * c + b * d));
    lsDCM.append(2 * (b * c + a * d));
    lsDCM.append(aSq - bSq + cSq - dSq);
    lsDCM.append(2 * (c * d - a * b));
    lsDCM.append(2 * (b * d - a * c));
    lsDCM.append(2 * (a * b + c * d));
    lsDCM.append(aSq - bSq - cSq + dSq);
    return lsDCM;

def DCM2Euler(lsDCM:list) -> (float,float,float):
    '''
    DCM=
    lsDCM[0] lsDCM[1] lsDCM[2]
    lsDCM[3] lsDCM[4] lsDCM[5]
    lsDCM[6] lsDCM[7] lsDCM[8]
    '''
    phi_val = math.atan2(lsDCM[7], lsDCM[8]);
    theta_val = math.asin(-lsDCM[6]);
    psi_val = math.atan2(lsDCM[3], lsDCM[0]);
    pi = M_PI;

    if math.fabs(theta_val - pi / 2) < 1.0e-3:
        phi_val = 0.0;
        psi_val = math.atan2(lsDCM[5], lsDCM[2]);
    elif math.fabs(theta_val + pi / 2) < 1.0e-3:
        phi_val = 0.0;
        psi_val = atan2(-lsDCM[5], -lsDCM[2]);

    return phi_val,theta_val,psi_val;

def Quat2Euler(q0:float,q1:float,q2:float,q3:float):
    dcm = Quat2DCM(q0,q1,q2,q3);
    return DCM2Euler(dcm);