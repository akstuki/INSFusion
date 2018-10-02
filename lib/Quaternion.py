'''
* main.py : Attitude caculator math lib
*
*          Copyright (C) 2018 by XiaoqiangChen, All rights reserved.
* author  : XiaoqiangChen
* mail    : 309905109@qq.com
* history : 2018/09/26  1.0  new
'''

import math

M_PI = 3.14159265358979323846

def quat2dcm(q_0: float, q_1: float, q_2: float, q_3: float)->list:
    '''
    DCM=
    dcm[0] dcm[1] dcm[2]
    dcm[3] dcm[4] dcm[5]
    dcm[6] dcm[7] dcm[8]
    '''
    dcm = []
    q0_sq = q_0*q_0
    q1_sq = q_1*q_1
    q2_sq = q_2*q_2
    q3_sq = q_3*q_3
    dcm.append(q0_sq + q1_sq - q2_sq - q3_sq)
    dcm.append(2 * (q_1 * q_2 - q_0 * q_3))
    dcm.append(2 * (q_0 * q_2 + q_1 * q_3))
    dcm.append(2 * (q_1 * q_2 + q_0 * q_3))
    dcm.append(q0_sq - q1_sq + q2_sq - q3_sq)
    dcm.append(2 * (q_2 * q_3 - q_0 * q_1))
    dcm.append(2 * (q_1 * q_3 - q_0 * q_2))
    dcm.append(2 * (q_0 * q_1 + q_2 * q_3))
    dcm.append(q0_sq - q1_sq - q2_sq + q3_sq)
    return dcm

def dcm2euler(dcm: list) -> (float, float, float):
    '''
    DCM=
    dcm[0] dcm[1] dcm[2]
    dcm[3] dcm[4] dcm[5]
    dcm[6] dcm[7] dcm[8]
    '''
    phi_val = math.atan2(dcm[7], dcm[8])
    theta_val = math.asin(-dcm[6])
    psi_val = math.atan2(dcm[3], dcm[0])

    if math.fabs(theta_val - M_PI / 2) < 1.0e-3:
        phi_val = 0.0
        psi_val = math.atan2(dcm[5], dcm[2])
    elif math.fabs(theta_val + M_PI / 2) < 1.0e-3:
        phi_val = 0.0
        psi_val = math.atan2(-dcm[5], -dcm[2])

    return phi_val, theta_val, psi_val

def quat2euler(q_0: float, q_1: float, q_2: float, q_3: float):
    '''convert quat to euler angle -> rad'''
    dcm = quat2dcm(q_0, q_1, q_2, q_3)
    return dcm2euler(dcm)
