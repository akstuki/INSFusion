'''
* main.py : Attitude caculator using ekf
*
*          Copyright (C) 2018 by XiaoqiangChen, All rights reserved.
* author  : XiaoqiangChen
* mail    : 309905109@qq.com
* history : 2018/09/26  1.0  new
'''
# ?????
#  success at matlab .m code, when migrate to python still not work now

from CAttitude import Attitude
from numpy import *
import numpy as np
from lib.Quaternion import Quat2Euler
from enum import Enum

class States(Enum):
	X_q0 = 0
	X_q1 = 1
	X_q2 = 2
	X_q3 = 3
	X_bx = 4
	X_by = 5
	X_bz = 6
	n_x  = 7

class Obss(Enum):
	y_roll = 0
	y_pitch = 1
	y_yaw = 2
	n_y  = 3

class EkfAttitude(Attitude):
    """docstring for EkfAttitude"""
    def __init__(self):
        super(EkfAttitude, self).__init__()
        self._strategy = "EKF";
        self.reset();

    def reset(self):
        self._Xs = mat([1.0,0,0,0,1.0*1e-3,1.0*1e-3,1.0*1e-3]).T;
        self._P = 1e-1*mat(eye(7,7,dtype=float));
        self._R = 1e-3*mat(eye(3,3,dtype=float));
        self._Q = 1e-8*mat(eye(7,7,dtype=float));

    def obsMatrix(self,q0:float,q1:float,q2:float,q3:float) -> matrix:
        H = mat(zeros((3,7)));
        q1sqaq2sqmul2s1 =2*(q1**2) + 2*(q2**2) - 1;
        q1sqaq2sqmul2s1sq = q1sqaq2sqmul2s1**2;
        q0q1aq2q3mul2 = 2*q0*q1 + 2*q2*q3
        q0q1aq2q3mul2sq = q0q1aq2q3mul2**2;
        H[0,0] = -(2*q1)/((q0q1aq2q3mul2sq/q1sqaq2sqmul2s1sq + 1)*(q1sqaq2sqmul2s1));
        H[0,1] = -((2*q0)/(q1sqaq2sqmul2s1) - (4*q1*(q0q1aq2q3mul2))/q1sqaq2sqmul2s1sq)/(q0q1aq2q3mul2sq/q1sqaq2sqmul2s1sq + 1);
        H[0,2] = -((2*q3)/(q1sqaq2sqmul2s1) - (4*q2*(q0q1aq2q3mul2))/q1sqaq2sqmul2s1sq)/(q0q1aq2q3mul2sq/q1sqaq2sqmul2s1sq + 1);
        H[0,3] = -(2*q2)/((q0q1aq2q3mul2sq/q1sqaq2sqmul2s1sq + 1)*(q1sqaq2sqmul2s1));
        q0q2sq1q3mul2 = 2*q0*q2 - 2*q1*q3;
        q0q2sq1q3mul2sq = q0q2sq1q3mul2**2;
        q0q2sq1q3mul2sqsubby1 = (1 - q0q2sq1q3mul2sq)**0.5;
        H[1,0] =  (2*q2)/q0q2sq1q3mul2sqsubby1;
        H[1,1] = -(2*q3)/q0q2sq1q3mul2sqsubby1;
        H[1,2] =  (2*q0)/q0q2sq1q3mul2sqsubby1;
        H[1,3] = -(2*q1)/q0q2sq1q3mul2sqsubby1;
        q0q3aq1q2mul2 = 2*q0*q3 + 2*q1*q2;
        q0q3aq1q2mul2sq = q0q3aq1q2mul2**2;
        q2saq3smul2s1 = 2*(q2**2) + 2*(q3**2) - 1;
        q2saq3smul2s1sq = q2saq3smul2s1**2;
        H[2,0] = -(2*q3)/((q0q3aq1q2mul2sq/q2saq3smul2s1sq + 1)*(q2saq3smul2s1));
        H[2,1] = -(2*q2)/((q0q3aq1q2mul2sq/q2saq3smul2s1sq + 1)*(q2saq3smul2s1));
        H[2,2] = -((2*q1)/(q2saq3smul2s1) - (4*q2*(q0q3aq1q2mul2))/q2saq3smul2s1sq)/(q0q3aq1q2mul2sq/q2saq3smul2s1sq + 1);
        H[2,3] = -((2*q0)/(q2saq3smul2s1) - (4*q3*(q0q3aq1q2mul2))/q2saq3smul2s1sq)/(q0q3aq1q2mul2sq/q2saq3smul2s1sq + 1);
        return H;

    def accelemeterUpdate(self,accel:list,mag:list) -> (float,float,float):
        ax =  accel[0];
        ay =  accel[1];
        az =  accel[2];
        mx =  mag[0];
        my =  mag[1];
        mz =  mag[2];
        q0 = self._Xs[0,0];
        q1 = self._Xs[1,0];
        q2 = self._Xs[2,0];
        q3 = self._Xs[3,0];

        H = self.obsMatrix(self._Xs[0,0],self._Xs[1,0],self._Xs[2,0],self._Xs[3,0]);
        
        S = H*self._P*(H.T) + self._R;
        Kg = self._P* (H.T)*(S.I);

        pitch,roll = self.accAtt([ax,ay,az]);
        yaw = self.magHeading([mx,my,mz],pitch,roll);
        z = mat([roll,pitch,yaw]).T;
        recipNorm = 1.0/math.sqrt(q0**2 + q1**2 + q2**2 + q3**2);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;
        pitch_predit,roll_predict,yaw_predict = Quat2Euler(q0,q1,q2,q3);
        z_predict = mat([roll_predict,pitch_predit,yaw_predict]).T;
        self._Xs = self._Xs + Kg*(z - z_predict); 
        self._P = (eye(7)-Kg*H)*self._P;
        self._P = (self._P+self._P.T)*0.5;

        self._Xs[0,0] = q0;
        self._Xs[1,0] = q1;
        self._Xs[2,0] = q2;
        self._Xs[3,0] = q3;

        return pitch_predit,roll_predict,yaw_predict

    def calculateAtt(self):
        lsGyros = zip(self._dataSet._lsDeltT,self._dataSet._lsGyroX,self._dataSet._lsGyroY,self._dataSet._lsGyroZ);
        lsAccs = zip(self._dataSet._lsAcclX,self._dataSet._lsAcclY,self._dataSet._lsAcclZ);
        lsmag = zip(self._dataSet._lsMageX,self._dataSet._lsMageY,self._dataSet._lsMageZ);
         
        for imu in zip(lsGyros,lsAccs,lsmag):
            wx = imu[0][1];
            wy = imu[0][2];
            wz = imu[0][3];
            dt =  imu[0][0]          
            qs = self._Xs[0,0];
            qx = self._Xs[1,0];
            qy = self._Xs[2,0];
            qz = self._Xs[3,0];

            # ---predict-----
            JokbiA = mat(zeros((7,7)));
            JokbiA[0,:] = [0 ,-wx,-wy,-wz, qx, qy, qz];
            JokbiA[1,:] = [wx, 0 , wz,-wy,-qs, qz,-qy];
            JokbiA[2,:] = [wy,-wz, 0 , wx,-qz,-qs, qx];
            JokbiA[3,:] = [wz, wy,-wx,  0, qy,-qx,-qs];
     
            A = eye(7) + 0.5*JokbiA*dt;
            self._Xs = A*self._Xs;
            self._P = A*self._P*A.T + self._Q;

            # ---update----
            pitch_predit,roll_predict,yaw_predict = self.accelemeterUpdate(imu[1],imu[2]);

            if math.isnan(pitch_predit) or math.isnan(roll_predict) or math.isnan(yaw_predict):
            	print('reset');
            	self.reset();

            self._lsPitch.append(pitch_predit);
            self._lsRoll.append(roll_predict);
            self._lsYaw.append(yaw_predict);
            
def main():
    sensorfile = r'test\09_26_14_sensor_combined_0.csv';
    attfile = r'test\09_26_14_vehicle_attitude_0.csv'
    att = EkfAttitude();
    att.loadData(sensorfile,attfile);
    att.calculateAtt();
    att.showFig();

if __name__ == '__main__':
    main()