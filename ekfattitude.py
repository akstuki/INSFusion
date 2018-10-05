'''
* main.py : attitude caculator using ekf
*
*          Copyright (C) 2018 by XiaoqiangChen, All rights reserved.
* author  : XiaoqiangChen
* mail    : 309905109@qq.com
* history : 2018/09/26  1.0  new
'''
# ?????
#  success at matlab .m code, when migrate to python still not work now

import math
from numpy import zeros
from numpy import matrix
from numpy import mat
from numpy import eye
from lib.quaternion import quat2euler
from attitude import attitude
from attitude import acc_att
from attitude import mag_heading

class EkfAttitude(attitude):
    """docstring for EkfAttitude"""
    def __init__(self):
        # pylint: disable=invalid-name
        super(EkfAttitude, self).__init__()
        self._strategy = "EKF"
        self._Xs = mat([1.0, 0, 0, 0, 1.0*1e-3, 1.0*1e-3, 1.0*1e-3]).T
        self._P = 1e-1*mat(eye(7, 7, dtype=float))
        self._R = 1e-3*mat(eye(3, 3, dtype=float))
        self._Q = 1e-7*mat(eye(7, 7, dtype=float))

    def reset(self):
        '''reset kalman filter to init state'''
        self._Xs = mat([1.0, 0, 0, 0, 1.0*1e-3, 1.0*1e-3, 1.0*1e-3]).T
        self._P = 1e-1*mat(eye(7, 7, dtype=float))
        self._R = 1e-3*mat(eye(3, 3, dtype=float))
        self._Q = 1e-7*mat(eye(7, 7, dtype=float))

    def accel_correct(self, accel: list, mag: list):
        '''measurement update - accel'''
        # pylint: disable=too-many-locals
        h_k = obsMatrix(self._Xs[0, 0], self._Xs[1, 0], self._Xs[2, 0], self._Xs[3, 0])

        s_k = h_k*self._P*(h_k.T) + self._R
        k_k = self._P* (h_k.T)*(s_k.I)

        pitch, roll = acc_att(accel)
        yaw = mag_heading(mag, pitch, roll)
        y_k = mat([roll, pitch, yaw]).T

        recip_norm = 1.0/math.sqrt(self._Xs[0, 0]**2 + self._Xs[1, 0]**2 + self._Xs[2, 0]**2 \
            + self._Xs[3, 0]**2)
        self._Xs[0, 0] *= recip_norm
        self._Xs[1, 0] *= recip_norm
        self._Xs[2, 0] *= recip_norm
        self._Xs[3, 0] *= recip_norm

        pitch_predit, roll_predict, yaw_predict = \
        quat2euler(self._Xs[0, 0], self._Xs[1, 0], self._Xs[2, 0], self._Xs[3, 0])
        y_predict = mat([roll_predict, pitch_predit, yaw_predict]).T

        self._Xs = self._Xs + k_k*(y_k - y_predict)
        self._P = (mat(eye(7, 7, dtype=float))-k_k*h_k)*self._P
        self._P = (self._P+self._P.T)*0.5

    def predict(self, imu: list):
        '''time update of kalman filter'''
        w_x = imu[1]
        w_y = imu[2]
        w_z = imu[3]
        d_t = imu[0]
        q_s = self._Xs[0, 0]
        q_x = self._Xs[1, 0]
        q_y = self._Xs[2, 0]
        q_z = self._Xs[3, 0]

        # ---predict-----
        jkbi_a = mat(zeros((7, 7)))
        jkbi_a[0, :] = [0, -w_x, -w_y, -w_z, q_x, q_y, q_z]
        jkbi_a[1, :] = [w_x, 0, w_z, -w_y, -q_s, q_z, -q_y]
        jkbi_a[2, :] = [w_y, -w_z, 0, w_x, -q_z, -q_s, q_x]
        jkbi_a[3, :] = [w_z, w_y, -w_x, 0, q_y, -q_x, -q_s]

        f_k = mat(eye(7, 7, dtype=float)) + 0.5*jkbi_a*d_t
        self._Xs = f_k*self._Xs
        self._P = f_k*self._P*(f_k.T) + self._Q

    def calculate_att(self):
        '''kalman filter main cycle'''
        imu_data = self._data_set.get_imu_data()
        for imu in imu_data:
            self.predict(imu[0])

            # ---update----
            self.accel_correct(imu[1], imu[2])

            pitch_hat, roll_hat, yaw_hat = \
            quat2euler(self._Xs[0, 0], self._Xs[1, 0], self._Xs[2, 0], self._Xs[3, 0])
            if math.isnan(pitch_hat) or math.isnan(roll_hat) or math.isnan(yaw_hat):
                print('reset')
                self.reset()

            self.add_pitch_roll_yaw(pitch_hat, roll_hat, yaw_hat)

def obsMatrix(q0: float, q1: float, q2: float, q3: float) -> matrix:
    '''accel measurement obs matrix caculation'''
    # pylint: disable=invalid-name
    # pylint: disable=line-too-long
    # pylint: disable=too-many-locals
    H = mat(zeros((3, 7)))
    q1sqaq2sqmul2s1 = 2*(q1**2) + 2*(q2**2) - 1
    q1sqaq2sqmul2s1sq = q1sqaq2sqmul2s1**2
    q0q1aq2q3mul2 = 2*q0*q1 + 2*q2*q3
    q0q1aq2q3mul2sq = q0q1aq2q3mul2**2
    H[0, 0] = -(2*q1)/((q0q1aq2q3mul2sq/q1sqaq2sqmul2s1sq + 1)*(q1sqaq2sqmul2s1))
    H[0, 1] = -((2*q0)/(q1sqaq2sqmul2s1) - (4*q1*(q0q1aq2q3mul2))/q1sqaq2sqmul2s1sq)/(q0q1aq2q3mul2sq/q1sqaq2sqmul2s1sq + 1)
    H[0, 2] = -((2*q3)/(q1sqaq2sqmul2s1) - (4*q2*(q0q1aq2q3mul2))/q1sqaq2sqmul2s1sq)/(q0q1aq2q3mul2sq/q1sqaq2sqmul2s1sq + 1)
    H[0, 3] = -(2*q2)/((q0q1aq2q3mul2sq/q1sqaq2sqmul2s1sq + 1)*(q1sqaq2sqmul2s1))
    q0q2sq1q3mul2 = 2*q0*q2 - 2*q1*q3
    q0q2sq1q3mul2sq = q0q2sq1q3mul2**2
    q0q2sq1q3mul2sq_subby1 = (1 - q0q2sq1q3mul2sq)**0.5
    H[1, 0] = (2*q2)/q0q2sq1q3mul2sq_subby1
    H[1, 1] = -(2*q3)/q0q2sq1q3mul2sq_subby1
    H[1, 2] = (2*q0)/q0q2sq1q3mul2sq_subby1
    H[1, 3] = -(2*q1)/q0q2sq1q3mul2sq_subby1
    q0q3aq1q2mul2 = 2*q0*q3 + 2*q1*q2
    q0q3aq1q2mul2sq = q0q3aq1q2mul2**2
    q2saq3smul2s1 = 2*(q2**2) + 2*(q3**2) - 1
    q2saq3smul2s1sq = q2saq3smul2s1**2
    H[2, 0] = -(2*q3)/((q0q3aq1q2mul2sq/q2saq3smul2s1sq + 1)*(q2saq3smul2s1))
    H[2, 1] = -(2*q2)/((q0q3aq1q2mul2sq/q2saq3smul2s1sq + 1)*(q2saq3smul2s1))
    H[2, 2] = -((2*q1)/(q2saq3smul2s1) - (4*q2*(q0q3aq1q2mul2))/q2saq3smul2s1sq)/(q0q3aq1q2mul2sq/q2saq3smul2s1sq + 1)
    H[2, 3] = -((2*q0)/(q2saq3smul2s1) - (4*q3*(q0q3aq1q2mul2))/q2saq3smul2s1sq)/(q0q3aq1q2mul2sq/q2saq3smul2s1sq + 1)
    return H

def main():
    '''main test'''
    att = EkfAttitude()
    att.test()

if __name__ == '__main__':
    main()
