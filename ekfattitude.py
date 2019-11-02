'''
* main.py : attitude caculator using ekf
*
*          Copyright (C) 2018 by XiaoqiangChen, All rights reserved.
* author  : XiaoqiangChen
* mail    : 309905109@qq.com
* history : 2018/09/26  1.0  new
'''
'''
Model
 states:
   attitude quaternion
   Delta Angle bias - rad (X,Y,Z)

 measurements:
  attitude euler angle get by below meas:
      ax, ay, az (acceleration body)
      XYZ magnetic flux
'''

'''Steps for EKF: 
*  Prediction: 
*             (1) Predicted State Estimate
*             (2) Predicted Covariance Estimate
*  Update:
*             (1) Innovation Covariance Calculation
*             (2) Kalman Gain Calculation
*             (3) Update State Estimate
*             (4) Update Covariance Estimate
*            (Steps copied from wikipedia)
*
* '''

import math
from numpy import zeros
from numpy import matrix
from numpy import mat
from numpy import eye
from lib.quaternion import quat2euler
from attitude import attitude
from attitude import acc_att
from attitude import mag_heading
from accelerometer import accelerometer
from gyroscope import gyroscope
from magnetometer import magnetometer
from typing import NoReturn

class EkfAttitude(attitude):
    """docstring for EkfAttitude"""
    def __init__(self):
        # pylint: disable=invalid-name
        super(EkfAttitude, self).__init__()
        self._strategy = "EKF"
        # _Xs = [q0, q1, q2, q3, p, q, r]^T
        self._Xs = mat([1.0, 0, 0, 0, 1.0*1e-3, 1.0*1e-3, 1.0*1e-3]).T
        self._P = 1e-1*mat(eye(7, 7, dtype=float))
        self._R = 1e-3*mat(eye(3, 3, dtype=float))
        self._Q = 1e-7*mat(eye(7, 7, dtype=float))

    def reset(self) -> NoReturn:
        '''reset kalman filter to init state'''
        self._Xs = mat([1.0, 0, 0, 0, 1.0*1e-3, 1.0*1e-3, 1.0*1e-3]).T
        self._P = 1e-1*mat(eye(7, 7, dtype=float))
        self._R = 1e-3*mat(eye(3, 3, dtype=float))
        self._Q = 1e-7*mat(eye(7, 7, dtype=float))

    def accel_correct(self, accel: accelerometer, mag: magnetometer) -> NoReturn:
        '''measurement update - accel'''
        # pylint: disable=too-many-locals
        h_k = obsMatrix(self._Xs[0, 0], self._Xs[1, 0], self._Xs[2, 0], self._Xs[3, 0])
        #Innovation Covariance Calculation
        s_k = h_k*self._P*(h_k.T) + self._R
        #Kalman Gain Calculation
        k_k = self._P* (h_k.T)*(s_k.I)

        pitch, roll = accel.acc_att()
        yaw = mag.mag_heading(pitch, roll)
        y_k = mat([roll, pitch, yaw]).T

        self.nomilize_state()

        pitch_predit, roll_predict, yaw_predict = \
        quat2euler(self._Xs[0, 0], self._Xs[1, 0], self._Xs[2, 0], self._Xs[3, 0])
        y_predict = mat([roll_predict, pitch_predit, yaw_predict]).T

        #Update State Estimate
        self._Xs = self._Xs + k_k*(y_k - y_predict)
        self.nomilize_state()

        #Update Covariance Estimate
        self._P = (mat(eye(7, 7, dtype=float))-k_k*h_k)*self._P
        self._P = (self._P+self._P.T)*0.5

    def nomilize_state(self) -> NoReturn:
        recip_norm = 1.0/math.sqrt(self._Xs[0, 0]**2 + self._Xs[1, 0]**2 + self._Xs[2, 0]**2 \
            + self._Xs[3, 0]**2)
        self._Xs[0, 0] *= recip_norm
        self._Xs[1, 0] *= recip_norm
        self._Xs[2, 0] *= recip_norm
        self._Xs[3, 0] *= recip_norm

    def predict(self, d_t: float, gyros: gyroscope) -> NoReturn:
        '''time update of kalman filter'''
        w_x = gyros._gyro_x*d_t
        w_y = gyros._gyro_y*d_t
        w_z = gyros._gyro_z*d_t
        q_s = self._Xs[0, 0]
        q_x = self._Xs[1, 0]
        q_y = self._Xs[2, 0]
        q_z = self._Xs[3, 0]

        # ---predict-----
        ''' *  q(t+1) = exp(1/2*delta_theta) * q(t)
        *  q(t+1) = (I + delta_theta / 2) * q(t) *******Taylor Expansion
        *  delta_theta = theta(t+1) - theta(t)
        *              = jkbi_a * _Xs * dt
        ''' 
     
        jkbi_a = mat(zeros((7, 7)))
        jkbi_a[0, :] = [0, -w_x, -w_y, -w_z, q_x, q_y, q_z]
        jkbi_a[1, :] = [w_x, 0, w_z, -w_y, -q_s, q_z, -q_y]
        jkbi_a[2, :] = [w_y, -w_z, 0, w_x, -q_z, -q_s, q_x]
        jkbi_a[3, :] = [w_z, w_y, -w_x, 0, q_y, -q_x, -q_s]

        f_k = mat(eye(7, 7, dtype=float)) + 0.5*jkbi_a*d_t
        #Predicted state estimate
        self._Xs = f_k*self._Xs
        #predicted covariance estimate
        self._P = f_k*self._P*(f_k.T) + self._Q

    def calculate_att(self) -> NoReturn:
        '''kalman filter main cycle'''
        imu_data = self._data_set.get_sensors_imu()
        for imu in imu_data:
            self.predict(imu[0], imu[1])

            # ---update----
            self.accel_correct(imu[2], imu[3])

            q0, q1, q2, q3 = self._Xs[0, 0], self._Xs[1, 0], self._Xs[2, 0], self._Xs[3, 0]
            pitch_hat, roll_hat, yaw_hat = quat2euler(q0, q1, q2, q3)
            if math.isnan(pitch_hat) or math.isnan(roll_hat) or math.isnan(yaw_hat):
                print('reset')
                self.reset()

            self.add_pitch_roll_yaw(pitch_hat, roll_hat, yaw_hat)

def obsMatrix(q0: float, q1: float, q2: float, q3: float) -> matrix:
    '''accel measurement obs matrix caculation'''
    # pylint: disable=invalid-name
    # pylint: disable=line-too-long
    # pylint: disable=too-many-locals
    
    '''
    *      y   = [roll, pitch, yaw]
    *    [phi  =   [atctan(2 * (q0 * q1 + q2 * q3)/(1 - 2 * (q1^2 + q2^2)))
    *     theta    arcsin(2(q0 * q2 - q3 * q1))
    *     psi]     arctan(2 * (q0_q3 + q1 * q2)/(1 - 2 * (q2^2 + q3^2)))]
    '''
    H = mat(zeros((3, 7)))
    q1sqaq2sqmul2s1 = 2*(q1**2) + 2*(q2**2) - 1
    q1sqaq2sqmul2s1sq = q1sqaq2sqmul2s1**2
    q0q1aq2q3mul2 = 2*q0*q1 + 2*q2*q3
    q0q1aq2q3mul2sq = q0q1aq2q3mul2**2
    H[1, 0] = -(2*q1)/((q0q1aq2q3mul2sq/q1sqaq2sqmul2s1sq + 1)*(q1sqaq2sqmul2s1))
    H[1, 1] = -((2*q0)/(q1sqaq2sqmul2s1) - (4*q1*(q0q1aq2q3mul2))/q1sqaq2sqmul2s1sq)/(q0q1aq2q3mul2sq/q1sqaq2sqmul2s1sq + 1)
    H[1, 2] = -((2*q3)/(q1sqaq2sqmul2s1) - (4*q2*(q0q1aq2q3mul2))/q1sqaq2sqmul2s1sq)/(q0q1aq2q3mul2sq/q1sqaq2sqmul2s1sq + 1)
    H[1, 3] = -(2*q2)/((q0q1aq2q3mul2sq/q1sqaq2sqmul2s1sq + 1)*(q1sqaq2sqmul2s1))
    q0q2sq1q3mul2 = 2*q0*q2 - 2*q1*q3
    q0q2sq1q3mul2sq = q0q2sq1q3mul2**2
    q0q2sq1q3mul2sq_subby1 = (1 - q0q2sq1q3mul2sq)**0.5
    H[0, 0] = (2*q2)/q0q2sq1q3mul2sq_subby1
    H[0, 1] = -(2*q3)/q0q2sq1q3mul2sq_subby1
    H[0, 2] = (2*q0)/q0q2sq1q3mul2sq_subby1
    H[0, 3] = -(2*q1)/q0q2sq1q3mul2sq_subby1
    q0q3aq1q2mul2 = 2*q0*q3 + 2*q1*q2
    q0q3aq1q2mul2sq = q0q3aq1q2mul2**2
    q2saq3smul2s1 = 2*(q2**2) + 2*(q3**2) - 1
    q2saq3smul2s1sq = q2saq3smul2s1**2
    H[2, 0] = -(2*q3)/((q0q3aq1q2mul2sq/q2saq3smul2s1sq + 1)*(q2saq3smul2s1))
    H[2, 1] = -(2*q2)/((q0q3aq1q2mul2sq/q2saq3smul2s1sq + 1)*(q2saq3smul2s1))
    H[2, 2] = -((2*q1)/(q2saq3smul2s1) - (4*q2*(q0q3aq1q2mul2))/q2saq3smul2s1sq)/(q0q3aq1q2mul2sq/q2saq3smul2s1sq + 1)
    H[2, 3] = -((2*q0)/(q2saq3smul2s1) - (4*q3*(q0q3aq1q2mul2))/q2saq3smul2s1sq)/(q0q3aq1q2mul2sq/q2saq3smul2s1sq + 1)
    return H

def main() -> NoReturn:
    '''main test'''
    att = EkfAttitude()
    att.test()

if __name__ == '__main__':
    main()
