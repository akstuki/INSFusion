'''
* main.py : attitude caculator using SO3
*
*          Copyright (C) 2018 by XiaoqiangChen, All rights reserved.
* author  : XiaoqiangChen
* mail    : 309905109@qq.com
* history : 2018/09/26  1.0  new
'''
import math
from attitude import attitude
from attitude import acc_att
from attitude import mag_heading
from lib.quaternion import dcm2euler
from lib.quaternion import quat2dcm
from lib.quaternion import quat2euler
from accelerometer import accelerometer
from gyroscope import gyroscope
from magnetometer import magnetometer

class SO3Attitude(attitude):
    """docstring for SO3Attitude"""

    # pylint: disable=too-many-instance-attributes
    _initialized = False

    def __init__(self):
        super(SO3Attitude, self).__init__()
        self._strateg_y = "SO3"
        self._gyro_offset_x = 0
        self._gyro_offset_y = 0
        self._gyro_offset_z = 0
        self._gyro_offset_count = 0
        self._gyro_bias_x = 0
        self._gyro_bias_y = 0
        self._gyro_bias_z = 0
        self.q_0 = 1.0
        self.q_1 = 0.0
        self.q_2 = 0.0
        self.q_3 = 0.0
        self.dq0 = 0.0
        self.dq1 = 0.0
        self.dq2 = 0.0
        self.dq3 = 0.0
        self.q0q0 = 0.0
        self.q0q1 = 0.0
        self.q0q2 = 0.0
        self.q0q3 = 0.0
        self.q1q1 = 0.0
        self.q1q2 = 0.0
        self.q1q3 = 0.0
        self.q2q2 = 0.0
        self.q2q3 = 0.0
        self.q3q3 = 0.0
        self._filter_inited = False
        self._two_ki = 0.05
        self._two_kp = 1.0

    def calculate_att(self):
        '''so3 main cycle'''
        imu_data = self._data_set.get_sensors_imu()
        for imu in imu_data:
            pitch, roll, yaw = self.so3_pitch_roll_yaw(imu)
            self.add_pitch_roll_yaw(pitch, roll, yaw)

    def so3_pitch_roll_yaw(self, imu: tuple) -> (float, float, float):
        ''' '''
        pitch = None
        roll = None
        yaw = None
        if not self._initialized:
            self.calibrate_gyros(imu[1])     
        else:
            gyros = self.add_gyro_bias(imu[1])
            delta_t = imu[0]
            # resver_accel = [a*(-1.0) for a in imu[1]]
            self.so3_update(gyros, imu[2], imu[3], delta_t)
            
            pitch, roll, yaw = quat2euler(self.q_0, self.q_1, self.q_2, self.q_3)
        return pitch, roll, yaw
    
    def calibrate_gyros(self, gyros: gyroscope):
        ''' '''
        self._gyro_offset_x += gyros._gyro_x
        self._gyro_offset_y += gyros._gyro_y
        self._gyro_offset_z += gyros._gyro_z
        self._gyro_offset_count += 1
        if self._gyro_offset_count == 1000:
            self._initialized = True
            self._gyro_offset_x /= self._gyro_offset_count
            self._gyro_offset_y /= self._gyro_offset_count
            self._gyro_offset_z /= self._gyro_offset_count

    def add_gyro_bias(self, gyros: gyroscope) -> list:
        gyro_x = gyros._gyro_x - self._gyro_offset_x
        gyro_y = gyros._gyro_y - self._gyro_offset_y
        gyro_z = gyros._gyro_z - self._gyro_offset_z
        return [gyro_x, gyro_y, gyro_z]

    def maga_correct(self, maga: magnetometer, halfex: float, halfey: float, halfez: float) -> tuple:
        '''direction of magnetic field to correct gyro'''
        # pylint: disable=too-many-locals
        if maga.is_valid():
            m_x, m_y, m_z = maga.normalised()

            # Reference direction of Earth's magnetic field
            h_x = 2.0*(m_x * (0.5-self.q2q2-self.q3q3) + \
                m_y * (self.q1q2-self.q0q3) + m_z * (self.q1q3+self.q0q2))
            h_y = 2.0 * (m_x * (self.q1q2+self.q0q3) + \
                m_y * (0.5-self.q1q1-self.q3q3) + m_z * (self.q2q3-self.q0q1))
            h_z = 2.0 * m_x * (self.q1q3-self.q0q2) + 2.0 * m_y * (self.q2q3+self.q0q1) \
            + 2.0 * m_z * (0.5-self.q1q1-self.q2q2)
            b_x = math.sqrt(h_x * h_x + h_y * h_y)
            b_z = h_z

            # Estimated direction of magnetic field
            halfwx = b_x * (0.5 - self.q2q2 - self.q3q3) + b_z * (self.q1q3 - self.q0q2)
            halfwy = b_x * (self.q1q2 - self.q0q3) + b_z * (self.q0q1 + self.q2q3)
            halfwz = b_x * (self.q0q2 + self.q1q3) + b_z * (0.5 - self.q1q1 - self.q2q2)

            # Error is sum of cross product between estimated direction
            # and measured direction of field vectors
            halfex += (m_y * halfwz - m_z * halfwy)
            halfey += (m_z * halfwx - m_x * halfwz)
            halfez += (m_x * halfwy - m_y * halfwx)

        return halfex, halfey, halfez

    def accel_correct(self, accel: accelerometer, halfex: float, halfey: float, halfez: float) -> tuple:
        '''using gravity direction to correct gyro'''
        if accel.is_valid():
            accel.reverse()
            a_x, a_y, a_z = accel.normalised()

            # Estimated direction of gravity and magnetic field
            halfvx = self.q1q3 - self.q0q2
            halfvy = self.q0q1 + self.q2q3
            halfvz = self.q0q0 - 0.5 + self.q3q3

            # Error is sum of cross product between estimated direction
            # and measured direction of field vectors
            halfex += a_y * halfvz - a_z * halfvy
            halfey += a_z * halfvx - a_x * halfvz
            halfez += a_x * halfvy - a_y * halfvx
        return halfex, halfey, halfez

    def normalise_quaternion(self):
        '''Normalise quaternion'''
        reciprocal_norm = 1.0/math.sqrt(self.q_0**2 + self.q_1**2 + self.q_2**2 + self.q_3**2)
        self.q_0 *= reciprocal_norm
        self.q_1 *= reciprocal_norm
        self.q_2 *= reciprocal_norm
        self.q_3 *= reciprocal_norm

        # Auxiliary variables to avoid repeated arithmetic
        self.q0q0 = self.q_0 * self.q_0
        self.q0q1 = self.q_0 * self.q_1
        self.q0q2 = self.q_0 * self.q_2
        self.q0q3 = self.q_0 * self.q_3
        self.q1q1 = self.q_1 * self.q_1
        self.q1q2 = self.q_1 * self.q_2
        self.q1q3 = self.q_1 * self.q_3
        self.q2q2 = self.q_2 * self.q_2
        self.q2q3 = self.q_2 * self.q_3
        self.q3q3 = self.q_3 * self.q_3

    def update_quaternion(self, g_x: float, g_y: float, g_z: float, d_t: float):
        '''Time update of quaternion.'''
        # ! q_k = q_{k-1} + dt*\dot{q}
        # ! \dot{q} = 0.5*q \otimes P(\omega)
        self.dq0 = 0.5*(self.q_1 * g_x + self.q_2 * g_y + self.q_3 * g_z)
        self.dq1 = 0.5*(self.q_0 * g_x + self.q_2 * g_z - self.q_3 * g_y)
        self.dq2 = 0.5*(self.q_0 * g_y - self.q_1 * g_z + self.q_3 * g_x)
        self.dq3 = 0.5*(self.q_0 * g_z + self.q_1 * g_y - self.q_2 * g_x)
        self.dq0 = -self.dq0

        self.q_0 += d_t*self.dq0
        self.q_1 += d_t*self.dq1
        self.q_2 += d_t*self.dq2
        self.q_3 += d_t*self.dq3

    def gyro_pid(self, gyros: list, halfex: float, halfey: float, halfez: float, d_t: float):
        '''pi feedback to gyro'''
        # pylint: disable=too-many-arguments
        if math.isclose(halfex, 0.0) or math.isclose(halfey, 0.0) or math.isclose(halfez, 0.0):
            print('skip pi')
        else:
            if self._two_ki > 0:
                self._gyro_bias_x += self._two_ki * halfex * d_t    # integral error scaled by Ki
                self._gyro_bias_y += self._two_ki * halfey * d_t
                self._gyro_bias_z += self._two_ki * halfez * d_t

                # apply integral feedback
                gyros[0] += self._gyro_bias_x
                gyros[1] += self._gyro_bias_y
                gyros[2] += self._gyro_bias_z
            else:
                self._gyro_bias_x = 0.0
                self._gyro_bias_y = 0.0
                self._gyro_bias_z = 0.0

            # Apply proportional feedback
            gyros[0] += self._two_kp * halfex
            gyros[1] += self._two_kp * halfey
            gyros[2] += self._two_kp * halfez

        return gyros[0], gyros[1], gyros[2]

    def so3_update(self, gyros: list, accel: accelerometer, maga: magnetometer, d_t: float):
        '''attitude update main cycle'''
        halfex = 0.0
        halfey = 0.0
        halfez = 0.0
        if not self._filter_inited:
            self._filter_inited = True
            self.so3_init(accel, maga)

        halfex, halfey, halfez = self.maga_correct(maga, halfex, halfey, halfez)

        halfex, halfey, halfez = self.accel_correct(accel, halfex, halfey, halfez)

        g_x, g_y, g_z = self.gyro_pid(gyros, halfex, halfey, halfez, d_t)

        self.update_quaternion(g_x, g_y, g_z, d_t)
        self.normalise_quaternion()

    def so3_init(self, accels: accelerometer, mags: magnetometer):
        '''caculate init attitude using accel and mag, then initlize quaternion'''
        init_pitch, init_roll = accels.acc_att()
        init_yaw = mags.mag_heading(init_pitch, init_roll)

        cos_rol = math.cos(init_roll * 0.5)
        sin_rol = math.sin(init_roll * 0.5)
        cos_pit = math.cos(init_pitch * 0.5)
        sin_pit = math.sin(init_pitch * 0.5)
        cos_yaw = math.cos(init_yaw * 0.5)
        sin_yaw = math.sin(init_yaw * 0.5)
        self.q_0 = cos_rol * cos_pit * cos_yaw + sin_rol * sin_pit * sin_yaw
        self.q_1 = sin_rol * cos_pit * cos_yaw - cos_rol * sin_pit * sin_yaw
        self.q_2 = cos_rol * sin_pit * cos_yaw + sin_rol * cos_pit * sin_yaw
        self.q_3 = cos_rol * cos_pit * sin_yaw - sin_rol * sin_pit * cos_yaw
        # auxillary variables to reduce number of repeated operations, for 1st pass
        self.q0q0 = self.q_0 * self.q_0
        self.q0q1 = self.q_0 * self.q_1
        self.q0q2 = self.q_0 * self.q_2
        self.q0q3 = self.q_0 * self.q_3
        self.q1q1 = self.q_1 * self.q_1
        self.q1q2 = self.q_1 * self.q_2
        self.q1q3 = self.q_1 * self.q_3
        self.q2q2 = self.q_2 * self.q_2
        self.q2q3 = self.q_2 * self.q_3
        self.q3q3 = self.q_3 * self.q_3

def main():
    '''test main'''
    att = SO3Attitude()
    att.test()

if __name__ == '__main__':
    main()
