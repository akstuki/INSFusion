'''
* main.py : Attitude caculator using SO3
*
*          Copyright (C) 2018 by XiaoqiangChen, All rights reserved.
* author  : XiaoqiangChen
* mail    : 309905109@qq.com
* history : 2018/09/26  1.0  new
'''

from CAttitude import Attitude
from lib.Quaternion import DCM2Euler
from lib.Quaternion import Quat2DCM
import math

class SO3Attitude(Attitude):
    """docstring for SO3Attitude"""
    _initialized = False;

    def __init__(self):
        super(SO3Attitude, self).__init__()
        self._strategy = "SO3";
        self._gyro_offset_x = 0;
        self._gyro_offset_y = 0;
        self._gyro_offset_z = 0;
        self._gyro_offset_count = 0;
        self.gyro_bias_x = 0;
        self.gyro_bias_y = 0;
        self.gyro_bias_z = 0;
        self.q0 = 1.0;
        self.q1 = 0.0;
        self.q2 = 0.0;
        self.q3 = 0.0;
        self.dq0 = 0.0;
        self.dq1 = 0.0;
        self.dq2 = 0.0;
        self.dq3 = 0.0;
        self.q0q0 = 0.0;
        self.q0q1 = 0.0;
        self.q0q2 = 0.0;
        self.q0q3 = 0.0;
        self.q1q1 = 0.0;
        self.q1q2 = 0.0;
        self.q1q3 = 0.0;
        self.q2q2 = 0.0;
        self.q2q3 = 0.0;
        self.q3q3 = 0.0;
        self.bFilterInit = False;
        self.twoKi = 0.05;
        self.twoKp = 1.0;

    def calculateAtt(self):
        lsGyros = zip(self._dataSet._lsDeltT,self._dataSet._lsGyroX,self._dataSet._lsGyroY,self._dataSet._lsGyroZ);
        lsAccs = zip(self._dataSet._lsAcclX,self._dataSet._lsAcclY,self._dataSet._lsAcclZ);
        lsmag = zip(self._dataSet._lsMageX,self._dataSet._lsMageY,self._dataSet._lsMageZ);
        for imu in zip(lsGyros,lsAccs,lsmag):
            if self._initialized == False:
                self._gyro_offset_x += imu[0][1];
                self._gyro_offset_y += imu[0][2];
                self._gyro_offset_z += imu[0][3];
                self._gyro_offset_count += 1;
                if self._gyro_offset_count == 1000:
                    self._initialized = True;
                    self._gyro_offset_x /= self._gyro_offset_count;
                    self._gyro_offset_y /= self._gyro_offset_count;
                    self._gyro_offset_z /= self._gyro_offset_count;
                self._lsPitch.append(None);
                self._lsRoll.append(None);
                self._lsYaw.append(None);
            else:
                gyro_x = imu[0][1] - self._gyro_offset_x;
                gyro_y = imu[0][2] - self._gyro_offset_y;
                gyro_z = imu[0][3] - self._gyro_offset_z;
                acc_x =  imu[1][0];
                acc_y =  imu[1][1];
                acc_z =  imu[1][2];
                mag_x =  imu[2][0];
                mag_y =  imu[2][1];
                mag_z =  imu[2][2];
                dt = imu[0][0];
                self.NonlinearSO3AHRSupdate(gyro_x,gyro_y,gyro_z,-acc_x,-acc_y,-acc_z,mag_x,mag_y,mag_z,dt);

                # Convert q->R, This R converts inertial frame to body frame.
                Rot_matrix = Quat2DCM(self.q0, self.q1,self.q2,self.q3);
                pitch,roll,yaw = DCM2Euler(Rot_matrix);

                self._lsPitch.append(pitch);
                self._lsRoll.append(roll);
                self._lsYaw.append(yaw);

    def NonlinearSO3AHRSupdate(self,gx:float,gy:float,gz:float,ax:float,ay:float,az:float, \
            mx:float,my:float,mz:float,dt:float):
        halfex = 0.0;
        halfey = 0.0;
        halfez = 0.0;
        if self.bFilterInit == False:
            self.bFilterInit = True;
            self.NonlinearSO3AHRSinit(-ax,-ay,-az,mx,my,mz);
        if math.isclose(mx,0.0) and math.isclose(my,0.0) and math.isclose(mz,0.0):
            print('mag=0',mx,my,mz) 
        else:
            recipNorm = 1.0/math.sqrt(mx**2 + my**2 + mz**2);
            mx *= recipNorm;
            my *= recipNorm;
            mz *= recipNorm;
        
            # Reference direction of Earth's magnetic field
            hx = 2.0*(mx * (0.5-self.q2q2-self.q3q3) + my * (self.q1q2-self.q0q3) + mz * (self.q1q3+self.q0q2));
            hy = 2.0 * (mx * (self.q1q2+self.q0q3) + my * (0.5-self.q1q1-self.q3q3) + mz * (self.q2q3-self.q0q1));
            hz = 2.0 * mx * (self.q1q3-self.q0q2) + 2.0 * my * (self.q2q3+self.q0q1) + 2.0 * mz * (0.5-self.q1q1-self.q2q2);
            bx = math.sqrt(hx * hx + hy * hy);
            bz = hz;
        
            # Estimated direction of magnetic field
            halfwx = bx * (0.5 - self.q2q2 - self.q3q3) + bz * (self.q1q3 - self.q0q2);
            halfwy = bx * (self.q1q2 - self.q0q3) + bz * (self.q0q1 + self.q2q3);
            halfwz = bx * (self.q0q2 + self.q1q3) + bz * (0.5 - self.q1q1 - self.q2q2);
        
            # Error is sum of cross product between estimated direction and measured direction of field vectors
            halfex += (my * halfwz - mz * halfwy);
            halfey += (mz * halfwx - mx * halfwz);
            halfez += (mx * halfwy - my * halfwx);

        if math.isclose(ax,0.0) and math.isclose(ay,0.0) and math.isclose(az,0.0):
            print('acc=0',ax,ay,az) 
        else:
            recipNorm = 1.0/math.sqrt(ax**2 + ay**2 + az**2);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            # Estimated direction of gravity and magnetic field
            halfvx = self.q1q3 - self.q0q2;
            halfvy = self.q0q1 + self.q2q3;
            halfvz = self.q0q0 - 0.5 + self.q3q3;
        
            # Error is sum of cross product between estimated direction and measured direction of field vectors
            halfex += ay * halfvz - az * halfvy;
            halfey += az * halfvx - ax * halfvz;
            halfez += ax * halfvy - ay * halfvx;

        if math.isclose(halfex,0.0) or math.isclose(halfey,0.0) or math.isclose(halfez,0.0):
            print('skip pi') 
        else:
            if self.twoKi>0:
                self.gyro_bias_x += self.twoKi * halfex * dt;    # integral error scaled by Ki
                self.gyro_bias_y += self.twoKi * halfey * dt;
                self.gyro_bias_z += self.twoKi * halfez * dt;
                
                # apply integral feedback
                gx += self.gyro_bias_x;
                gy += self.gyro_bias_y;
                gz += self.gyro_bias_z;
            else:
                gyro_bias_x=gyro_bias_y=gyro_bias_z=0.0;

            # Apply proportional feedback
            gx += self.twoKp * halfex;
            gy += self.twoKp * halfey;
            gz += self.twoKp * halfez;

        # Time derivative of quaternion. q_dot = 0.5*q\otimes omega.
        # ! q_k = q_{k-1} + dt*\dot{q}
        # ! \dot{q} = 0.5*q \otimes P(\omega)
        self.dq0 = 0.5*(-self.q1 * gx - self.q2 * gy - self.q3 * gz);
        self.dq1 = 0.5*( self.q0 * gx + self.q2 * gz - self.q3 * gy);
        self.dq2 = 0.5*( self.q0 * gy - self.q1 * gz + self.q3 * gx);
        self.dq3 = 0.5*( self.q0 * gz + self.q1 * gy - self.q2 * gx); 
    
        self.q0 += dt*self.dq0;
        self.q1 += dt*self.dq1;
        self.q2 += dt*self.dq2;
        self.q3 += dt*self.dq3;
        
        # Normalise quaternion
        recipNorm = 1.0/math.sqrt(self.q0**2 + self.q1**2 + self.q2**2 + self.q3**2);
        self.q0 *= recipNorm;
        self.q1 *= recipNorm;
        self.q2 *= recipNorm;
        self.q3 *= recipNorm;

        # Auxiliary variables to avoid repeated arithmetic
        self.q0q0 = self.q0 * self.q0;
        self.q0q1 = self.q0 * self.q1;
        self.q0q2 = self.q0 * self.q2;
        self.q0q3 = self.q0 * self.q3;
        self.q1q1 = self.q1 * self.q1;
        self.q1q2 = self.q1 * self.q2;
        self.q1q3 = self.q1 * self.q3;
        self.q2q2 = self.q2 * self.q2;
        self.q2q3 = self.q2 * self.q3;
        self.q3q3 = self.q3 * self.q3;  


            
    def NonlinearSO3AHRSinit(self,ax:float,ay:float,az:float, mx:float,my:float,mz:float):
        initialPitch,initialRoll = self.accAtt([ax,ay,az]);
        initialHdg = self.magHeading([mx,my,mz],initialPitch,initialRoll);

        cosRoll = math.cos(initialRoll * 0.5);
        sinRoll = math.sin(initialRoll * 0.5);
    
        cosPitch = math.cos(initialPitch * 0.5);
        sinPitch = math.sin(initialPitch * 0.5);
    
        cosHeading = math.cos(initialHdg * 0.5);
        sinHeading = math.sin(initialHdg * 0.5);
    
        self.q0 = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
        self.q1 = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
        self.q2 = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
        self.q3 = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;
    
        # auxillary variables to reduce number of repeated operations, for 1st pass
        self.q0q0 = self.q0 * self.q0;
        self.q0q1 = self.q0 * self.q1;
        self.q0q2 = self.q0 * self.q2;
        self.q0q3 = self.q0 * self.q3;
        self.q1q1 = self.q1 * self.q1;
        self.q1q2 = self.q1 * self.q2;
        self.q1q3 = self.q1 * self.q3;
        self.q2q2 = self.q2 * self.q2;
        self.q2q3 = self.q2 * self.q3;
        self.q3q3 = self.q3 * self.q3;

def main():
    sensorfile = r'test\09_26_14_sensor_combined_0.csv';
    attfile = r'test\09_26_14_vehicle_attitude_0.csv'
    att = SO3Attitude();
    att.loadData(sensorfile,attfile);
    att.calculateAtt();
    att.showFig();

if __name__ == '__main__':
    main()