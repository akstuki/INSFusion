# INSFusion
python project to caculate the AHRS, method: pure integration,accelmeter, EKF,complementary, SO3

## env
window, linux

## language
python3 (3.5 or newer)

## usage
```bash
main.py -s <sensorfile> -a <attfile> -m <method>  
```
　-s and -a are cmd args to give the datafile name ,the data format is [px4 ulog](https://github.com/PX4/pyulog)  
　　-s give the sensor_compined file  
　　-a give the vechile attitude file  
　-m to git the attitude cculator method   
　　i:integration  
　　a:accelameter  
　　c:complementary  
　　s:SO3
　　e:ekf  

## test
the test data are in test folder, below is one test result
```bash
python main.py -m c
```
![Image text](https://github.com/akstuki/INSFusion/blob/master/img/so3.png)
