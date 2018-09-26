# INSFusion
python project to caculate the AHRS, method: pure integration,accelmeter, EKF,complementary, SO3

## env
window, linux

## language
python3

## usage
main.py -s <sensorfile> -a <attfile> -m <method>  
	-s -a are cmd args to give the datafile name  
	    the data format is px4 ulog  
	    -s give the sensor_compined file  
	    -a give the vechile attitude file  
	-m to git the attitude cculator method   
	    i:integration  
        a:accelameter  
        c:complementary  
        e:ekf  

## test
the test data are in test folder, below is one test result
![Image text](https://github.com/akstuki/INSFusion/blob/master/img/com.png)