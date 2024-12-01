from machine import Pin, I2C
import icm20948
from math import degrees, atan2
from utime import sleep_ms
from ustruct import unpack_from

I2C0_SCL = Pin(07) # I2C SCL 
I2C0_SDA = Pin(06) # I2C SCA
    
i2c0 = I2C(0, sda=I2C0_SDA, scl=I2C0_SCL, freq = 400_000)


#dmp = False for direct Acc/Gyr/Magn
#dmp = True for DMP processor

imu = icm20948.ICM20948(i2c0, dmp = False, debug=True)

if not imu.dmp_ready :
    
    imu.gyro_cal()

    while True:
        a = imu.acc()
        g = imu.gyro()
        m = imu.mag()   # Read directly to the slave
        imu.Q_update_full(m, a, g)
        heading = imu.heading
        pitch = imu.pitch
        roll = imu.roll

        print(f"""
Accel: {a[0]:05.2f} {a[1]:05.2f} {a[2]:05.2f}
Gyro:  {g[0]:05.2f} {g[1]:05.2f} {g[2]:05.2f}
Mag:   {m[0]:05.2f} {m[1]:05.2f} {m[2]:05.2f}
HPT:   {heading:7.3f} {pitch:7.3f} {roll:7.3f}""")
        
        sleep_ms(25)
    
else :
           
    #imu.DMP_enable_sensor("ACCELEROMETER",True)
    #imu.DMP_enable_sensor("GYROSCOPE",True)
    #imu.DMP_enable_sensor("RAW_ACCELEROMETER",True)
    #imu.DMP_enable_sensor("RAW_GYROSCOPE",True)
    imu.DMP_enable_sensor("MAGNETIC_FIELD_UNCALIBRATED",True)
    #imu.DMP_enable_sensor("GYROSCOPE_UNCALIBRATED",True)
    #imu.DMP_enable_sensor("ACTIVITY_CLASSIFICATON",True)
    #imu.DMP_enable_sensor("STEP_DETECTOR",True)
    #imu.DMP_enable_sensor("STEP_COUNTER",True)
    #imu.DMP_enable_sensor("GAME_ROTATION_VECTOR",True)
    #imu.DMP_enable_sensor("ROTATION_VECTOR",True)
    #imu.DMP_enable_sensor("GEOM_ROTATION_VECTOR",True)
    #imu.DMP_enable_sensor("GEOM_FIELD",True)
    #imu.DMP_enable_sensor("GRAVITY",True)
    #imu.DMP_enable_sensor("LINEAR_ACCELERATION",True)
    #imu.DMP_enable_sensor("ORIENTATION",True)
    
    while True:
        imu.DMP_fifo_read()
