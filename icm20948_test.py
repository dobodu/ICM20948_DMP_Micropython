from machine import Pin, I2C
import icm20948
from math import degrees, atan2
from utime import sleep_ms

I2C0_SCL = Pin(07)          # I2C SCL 
I2C0_SDA = Pin(06)          #  I2C SCA
    
i2c0 = I2C(0, sda=I2C0_SDA, scl=I2C0_SCL, freq = 400_000)
imu = icm20948.ICM20948(i2c0,debug=True)

imu.set_acc_sample_rate()
imu.set_acc_low_pass()
imu.set_acc_full_scale()
imu.set_gyro_sample_rate()
imu.set_gyro_low_pass()
imu.set_gyro_full_scale()

DMP = True

if not DMP :
    
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
    
    imu.DMP_config()
        
    imu.DMP_activate_sensor("ACCELEROMETER",True)
    imu.DMP_activate_sensor("GAME_ROTATION_VECTOR",True)
    
    while True:
        i = imu.DMP_fifo_count()
        if i > 0 :
            imu.DMP_fifo_read(i)
        sleep_ms(10)