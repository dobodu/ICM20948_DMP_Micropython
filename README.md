<h1 align="left">ICM 20948 Micropython Library (Direct and DMP supporrt)</h1>

###

<p align="left">This library support 9-DOF ICM20948</p>

###

<h2 align="left">Usage</h2>

###

<p align="left">I2C0_SCL = Pin(07)  # I2C SCL Pin<br>I2C0_SDA = Pin(06) # I2C SCA Pin<br>i2c0 = I2C(0, sda=I2C0_SDA, scl=I2C0_SCL, freq = 400_000) # I2C Bus<br><br>imu = icm20948.ICM20948(i2c0, dmp = True, debug=0b01001)</p>

###

<h2 align="left">Library usage</h2>

###

<p align="left">dmp = True or False depending if you want to use internal Digital Motion Processor<br><br>debug = 5 bit debugging options:<br><br>#         bit 0 : 01 : ICM Chip Level<br>#         bit 1 : 02 : DMP Chip Acces<br>#         bit 2 : 04 : DMP Fifo processing<br>#         bit 3 : 08 : DMP Output<br>#         bit 4 : 16 : ICM Register Access</p>

###

<h2 align="left">Sensor Activation</h2>

###

<p align="left">#imu.DMP_enable_sensor("ACCELEROMETER",True)<br>    #imu.DMP_enable_sensor("GYROSCOPE",True)<br>    #imu.DMP_enable_sensor("RAW_ACCELEROMETER",True)<br>    #imu.DMP_enable_sensor("RAW_GYROSCOPE",True)<br>    imu.DMP_enable_sensor("MAGNETIC_FIELD_UNCALIBRATED",True)<br>    #imu.DMP_enable_sensor("GYROSCOPE_UNCALIBRATED",True)<br>    #imu.DMP_enable_sensor("ACTIVITY_CLASSIFICATON",True)<br>    #imu.DMP_enable_sensor("STEP_DETECTOR",True)<br>    #imu.DMP_enable_sensor("STEP_COUNTER",True)<br>    #imu.DMP_enable_sensor("GAME_ROTATION_VECTOR",True)<br>    #imu.DMP_enable_sensor("ROTATION_VECTOR",True)<br>    #imu.DMP_enable_sensor("GEOM_ROTATION_VECTOR",True)<br>    #imu.DMP_enable_sensor("GEOM_FIELD",True)<br>    #imu.DMP_enable_sensor("GRAVITY",True)<br>    #imu.DMP_enable_sensor("LINEAR_ACCELERATION",True)<br>    #imu.DMP_enable_sensor("ORIENTATION",True)</p>

###
