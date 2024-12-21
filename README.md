# ICM 20948 Micropython Library

<p align="left">This library support 9-DOF TDK Inversense ICM20948 chip in full Micropython</p>

- Handles Accelerometer, gyroscope and compass in "direct mode"

- Handles DMP (Digital Motion Processor) to output sensors but also fusion quaternions.

## Greetings first

A spécial thanks to Paul CLARK from Sparkfun, his C/C++ Arduino library was a real inspiration to gain the full micropython library working.
You ought to give a look at [GitHub - sparkfun/SparkFun_ICM-20948_ArduinoLibrary: Arduino support for ICM_20948 w/ portable C backbone](https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary))

The library is a single bloc of code, handling all needed parameters that were spread away in several C or C++ library, and is fully documented
in order that everyone interested might complete my piece of work. As reverse engineering was a bit time consumming, I wanted this readme to be
the most completed I could.



## About the chip itself...

<img title="" src="/ressources/icm20948_chip.png" alt="" width="239" data-align="center">

The chip is in fact composed of 3 chips 

- The Main ICM chip with accelerometer and compass

- The AK09916 magnetometer

- The ATMEL SAM G55 Arm® Cortex®-M4 (ATSAMG55J19) DMP chip ()



The ICM chip bloc diagram is 

<img title="" src="/ressources/icm20948_bloc.png" alt="" width="" data-align="center">>

So we understand quickly

- That Compass access is not straight forward but can be accessed by I2C Master interface (I2C master read slave register and outputs values in the appropriate register of the ICM chip)

- DMP processor is working standalone, can access accelerometer and gyroscope sensors but also compass (thanks to I2C master) and can output FIFO datas.

## Orientation Considerations

We must be aware that the two sensors chips (ICM and AK09916), while mounted in the same package, do not share the same orientation.

| <img title="" src="/ressources/orientation_magnetometer_gyroscope.png"> | <img title="" src="/ressources/orientation_compass.png"> |
| ---------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------- |

Considering Accelerometer orthonomous reference (left) the compass orthonomous reference (right) is inverted for Y and Z axis. we can write :

MAG_x_ICM  = MAG_x_AK9916
MAG_y_ICM = - MAG_y_AK9916
MAG_z_ICM = - MAG_z_AK9916

or in a matrix way

MAG_ICM =  [[1   0    0][0  -1   0][0   0  -1]]  . MAG_AK9916
         
## DMP way of working

DMP Processor need to be told what kind of report is expected.
The library itself convert the report we ask into a DMP understandable report name , that depends on the "DMP firmware". 

List of DMP reports authorized with DMP firmware are listed below. The controls bits is the internal code we need to write to DMP memory (DMP_DO_Ctrl_1) in order to activate the corresponding report. This code is in fact, the header of the expected report message

<img title="" src="/ressources/sensors_ctrl_bits.png">

As an example, when requiring Gyroscope sensor, we need to write 0x4048.
This value corresponds to 0x4000 + 0x0040 + 0x0008 = Gyroscope + Gyroscope Calibration + Secondary Message (we'll explain later) as referenced in the chart below.

<img title="" src="file:///C:/Users/Ludovic/Desktop/Micropython/ICM20948/ressources/reports_1_ctrl_bits.png" alt="" width="439">

We will also need to detail what is the content we expect in the secondary message, by writing the appropriate value to DMP memory (DMP_DO_Ctrl_2).  In our example, we'd better write 0x2000 in order to receive the compass accuracy message.

<img title="" src="/ressources/reports_2_ctrl_bits.png" alt="" width="439">

All this selection stuff if automaticaly done by the DMP_enable_sensor function and rely on importants parameter as 

INV_NEEDS_ACCEL_MASK0 =  0b11100010100111101000111000001010

This bit line is in fact an image of the following chart

<img title="" src="/ressources/mask_needed.png" alt="" width="439">

Explanation won't be complete if I do not explain that we also need to write 2 other registers 

We must also set the DMP Memory (DMP_DATA_RDY_STATUS) depending on the sensor we need

<img title="" src="/ressources/data_ready.png" alt="" width="439">

and also set the DMP memory (DMP_DATA_MOTION_EVENT_CTRL) the same way

<img title="" src="/ressources/motion_event.png" alt="" width="439">

Once we have done all this, the DMP processsor is ready to output message to the FIFO. I won't explain the way it works, quite simple, but once a message is received, we must parse the FIFO buffer, starting to read the header (2 bytes) and eventually the second header (2 bytes). They will both inform us of the data type that is present into the buffer (and has always the same structure) and we just need to process byte by byte.



# Limitation of the library

The library handles does not handle all the sensors available in the DMP. Only a few of them can be processed, the list is given below with the correspondance between the sensors we handle and the android sensors of DMP.

<img title="" src="/ressources/icm_to_android.png" alt="" width="780">



# Library setup

We first need to setup and I2C bus and declare the icm20948 library :

```
I2C0_SCL = Pin(07)
I2C0_SDA = Pin(06)
i2c0 = I2C(0, sda=I2C0_SDA, scl=I2C0_SCL, freq = 400_000)
imu = icm20948.ICM20948(i2c0, dmp = True, debug=0b01001)
```

Library has several options :



ICM20948 (i2c , addr, dmp, debug):

where :

i2c is mandatory, and is the I2C bus name,

addr is optionnal, if not given, the library will check the I2C bus for appropriate address,

dmp is False by default, an can be set to true if DMP is required

debug is optionnal but might help further developments 

> bit 0 : 01 :  ICM Chip Debug (Outputs library main messages)
> 
> bit 1 : 02 : DMP Chip debug (Outputs DMP memory access)
> 
> bit 2 : 04 : DMP FIFO processing debug (Outputs values during FIFO processing)
> 
> bit 3 : 08 : DMP Output debug (Outputs values after FIFO process)
> 
> bit 4 : 16 : DMP Register / I2C debug (Outputs bus communication)

### 

# Library usage

## Non DMP way

```
imu.gyro_cal()
imu.acc_cal()
imu.mag_cal()

while True:
    a = imu.acc()
    g = imu.gyro()
    m = imu.mag()
    imu.Q_update_full(m, a, g)
    heading = imu.heading
    pitch = imu.pitch
    roll = imu.roll
```

## DMP way

```
    
    imu.DMP_enable_sensor("GYROSCOPE",True)
    imu.DMP_enable_sensor("RAW_ACCELEROMETER",True)
        
    while True:
        imu.DMP_fifo_read()
```
