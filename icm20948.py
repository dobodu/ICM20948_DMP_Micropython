from ustruct import unpack_from
from math import asin, atan2, degrees, radians, sqrt
from utime import sleep_ms, ticks_ms, ticks_us, ticks_diff, localtime

LIBNAME = "ICM20948"
LIBVERSION = "0.99-9.1"

# This micropython library drive the TDK ICM20948 9 axis sensors
# It can work :
#    As a basic sensor (icm20948.acc() will return accelerator values etc...
#    As a DMP sensor (FIFO values will be processed)
# It includes :
#    A quaternion gestion for the basic sensor (see Q_update_full or Q_update_nomag)
#
# Many thanks to : Sparfun helpfull library
#                  (see https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary) 
#
#                  Master Thesis of John CAPPELLE
#                  (see https://jonacappelle.github.io/Master-Thesis)
#
# All reading are converted with this orientation
#
#         ^                        Magnetometer
#         | Y+    . Z+                          X Z+ 
#     ----------                     ---------
#    |°1        |                   |°1       |
#    | ICM20948 |  -> X+            | AK09916 |  -> X+
#    |          |                   |         |
#     ----------                     ---------
#    Accelerometer                      |
#    Gyrometer                          v Y+
#
# Thus  X_mag_ICM20948 =  X_mag_AK9916
#       Y_mag_ICM20948 = -Y_mag_AK9916
#       Z_mag_ICM20948 = -Z_mag_AK9916
# 
# Transfert Matrix is
#
#        [[ 1  0  0]
#         [ 0 -1  0]
#         [ 0  0 -1]]
#
# Working / Not working DMP Sensors
#
#  Sensor                        Working
#  -----------------------------------------------------
#  ACCELEROMETER :               OK
#  GYROSCOPE :                   OK
#  RAW_ACCELEROMETER             OK
#  RAW_GYROSCOPE                 OK
#  MAGNETIC_FIELD_UNCALIBRATED : OK
#  GYROSCOPE_UNCALIBRATED        OK          
#  ACTIVITY_CLASSIFICATON        NO          Missing ANDROID_SENSORS_CTRL_BITS
#  STEP_DETECTOR                 NO          Missing header2 ?
#  STEP_COUNTER                  NO          Missing header2 ?
#  GAME_ROTATION_VECTOR          OK
#  ROTATION_VECTOR               OK
#  GEOM_ROTATION_VECTOR :        OK
#  GEOM_FIELD :                  OK
#  GRAVITY :                     OK
#  LINEAR_ACCELERATION           OK
#  ORIENTATION                   OK
#
# To do :
#
# Implement DMP bias writting for accelerometer and gyro
# Check self.DMP_set_gyro_sf
# Check quaternions
# Check DMP Gyro Bias vales
# Write DMP interrupt enable function
# Understand and rewrite Configure Acceleration Only Gains, Alpha Var and AVAr

#================================================================================
# CHIP ADRESSES AND IDS
#================================================================================

ICM_ADDRESS = (0x68, 0x69)
ICM_CHIP_ID = 0xEA
AK_I2C_ADDR = 0x0C
AK_CHIP_ID = 0x09

#================================================================================
# ICM20948 REGISTERS
#================================================================================

#BANK COMMON
ICM_BANK_SEL = 0x7F

#BANK0
ICM_WHO_AM_I = 0x00
ICM_USER_CTRL = 0x03
ICM_USER_CTRL_DMP_EN = 0b10000000
ICM_USER_CTRL_FIFO_EN = 0b01000000
ICM_USER_CTRL_I2C_MST_EN = 0b00100000
ICM_USER_CTRL_I2C_IF_DIS = 0b00010000
ICM_USER_CTRL_DMP_RST = 0b00001000
ICM_USER_CTRL_SRAM_RST = 0b00000100
ICM_USER_CTRL_I2C_MST_RST = 0b00000010
ICM_LP_CFG = 0x05
ICM_LP_CFG_MST =  0b01000000
ICM_LP_CFG_ACC =  0b00100000
ICM_LP_CFG_GYRO = 0b00010000
ICM_PWR_MGMT_1 = 0x06
ICM_PWR_MGMT_1_RESET = 0x80
ICM_PWR_MGMT_1_SLEEP = 0x40
ICM_PWR_MGMT_1_LP = 0x20
ICM_PWR_MGMT_1_NO_TEMP = 0x08
ICM_PWR_MGMT_1_CLOCK_RESET = 0x07
ICM_PWR_MGMT_1_CLOCK_AUTO = 0x01
ICM_PWR_MGMT_1_CLOCK_INTERNAL = 0x00
ICM_PWR_MGMT_2 = 0x07
ICM_INT_PIN_CFG = 0x0F
ICM_INT_PIN_CFG_ACTL = 0x80
ICM_INT_PIN_CFG_OPEN = 0x40
ICM_INT_PIN_CFG_LATCH__EN = 0x20
ICM_INT_PIN_CFG_ANYRD_2CLEAR = 0x10
ICM_INT_PIN_CFG_ACTL_FSYNC = 0x08
ICM_INT_PIN_CFG_FSYNC_MODE_EN = 0x04
ICM_INT_PIN_CFG_BYPASS_EN = 0x02
ICM_INT_ENABLE = 0x10
ICM_INT_ENABLE_1 = 0x11
ICM_INT_ENABLE_2 = 0x12
ICM_INT_ENABLE_3 = 0x13
ICM_I2C_MST_STATUS = 0x17
ICM_DMP_INT_STATUS = 0x18 #thanks Sparkfun
ICM_INT_STATUS =0x19
ICM_INT_STATUS_1 =0x1A
ICM_INT_STATUS_2 =0x1B
ICM_INT_STATUS_2 =0x1C
ICM_SINGLE_FIFO_PRIORITY_SEL = 0x26 #thanks Sparkfun
ICM_DELAY_TIME_H = 0x28
ICM_ACCEL_XOUT_H = 0x2D
ICM_GYRO_XOUT_H = 0x33
ICM_TEMP_OUT_H = 0x39
ICM_EXT_SLV_SENS_DATA_00 = 0x3B # Linked to AK_ST1 (0x10) or AK_RSV2 (0x03)
ICM_EXT_SLV_SENS_DATA_01 = 0x3C # Linked to AK_HXL (0x11) or ??? (0x04)
ICM_EXT_SLV_SENS_DATA_02 = 0x3D # Linked to AK_HXH (0x12) or AK_HXH (0x05)
ICM_EXT_SLV_SENS_DATA_03 = 0x3E # Linked to AK_HYL (0x13) or AK_HXL (0x06)
ICM_EXT_SLV_SENS_DATA_04 = 0x3F # Linked to AK_HYH (0x14) or AK_HYH (0x07)
ICM_EXT_SLV_SENS_DATA_08 = 0x43 # Linked to AK_ST2
#Add other there if needed
ICM_FIFO_EN_1 = 0x66
ICM_FIFO_EN_2 = 0x67
ICM_FIFO_RST = 0x68
ICM_FIFO_MODE = 0x69
ICM_FIFO_COUNTH = 0x70
ICM_FIFO_COUNTL = 0x71
ICM_FIFO_R_W = 0x72
ICM_DATA_RDY_STATUS = 0x74
ICM_HW_FIX_DISABLE = 0x75  #thanks Sparkfun
ICM_FIFO_CFG = 0x76
ICM_MEM_START_ADDR = 0x7C
ICM_MEM_R_W = 0x7D
ICM_MEM_BANK_SEL = 0x7E

#BANK1
ICM_SELF_TEST_X_GYRO = 0x02
ICM_SELF_TEST_Y_GYRO = 0x03
ICM_SELF_TEST_Z_GYRO = 0x04
ICM_SELF_TEST_X_ACCEL = 0x0E
ICM_SELF_TEST_Y_ACCEL = 0x0F
ICM_SELF_TEST_Z_ACCEL = 0x10
ICM_XA_OFFS_H = 0x14
ICM_YA_OFFS_H = 0x17
ICM_ZA_OFFS_H = 0x1A
ICM_TIMEBASE_CORRECTION_PLL = 0x28

#BANK2
ICM_GYRO_SMPLRT_DIV = 0x00
ICM_GYRO_CONFIG_1 = 0x01
ICM_GYRO_CONFIG_1_GYRO_FS_SEL_MASK = 0b11111001
ICM_GYRO_CONFIG_1_GYRO_DLPCFCFG_MASK = 0b10001110
ICM_GYRO_CONFIG_2 = 0x02
ICM_ODR_ALIGN_EN = 0x09
ICM_ACCEL_SMPLRT_DIV_1 = 0x10
ICM_ACCEL_SMPLRT_DIV_2 = 0x11
ICM_ACCEL_INTEL_CTRL = 0x12
ICM_ACCEL_WOM_THR = 0x13
ICM_ACCEL_CONFIG_1 = 0x14
ICM_ACCEL_CONFIG_1_ACCEL_FS_SEL_MASK = 0b11111001
ICM_ACCEL_CONFIG_1_ACCEL_DLPFCFG_MASK = 0b10001110
ICM_ACCEL_CONFIG_2 = 0x15
ICM_PRS_ODR_CONFIG = 0x20
ICM_PRGM_START_ADDRH = 0x50
ICM_PRGM_START_ADDRL = 0x51
ICM_FSYNC_CONFIG = 0x52
ICM_TEMP_CONFIG = 0x53
ICM_FSYNC_CONFIG = 0x52
ICM_TEMP_CONFIG = 0x53
ICM_MOD_CTRL_USR = 0x54
ICM_MOD_CTRL_USR = 0x54
ICM_MOD_CTRL_USR_REG_LP_DMP_EN = 0x01

#BANK3
ICM_I2C_MST_ODR_CONFIG = 0x00
ICM_I2C_MST_CTRL = 0x01
ICM_I2C_MST_CTRL_MULTI = 0b10000000 #Multi master
ICM_I2C_MST_CTRL_NSR =   0b00010000 #Stop between reads
ICM_I2C_MST_DELAY_CTRL = 0x02
ICM_I2C_SLV0_ADDR = 0x03
ICM_I2C_SLV0_REG = 0x04
ICM_I2C_SLV0_CTRL = 0x05
ICM_I2C_SLV0_DO = 0x06
ICM_I2C_SLV1_ADDR = 0x07
ICM_I2C_SLV1_REG = 0x08
ICM_I2C_SLV1_CTRL = 0x09
ICM_I2C_SLV1_DO = 0x0A
ICM_I2C_SLV2_ADDR = 0x0B
ICM_I2C_SLV2_REG = 0x0C
ICM_I2C_SLV2_CTRL = 0x0D
ICM_I2C_SLV2_DO = 0x0E
ICM_I2C_SLV3_ADDR = 0x0F
ICM_I2C_SLV3_REG = 0x10
ICM_I2C_SLV3_CTRL = 0x11
ICM_I2C_SLV3_DO = 0x12
ICM_I2C_SLV4_ADDR = 0x13
ICM_I2C_SLV4_REG = 0x14
ICM_I2C_SLV4_CTRL = 0x15
ICM_I2C_SLV4_DO = 0x16
ICM_I2C_SLV4_DI = 0x16
#BANK 3 COMMON
ICM_I2C_SLV_ADDR_RNW = 0x80
ICM_I2C_SLV_CTRL_SLV_ENABLE = 0x80
ICM_I2C_SLV_CTRL_BYTE_SWAP = 0x40
ICM_I2C_SLV_CTRL_REG_DIS = 0x20
ICM_I2C_SLV_CTRL_REG_GROUP = 0x10

#================================================================================
# AK09916 REGISTERS
#================================================================================

AK_WIA1 = 0x00
AK_WIA2 = 0x01
AK_RSV1 = 0x02
AK_RSV2 = 0x03 #Reserved register, used for DMP reading
AK_HXH_RSV = 0x04 # ? Hidden Magnetometer Data in Big Indian Format
AK_ST1 = 0x10
AK_ST1_DOR = 0b00000010   # Data overflow bit
AK_ST1_DRDY = 0b00000001  # Data self.ready bit
AK_HXL = 0x11
AK_ST2 = 0x18
AK_ST2_HOFL = 0b00001000   # Magnetic sensor overflow bit
AK_CNTL2 = 0x31
AK_CNTL2_MODE_OFF = 0
AK_CNTL2_MODE_SINGLE = 1
AK_CNTL2_MODE_10HZ  = 2
AK_CNTL2_MODE_20HZ  = 4
AK_CNTL2_MODE_50HZ  = 6
AK_CNTL2_MODE_100HZ = 8
AK_CNTL2_MODE_TEST = 16
AK_CNTL3 = 0x32
AK_CNTL3_RESET = 0x01

#================================================================================
# CONSTANTS RELATED TO ICM20948 AND AK09916
#================================================================================

# OFFSETS AND SENSITIVITY
ICM_TEMPERATURE_SENSITIVITY = 333.87 # CHAPTER 3.4
ICM_ROOM_TEMP_OFFSET = 21			 # CHAPTER 3.4
ICM_TEMPERATURE_DEGREES_OFFSET = 21	 # CHAPTER 8.31
AK_MAGNETOMETER_SENSITIVITY = 0.15	 # CHAPTER 3.3

#RANGES AND SENSITIVITY
GYRO_SCALE_RANGE = {250: 0b00, 500: 0b01, 1000: 0b10, 2000: 0b11}   # CHAPTER 3.1
GYRO_SENSITIVITY_FACTOR = [131, 65.5, 32.8, 16.4]					# CHAPTER 3.1
ACC_SCALE_RANGE = {2: 0b00, 4: 0b01, 8: 0b10, 16: 0b11} 			# CHAPTER 3.2
ACC_SENSITIVITY_FACTOR = [16384.0, 8192.0, 4096.0, 2048.0]			# CHAPTER 3.2

#================================================================================
# DMP MEM REGISTER
#================================================================================

#DMP INFOS
DMP_MEM_BANK_SIZE = 256
#DMP MEMORY 
DMP_START_ADDRESS = 0x1000
DMP_LOAD_START = 0x90
DMP_MAX_WRITE = 16
#DATA OUTPUT CONTROL
DMP_DATA_OUT_CTL1 = 0x40 # 4*16+0 - 16 bits
DMP_DATA_OUT_CTL2 = 0x42 # 4*16+2 - 16 bits
DMP_DATA_INTR_CTL = 0x4C # 4*16+12 - 16 bits
DMP_DATA_FIFO_WATERMARK = 0x1FE # 31*16+14 - 16 bits
#MOTION EVENT CONTROL
DMP_DATA_MOTION_EVENT_CTRL = 0x4E # 4*16+14 - 16 bits
#INDICATE TO DMP WHICH SENSOR ARE AVAILABLE
DMP_DATA_RDY_STATUS = 0x8A # 8*16+10
#BATCH MODE
DMP_BM_BATCH_CNTR = 0x1B0 # 27*16+0
DMP_BM_BATCH_THLD = 0x13C # 19*16+12
DMP_BM_BATCH_MASK = 0x15E # 21*16+14
#SENSOR OUTPUT DATA RATE - ALL 16 BITS
DMP_ODR_GEOMAG = 0xA0 # 10*16+0
DMP_ODR_PQUAT6 = 0xA4 # 10*16+4
DMP_ODR_QUAT9 = 0xA8 # 10*16+8
DMP_ODR_QUAT6 = 0xAC # 10*16+12
DMP_ODR_ALS = 0xB2 # 11*16+2
DMP_ODR_CPASS_CALIBR = 0xB4 # 11*16+4
DMP_ODR_CPASS = 0xB6 # 11*16+6
DMP_ODR_GYRO_CALIB = 0xB8 # 11*16+8
DMP_ODR_GYRO = 0xBA # 11*16+10
DMP_ODR_PRESSURE = 0xBC # 11*16+12
DMP_ODR_ACCEL = 0xBE # 11*16+14
#SENSOR OUTPUT DATA RATE COUNTER - ALL 16 BITS
DMP_ODR_CNTR_GEOMAG = 0x80 # 8*16+0
DMP_ODR_CNTR_PQUAT6 = 0x84 # 8*16+4
DMP_ODR_CNTR_QUAT9 = 0x88 # 8*16+8
DMP_ODR_CNTR_QUAT6 = 0x8C # 8*16+12
DMP_ODR_CNTR_ALS = 0x92 # 9*16+2
DMP_ODR_CNTR_CPASS_CALIBR = 0x94 # 9*16+4
DMP_ODR_CNTR_CPASS = 0x96 # 9*16+6
DMP_ODR_CNTR_GYRO_CALIBR = 0x98 # 9*16+8
DMP_ODR_CNTR_GYRO = 0x9A # 9*16+10
DMP_ODR_CNTR_PRESSURE = 0x9C # 9*16+12
DMP_ODR_CNTR_ACCEL = 0x9E # 9*16+14
#MOUNTING MATRIX - ALL 32 BITS
DMP_CPASS_MTX_00 = 0x170 # 23*16+0
DMP_CPASS_MTX_01 = 0x174 # 23*16+4
DMP_CPASS_MTX_02 = 0x178 # 23*16+8
DMP_CPASS_MTX_10 = 0x17C # 23*16+12
DMP_CPASS_MTX_11 = 0x180 # 24*16+0
DMP_CPASS_MTX_12 = 0x184 # 24*16+4
DMP_CPASS_MTX_20 = 0x188 # 24*16+8
DMP_CPASS_MTX_21 = 0x18C # 24*16+12
DMP_CPASS_MTX_22 = 0x190 # 25*16+0
#BIAS CALIBRATION  - ALL 32 BITS
# Scaled by ACC 2^12 (FSR 4G) - GYRO 2^15 - COMPASS 2^16
DMP_ACCEL_BIAS_X = 0x6E4 # 110*16+4
DMP_ACCEL_BIAS_Y = 0x6E8 # 110*16+8
DMP_ACCEL_BIAS_Z = 0x6EC # 110*16+12
DMP_CPASS_BIAS_X = 0x7E4 # 126*16+4
DMP_CPASS_BIAS_Y = 0x7E8 # 126*16+8
DMP_CPASS_BIAS_Z = 0x7EC # 126*16+12
DMP_GYRO_ACCURACY = 0x8A2 # 138*16+2
DMP_GYRO_BIAS_X = 0x8B4 # 139*16+4
DMP_GYRO_BIAS_Y = 0x8B8 # 139*16+8
DMP_GYRO_BIAS_Z = 0x8BC # 139*16+12
DMP_GYRO_BIAS_SET = 0x8A6 # 138*16+6
DMP_GYRO_LAST_TEMPR = 0x860 # 134*16+0
DMP_GYRO_SLOPE_X = 0x4E4 # 78*16+4
DMP_GYRO_SLOPE_Y = 0x4E8 # 78*16+8
DMP_GYRO_SLOPE_Z = 0x4EC # 78*16+12
# ACCELEROMETER CALIBRATION
DMP_ACCEL_ACCURACY = 0x610 # 97*16+0
DMP_ACCEL_CAL_RESET = 0x4D0 # 77*16+0
DMP_ACCEL_VARIANCE_THRESH = 0x5D0 # 93*16+0
DMP_ACCEL_CAL_RATE = 0x5E4 # 94*16+4   16-bit: 0 (225Hz, 112Hz, 56Hz)
DMP_ACCEL_PRE_SENSOR_DATA = 0x614 # 97*16+4
DMP_ACCEL_COVARIANCE = 0x658 # 101*16+8
DMP_ACCEL_ALPHA_VAR = 0x5B0 # 91*16+0  32-bit: 1026019965 (225Hz) 977872018 (112Hz) 882002213 (56Hz)
DMP_ACCEL_A_VAR = 0x5C0 # 92*16+0 32-bit: 47721859 (225Hz) 95869806 (112Hz) 191739611 (56Hz)
DMP_ACCEL_CAL_INIT = 0x5E2 # 94*16+2
DMP_ACCEL_CAL_SCALE_COVQ_IN_RANGE = 0xC20 # 194*16+0
DMP_ACCEL_CAL_SCALE_COVQ_OUT_RANGE = 0xC30 # 195*16+0
DMP_ACCEL_CAL_TEMPERATURE_SENSITIVITY = 0xC24 # 194*16+4
DMP_ACCEL_CAL_TEMPERATURE_OFFSET_TRIM = 0xC2C # 194*16+12
DMP_CPASS_ACCURACY = 0x250 # 37*16+0
DMP_CPASS_BIAS_SET = 0x22E # 34*16+14
DMP_MAR_MODE = 0x252 # 37*16+2
DMP_CPASS_COVARIANCE = 0x730 # 115*16+0
DMP_CPASS_COVARIANCE_CUR = 0x768 # 118*16+8
DMP_CPASS_REF_MAG_3D = 0x7A0 # 122*16+0
DMP_CPASS_CAL_INIT = 0x720 # 114*16+0
DMP_CPASS_EST_FIRST_BIAS = 0x710 # 113*16+0
DMP_MAG_DISTURB_STATE = 0x712 # 113*16+2
DMP_CPASS_VAR_COUNT = 0x706 # 112*16+6
DMP_CPASS_COUNT_7 = 0x572 # 87*16+2
DMP_CPASS_MAX_INNO = 0x7C0 # 124*16+0
DMP_CPASS_BIAS_OFFSET = 0x714 # 113*16+4
DMP_CPASS_CUR_BIAS_OFFSET = 0x724 # 114*16+4
DMP_CPASS_PRE_SENSOR_DATA = 0x574 # 87*16+4
#COMPASS CALIBRATION PARAMS TO BE ADJUSTER WITH SAMPLING RATE
DMP_CPASS_TIME_BUFFER = 0x70E # 112*16+14
DMP_CPASS_RADIUS_3D_THRESH_ANOMALY = 0x708 # 112*16+8 : 4 bytes
DMP_CPASS_STATUS_CHK = 0x19C # 25*16+12
#GAINS
DMP_ACCEL_FB_GAIN = 0x220 # 34*16+0
DMP_ACCEL_ONLY_GAIN = 0x10C # 16*16+12 - 15252014 (225Hz) 30504029 (112Hz) 61117001 (56Hz)
DMP_GYRO_SF = 0x130 # 19*16+0 - 32-bit: gyro scaling factor
#9-axis
DMP_MAGN_THR_9X = 0x500 # 80*16+0
DMP_MAGN_LPF_THR_9X = 0x508 # 80*16+8
DMP_QFB_THR_9X = 0x50C # 80*16+12
#DMP RUNNING COUNTER
DMP_DMPRATE_CNTR = 0x124 # 18*16+4
#PEDOMETER
DMP_PEDSTD_BP_B = 0x31C # 49*16+12
DMP_PEDSTD_BP_A4 = 0x340 # 52*16+0
DMP_PEDSTD_BP_A3 = 0x344 # 52*16+4
DMP_PEDSTD_BP_A2 = 0x348 # 52*16+8
DMP_PEDSTD_BP_A1 = 0x34C # 52*16+12
DMP_PEDSTD_SB = 0x328 # 50*16+8
DMP_PEDSTD_SB_TIME = 0x32C # 50*16+12
DMP_PEDSTD_PEAKTHRSH = 0x398 # 57*16+8
DMP_PEDSTD_TIML = 0x32A # 50*16+10
DMP_PEDSTD_TIMH = 0x32E # 50*16+14
DMP_PEDSTD_PEAK = 0x394 # 57*16+4
DMP_PEDSTD_STEPCTR = 0x360 # 54*16+0
DMP_PEDSTD_STEPCTR2 = 0x3A8 # 58*16+8
DMP_PEDSTD_TIMECTR = 0x3C4 # 60*16+4
DMP_PEDSTD_DECI = 0x3A0 # 58*16+0
DMP_PEDSTD_SB2 = 0x3CE # 60*16+14
DMP_STPDET_TIMESTAMP = 0x128 # 18*16+8
DMP_PEDSTEP_IND = 0x134 # 19*16+4
DMP_PED_Y_RATIO = 0x110 # 17*16+0
# SMD
DMP_SMD_VAR_TH = 0x8DC # 141*16+12
DMP_SMD_VAR_TH_DRIVE = 0x8FC # 143*16+12
DMP_SMD_DRIVE_TIMER_TH = 0x8F8 # 143*16+8
DMP_SMD_TILT_ANGLE_TH = 0xB3C # 179*16+12
DMP_BAC_SMD_ST_TH = 0xB38 # 179*16+8
DMP_BAC_ST_ALPHA4 = 0xB4C # 180*16+12
DMP_BAC_ST_ALPHA4A = 0xB0C # 176*16+12
#WAKE ON MOTION
DMP_WOM_ENABLE = 0x40E # 64*16+14
DMP_WOM_STATUS = 0x406 # 64*16+6
DMP_WOM_THRESHOLD = 0x400 # 64*16+0
DMP_WOM_CNTR_TH = 0x40C # 64*16+12
#ACTIVITY RECOGNITION
DMP_BAC_RATE = 0x30A # 48*16+10
DMP_BAC_STATE = 0xB30 # 179*16+0
DMP_BAC_STATE_PREV = 0xB34 # 179*16+4
DMP_BAC_ACT_ON = 0xB60 # 182*16+0
DMP_BAC_ACT_OFF = 0xB70 # 183*16+0
DMP_BAC_STILL_S_F = 0xB10 # 177*16+0
DMP_BAC_RUN_S_F = 0xB14 # 177*16+4
DMP_BAC_DRIVE_S_F = 0xB20 # 178*16+0
DMP_BAC_WALK_S_F = 0xB24 # 178*16+4
DMP_BAC_SMD_S_F = 0xB28 # 178*16+8
DMP_BAC_BIKE_S_F = 0xB2C # 178*16+12
DMP_BAC_E1_SHORT = 0x920 # 146*16+0
DMP_BAC_E2_SHORT = 0x924 # 146*16+4
DMP_BAC_E3_SHORT = 0x928 # 146*16+8
DMP_BAC_VAR_RUN = 0x94C # 148*16+12
DMP_BAC_TILT_INIT = 0xB50 # 181*16+0
DMP_BAC_MAG_ON = 0xE10 # 225*16+0
DMP_BAC_PS_ON = 0x4A0 # 74*16+0
DMP_BAC_BIKE_PREFERENCE = 0xAD8 # 173*16+8
DMP_BAC_MAG_I2C_ADDR = 0xE58 # 229*16+8
DMP_BAC_PS_I2C_ADDR = 0x4B4 # 75*16+4
DMP_BAC_DRIVE_CONFIDENCE = 0x900 # 144*16+0
DMP_BAC_WALK_CONFIDENCE = 0x904 # 144*16+4
DMP_BAC_SMD_CONFIDENCE = 0x908 # 144*16+8
DMP_BAC_BIKE_CONFIDENCE = 0x90C # 144*16+12
DMP_BAC_STILL_CONFIDENCE = 0x910 # 145*16+0
DMP_BAC_RUN_CONFIDENCE = 0x914 # 145*16+4
DMP_BAC_MODE_CNTR = 0x960 # 150*16+0
DMP_BAC_STATE_T_PREV = 0xB94 # 185*16+4
DMP_BAC_ACT_T_ON = 0xB80 # 184*16+0
DMP_BAC_ACT_T_OFF = 0xB84 # 184*16+4
DMP_BAC_STATE_WRDBS_PREV = 0xB98 # 185*16+8
DMP_BAC_ACT_WRDBS_ON = 0xB88 # 184*16+8
DMP_BAC_ACT_WRDBS_OFF = 0xB8C # 184*16+12
DMP_BAC_ACT_ON_OFF = 0xBE2 # 190*16+2
DMP_PREV_BAC_ACT_ON_OFF = 0xBC2 # 188*16+2
DMP_BAC_CNTR = 0x302 # 48*16+2
#FLIP AND PICK-UP
DMP_FP_VAR_ALPHA = 0xF58 # 245*16+8
DMP_FP_STILL_TH = 0xF64 # 246*16+4
DMP_FP_MID_STILL_TH = 0xF48 # 244*16+8
DMP_FP_NOT_STILL_TH = 0xF68 # 246*16+8
DMP_FP_VIB_REJ_TH = 0xF18 # 241*16+8
DMP_FP_MAX_PICKUP_T_TH = 0xF4C # 244*16+12
DMP_FP_PICKUP_TIMEOUT_TH = 0xF88 # 248*16+8
DMP_FP_STILL_CONST_TH = 0xF6C # 246*16+12
DMP_FP_MOTION_CONST_TH = 0xF08 # 240*16+8
DMP_FP_VIB_COUNT_TH = 0xF28 # 242*16+8
DMP_FP_STEADY_TILT_TH = 0xF78 # 247*16+8
DMP_FP_STEADY_TILT_UP_TH = 0xF2C # 242*16+12
DMP_FP_Z_FLAT_TH_MINUS = 0xF38 # 243*16+8
DMP_FP_Z_FLAT_TH_PLUS = 0xF3C # 243*16+12
DMP_FP_DEV_IN_POCKET_TH = 0x4CC # 76*16+12
DMP_FP_PICKUP_CNTR = 0xF74 # 247*16+4
DMP_FP_RATE = 0xF0C # 240*16+12
#GYRO FSR
DMP_GYRO_SCALE = 0x48C # 72*16+12
#ACCEL FSR
DMP_ACC_SCALE = 0x1E0 # 30*16+0
DMP_ACC_SCALE2 = 0x4F4 # 79*16+4
#EIS AUTHENTIFICATION
DMP_EIS_AUTH_INPUT = 0xA04 # 160*16+4
DMP_EIS_AUTH_OUTPUT = 0xA00 # 160*16+0
#B2S
DMP_B2S_RATE = 0x308 # 48*16+8
#BRING TO SEE MOUNTING MATRIX 
DMP_B2S_MTX_00 = 0xD00 # 208*16+0
DMP_B2S_MTX_01 = 0xD04 # 208*16+4
DMP_B2S_MTX_02 = 0xD08 # 208*16+8
DMP_B2S_MTX_10 = 0xD0C # 208*16+12
DMP_B2S_MTX_11 = 0xD10 # 209*16+0
DMP_B2S_MTX_12 = 0xD14 # 209*16+4
DMP_B2S_MTX_20 = 0xD18 # 209*16+8
DMP_B2S_MTX_21 = 0xD1C # 209*16+12
DMP_B2S_MTX_22 = 0xD20 # 210*16+0
#DMP ORIENTATION PARAMETERS (Q30) INITIALISATION 
DMP_Q0_QUAT6 = 0x210 # 33*16+0
DMP_Q1_QUAT6 = 0x214 # 33*16+4
DMP_Q2_QUAT6 = 0x218 # 33*16+8
DMP_Q3_QUAT6 = 0x21C # 33*16+12

#================================================================================
# DMP CONSTANTS AND RELATED STUFF
#================================================================================

#DMP DATA SIZE
DMP_Header_Bytes = 2
DMP_header2_Bytes = 2
DMP_Raw_Accel_Bytes = 6
DMP_Raw_Gyro_Bytes = 6
DMP_Gyro_Bias_Bytes = 6
DMP_Compass_Bytes = 6
DMP_ALS_Bytes = 8 # Byte[0]: Dummy, Byte[2:1]: Ch0DATA, Byte[4:3]: Ch1DATA, Byte[6:5]: PDATA, Byte[7]: Dummy
DMP_Quat6_Bytes = 12 # 3 x 4_Bytes data (Q1, Q2, Q3, Q0 is deducted throught Q0^2+Q1^2+Q2^2+Q3^2=1
DMP_Quat9_Bytes = 14 # 3 x 4_Bytes data + 2_Bytes heading accuracy
DMP_Pedom_Quat6_Bytes = 6
DMP_Geomag_Bytes = 14 #same as Quat9, The quaternion data is scaled by 2^30
DMP_Pressure_Bytes = 6 # Byte [2:0]: Pressure data, Byte [5:3]: Temperature data
DMP_Gyro_Calibr_Bytes = 12 # Hardware unit scaled by 2^15
DMP_Compass_Calibr_Bytes = 12 # Hardware unit scaled by 2^16
DMP_Step_Detector_Bytes = 4
DMP_Accel_Accuracy_Bytes = 2
DMP_Gyro_Accuracy_Bytes = 2
DMP_Compass_Accuracy_Bytes = 2
DMP_Fsync_Detection_Bytes = 2
DMP_Pickup_Bytes = 2
DMP_Activity_Recognition_Bytes = 6 # Byte [0]: State-Start, Byte [1]: State-End, Byte [5:2]: timestamp
DMP_Secondary_On_Off_Bytes = 2
DMP_Footer_Bytes = 2
DMP_Maximum_Bytes = 14

#DMP Data_Output_Control_1 (from highest bit to lowest bit)
#also used for Header_1 Bitmap check in FIFO decoding
DMP_DO_Ctrl_1_Accel = 0x8000 #16 bit
DMP_DO_Ctrl_1_Gyro = 0x4000 #16 bit
DMP_DO_Ctrl_1_Compass = 0x2000 #16 bit
DMP_DO_Ctrl_1_ALS = 0x1000 #16 bit
DMP_DO_Ctrl_1_Quat6 = 0x0800 #32 bit 6 axis
DMP_DO_Ctrl_1_Quat9 = 0x0400 #32 bit 6 axis
DMP_DO_Ctrl_1_Pedom_Quat6 = 0x0200 #16 bit
DMP_DO_Ctrl_1_Geomag = 0x0100 # 32 bit + heading accuracy
DMP_DO_Ctrl_1_Pressure = 0x0080 #16 bit
DMP_DO_Ctrl_1_Gyro_Calibr = 0x0040 #32 bit
DMP_DO_Ctrl_1_Compass_Calibr = 0x0020 #32 bit
DMP_DO_Ctrl_1_Step_Detector = 0x0010 #Pedometer Step detector
DMP_DO_Ctrl_1_Header2 = 0x0008 #Header 2
DMP_DO_Ctrl_1_Step_Ind_2 = 0x0004 #Pedometer Step Indicator Bit 2
DMP_DO_Ctrl_1_Step_Ind_1 = 0x0002 #Pedometer Step Indicator Bit 1
DMP_DO_Ctrl_1_Step_Ind_0 = 0x0001 #Pedometer Step Indicator Bit 0

#DMP Data_Output_Control_2 (from highest bit to lowest bit)
#also used for Header_2 Bitmap check in FIFO decoding
DMP_DO_Ctrl_2_Accel_Accuracy = 0x4000
DMP_DO_Ctrl_2_Gyro_Accuracy = 0x2000
DMP_DO_Ctrl_2_Compass_Accuracy = 0x1000
DMP_DO_Ctrl_2_Fsync = 0x0800
DMP_DO_Ctrl_2_Pickup = 0x0400
DMP_DO_Ctrl_2_Batch_Mode_Enable = 0x0100
DMP_DO_Ctrl_2_Activity_Recog = 0x0080
DMP_DO_Ctrl_2_Secondary_On_Off = 0x0040

#DMP Interruption Masks
#Determine wich sensor needs to be on (32bits)
INV_NEEDS_ACCEL_MASK0 =  0b11100010100111101000111000001010
INV_NEEDS_ACCEL_MASK1 =  0b00000000000000000000011011101000
INV_NEEDS_GYRO_MASK0 =   0b11100110000000011000111000011000
INV_NEEDS_GYRO_MASK1 =   0b00000000000000000000100000011000
INV_NEEDS_COMPAS_MASK0 = 0b10000011000100000100100000001100
INV_NEEDS_COMPAS_MASK1 = 0b00000000000000000000000010000100
INV_NEEDS_PRES_MASK0 =   0b00010000000000000000000001000000
INV_NEEDS_PRES_MASK1 =   0b00000000000000000000000000000000

#DMP Data ready
DMP_Data_Ready_Gyro = 0x0001
DMP_Data_Ready_Accel = 0x0002
DMP_Data_Ready_Secondary_Compass = 0x0008

#DMP Event Control
DMP_Motion_Event_Control_BAC_Wearable = 0x8000
DMP_Motion_Event_Control_Activity_Recog_Pedom = 0x4000
DMP_Motion_Event_Control_Pedometer_Interrupt = 0x2000
DMP_Motion_Event_Control_Tilt_Interrupt = 0x1000
DMP_Motion_Event_Control_Significant_Motion_Det = 0x0800
DMP_Motion_Event_Control_Accel_Calibr = 0x0200
DMP_Motion_Event_Control_Gyro_Calibr = 0x0100
DMP_Motion_Event_Control_Compass_Calibr = 0x0080
DMP_Motion_Event_Control_9axis = 0x0040
DMP_Motion_Event_Control_BTS = 0x0020
DMP_Motion_Event_Control_Pickup = 0x0010
DMP_Motion_Event_Control_Geomag = 0x0008
DMP_Motion_Event_Control_Bring_Look_To_See = 0x0004
DMP_Motion_Event_Control_Activity_Recog_Pedom_Accel = 0x0002

DMP_SENSORS_2_ANDROID = {
    0 : 1, 1 : 4, 2 : 42, 3 : 43, 4 : 14, 5 : 16, 6 : 47, 7 : 18, 8 : 19,
    9 : 15, 10 : 11, 11 : 20, 12 : 2, 13 : 17, 14 : 46, 15 : 41, 16 : 9,
    17 : 10, 18 : 3, 19 : 45, 20 : 44}

#Android sensor control bits, 45 to 46 are trials
ANDROID_SENSORS_CTRL_BITS = { 0 : 0xFFFF, 1 : 0x8008, 2 : 0x0028, 3 : 0x0408,
    4 : 0x4048, 5 : 0x1008, 6 : 0x0088, 7 : 0xFFFF, 8 : 0xFFFF, 9 : 0x0808,
    10 : 0x8808, 11 : 0x0408, 12 : 0xFFFF, 13 : 0xFFFF, 14 : 0x2008, 15 : 0x0808,
    16 : 0x4008, 17 : 0x0000, 18 : 0x0018, 19 : 0x0010, 20 : 0x0108, 21 : 0xFFFF,
    22 : 0xFFFF, 23 : 0x8008, 24 : 0x0028, 25 : 0x0408, 26 : 0x4048, 27 : 0x1008,
    28 : 0x0088, 29 : 0x0808, 30 : 0x8808, 31 : 0x0408, 32 : 0xFFFF, 33 : 0xFFFF,
    34 : 0x2008, 35 : 0x0808, 36 : 0x4008, 37 : 0x0018, 38 : 0x0010, 39 : 0x0108,
    40 : 0xFFFF, 41 : 0x0000, 42 : 0x8008, 43 : 0x4048, 44 : 0xFDF8, 45 : 0xFFFF,
    46 : 0xFFFF, 47 : 0x4000}

DMP_SENSORS = {
    "ACCELEROMETER" : 0, "GYROSCOPE" : 1, "RAW_ACCELEROMETER" : 2, "RAW_GYROSCOPE" : 3,
    "MAGNETIC_FIELD_UNCALIBRATED" : 4, "GYROSCOPE_UNCALIBRATED" : 5, "ACTIVITY_CLASSIFICATON" : 6,
    "STEP_DETECTOR" : 7, "STEP_COUNTER" : 8, "GAME_ROTATION_VECTOR" : 9, "ROTATION_VECTOR" : 10,
    "GEOM_ROTATION_VECTOR" : 11, "GEOM_FIELD" : 12, "WAKEUP_SIGNIFICANT_MOTION" : 13,
    "FLIP_PICKUP" : 14, "WAKEUP_TILT_DETECTOR" : 15, "GRAVITY" : 16, "LINEAR_ACCELERATION" : 17,
    "ORIENTATION" : 18, "B2S" : 19, "ALL" : 20}

DMP_ACTIVITY = {"Drive": 0x01, "Walk": 0x02, "Run": 0x04, "Bike": 0x08, "Tilt": 0x10, "Still": 0x20}
DMP_SECONDARY = {"Gyro_Off": 0x01, "Gyro_On": 0x02, "Compass_Off": 0x04, "Compass_On": 0x08, "Prox_Off": 0x10, "Prox_On": 0x20}

#================================================================================
# LIBRARY ITSELF
#================================================================================
#
# i2c = i2c bus :                       Mandatory
# add = ICM20948 i2c adress :           Optional
# dmp = Bool True if D2M must be used : Optional
# debug = debug level :                 Optional
#
#         bit 0 : 01 : ICM Chip Level
#         bit 1 : 02 : DMP Chip Acces
#         bit 2 : 04 : DMP Fifo processing
#         bit 3 : 08 : DMP Output
#         bit 4 : 16 : ICM Register Access
#
#================================================================================

class ICM20948:

    #Initialise the IMU
    def __init__(self, i2c=None, addr=None, dmp= False, debug=17):
        
        #Initialize class variables
        self._bus = i2c
        self._addr = addr
        self._dmp = dmp
        self._debug = debug
        self._ready = False
        self._dmp_ready = False       
        
        #Variables for ICM or DMP
        self._bank = -1
        self._dmp_bank = -1
        self._fifo_buffer = bytearray(4096) # Chap 1.2 buffer is 4kB     
        self._android_sensor_bitmask_0 = 0
        self._android_sensor_bitmask_1 = 0
        self._dmp_data_out_ctl1 = 0
        self._dmp_data_out_ctl2 = 0
        self._dmp_data_intr_ctl = 0
        self._gyro_sf = 0
        self._gyro_sf_pll = 0
        
        #Other buffers
        self._rw_buffer = bytearray(1)
        self._buffer = bytearray(1)
        self._data = bytearray(14)
        self._data_ordered = bytearray(14)
        
        #Sensitivity 
        self._acc_s = 0 # acc
        self._gyro_s = 0 # gyro
        self._mag_s = AK_MAGNETOMETER_SENSITIVITY # mag
        
        #Bias variable for calibration
        self._accbias_en = False
        self._gyrbias_en = False
        self._magbias_en = False
        self._accbias = (0,0,0) # acc bias
        self._gyrbias = (0,0,0) # gyr bias
        self._magbias = (0,0,0) # mag bias
        
        #Sensor storage from FIFO
        self._acc = (0,0,0)
        self._acc_ac = 0
        self._gyr = (0,0,0)
        self._gyr_ac = 0
        self._mag = (0,0,0)
        self._mag_ac = 0
        self._gyr_cal = (0,0,0)
        self._mag_cal = (0,0,0)
        self._quat6 = (0,0,0)
        self._pquat6 = (0,0,0)
        self._quat9 = (0,0,0)
        self._quat9_ac = 0
        self._gquat6 = (0,0,0)
        self._gquat6_ac = 0
        self._press = 0
        self._temp = 0
        self._als = 0
        self._steps = 0
        self._fsynch = 0
        self._pickup = 0
        self._sec_onoff = 0
        self._gyr_count = 0
        self._act_recog = (0,0,0)
         
        #Quaterion and Orientation
        self.q = [1.0, 0.0, 0.0, 0.0] # vector to hold quaternion
        self._north_angle = 0 # angle from magnetic north to real north
        self.beta = sqrt(3.0 / 4.0) * radians(40) # compute beta (see README)
        self.pitch = 0
        self.heading = 0
        self.roll = 0
        self.lasttime = ticks_us()
        self.newtime = 0
        
        #Scannig i2c bus
        if addr is None :
            devices = set(self._bus.scan())
            mpus = devices.intersection(set(ICM_ADDRESS))
            nb_of_mpus = len(mpus)
            if nb_of_mpus == 0:
                self._ready = False
                raise ValueError("No ICM20948 detected")
            elif nb_of_mpus == 1:
                self._addr = mpus.pop()
                self._dbg(1,"DEVICE FOUND AT ADDRESS",hex(self._addr))
            else:
                raise ValueError("Two ICM20948 detected: must specify a device address")
        else :
            self._addr = addr
        
        #Check Chip ID telling Slave 0 to read 
        if not self.read(0, ICM_WHO_AM_I) == ICM_CHIP_ID:
            raise RuntimeError("Unable to find ICM20948")
        else :
            self._dbg(1,"CHIP ID NUMBER IS",hex(ICM_CHIP_ID))
        
        #Reset the Chip 
        self.reg_config(0,ICM_PWR_MGMT_1, ICM_PWR_MGMT_1_RESET, True)
        sleep_ms(10)
        
        #Set Clock Auto 
        self.write(0, ICM_PWR_MGMT_1, ICM_PWR_MGMT_1_CLOCK_AUTO)
        
        #Put all sensor On
        self.write(0, ICM_PWR_MGMT_2, 0x00)
        
        #Configure I2C Master Clock
        self.write(3, ICM_I2C_MST_CTRL, ICM_I2C_MST_CTRL_NSR | 0x07)  #I2C MSTR CLOCK = 07 = 345,6kHz
        
        #Activate I2C Master
        self.reg_config(0, ICM_USER_CTRL, ICM_USER_CTRL_I2C_MST_EN, True)
        
        #Configure ODR
        #self.write(2, ICM_ODR_ALIGN_EN, 0x01)
        
        # Check if we cas access Magnetometer
        # enabling Slave 0 Read AK_WIA thought AK_I2C_ADD
        # reading result throught ICM_EXT_SLV_SENS_DATA_00
        self.slave_config(0, AK_I2C_ADDR, AK_WIA2, 1, True, True, False, False, False)
        if self.read(0, ICM_EXT_SLV_SENS_DATA_00) != AK_CHIP_ID:
            self._dbg(1, "AK09916 Magnetometer Chip Not Found... ")
        else :
            self._dbg(1, "AK09916 Magnetometer Chip Found... ")
        
        # Reset the magnetometer
        # Write Slave 0 WK_CNTL3 the Reset bt OK_CNTL3_RESET
        # Then enable Slave 0 to read AK_CNTL3 throught AK_I2C_ADD
        # reading result throught ICM_EXT_SLV_SENS_DATA_00
        self.slave_config(0, AK_I2C_ADDR, AK_CNTL3, 1, False, True, False, False, False,AK_CNTL3_RESET)
        self.slave_config(0, AK_I2C_ADDR, AK_CNTL3, 1, True, True, False, False, False)
        while self.read(0, ICM_EXT_SLV_SENS_DATA_00) == 0x01: #Loop util reset bit remains on
            sleep_ms(10)
        self._dbg(1, "AK09916 Magnetometer Chip Reseted... ")
        
        #If we raised here, everything is fine
        self._ready = True
        
        #finally we start continuous ready of magnetometer
        if dmp :
            self._dmp_ready = True
            self.DMP_config()
        else :
            self.ICM_config()

#=========== Below are all the chip settings function (ICM direct) ===========================

    #Get accelerator sensitivity
    @property
    def get_acc_sensitivity(self):
        # Read accelerometer full scale range
        scale = (self.read(2, ICM_ACCEL_CONFIG_1) & 0x06) >> 1
        return (1 / ACC_SENSITIVITY_FACTOR[scale])

    @property
    def get_gyro_sensitivity(self):
        # Read back the degrees per second rate
        scale = (self.read(2, ICM_GYRO_CONFIG_1) & 0x06) >> 1
        return (1 / GYRO_SENSITIVITY_FACTOR[scale])

    #Set the accelerometer sample rate in Hz (125Hz - 1.125 kHz)
    def set_acc_sample_rate(self, rate=125):
        # Sample_rate = 1125 Hz / (1 + Acc_sample_rate_divider)
        # So Acc_sample_rate_divider = (1125 / sample_rate) - 1 
        rate = int((1125.0 / rate) - 1)
        self.write(2, ICM_ACCEL_SMPLRT_DIV_1, rate >> 8)
        self.write(2, ICM_ACCEL_SMPLRT_DIV_2, rate & 0xff)
        
    #Set the acceleration full scale range to +- supplied value
    #uses ACC_SCALE_RANGE = {2: 0b00, 4: 0b01, 8: 0b10, 16: 0b11}
    def set_acc_full_scale(self, scale=16):
        
        #Set Acc Full scale in ICM Registers      
        value = self.read(2, ICM_ACCEL_CONFIG_1) & ICM_ACCEL_CONFIG_1_ACCEL_FS_SEL_MASK
        acc_range = ACC_SCALE_RANGE[scale]
        value |= acc_range << 1
        self.write(2, ICM_ACCEL_CONFIG_1, value)
        self._acc_s = self.get_acc_sensitivity
        #Set DMP Acc Full scale in DMP Memory
        if self._dmp :
            #Configure Acceleration scaling to DMP
            #Internally DMP scales Acc Raw data to 2^25 = 1g when FSR = 4g
            #Inv library tells to write DMP_ACC_SCALE = 0x04000000 = 2^26 for 4g 
            #We extrapolated to 2G : 2^25 // 4G : 2^26 // 8G : 2^27 // 16G : 2^28
            DMP_ACC_SCALE_FACTOR = bytearray(4)
            value = 0x01 << (25 + acc_range)
            DMP_ACC_SCALE_FACTOR[0] = value >> 24
            DMP_ACC_SCALE_FACTOR[1] = value >> 16
            DMP_ACC_SCALE_FACTOR[2] = value >> 8
            DMP_ACC_SCALE_FACTOR[3] = value & 0xFF
            self.DMP_write(DMP_ACC_SCALE, DMP_ACC_SCALE_FACTOR)
            #In order to output hardaware unit data as configured FSR write
            #Inv library tells to write DMP_ACC_SCALE2 = 0x40000 = 2^18 for 4g
            #We extrapolated to 1G : 2^17 // 4G : 2^18 // 8G : 2^19 // 16G : 2^20
            DMP_ACC_SCALE2_FACTOR = bytearray(4)
            value = value >> 8
            DMP_ACC_SCALE2_FACTOR[0] = value >> 24
            DMP_ACC_SCALE2_FACTOR[1] = value >> 16
            DMP_ACC_SCALE2_FACTOR[2] = value >> 8
            DMP_ACC_SCALE2_FACTOR[3] = value & 0xFF
            self.DMP_write(DMP_ACC_SCALE2, DMP_ACC_SCALE2_FACTOR)              
        #Finally Adjust sensitivity    
        self._gyro_s = 1 / ACC_SENSITIVITY_FACTOR[acc_range]

    #Configure the accelerometer low pass filter
    def set_acc_low_pass(self, enabled=True, mode=5):
        value = self.read(2, ICM_ACCEL_CONFIG_1) & ICM_ACCEL_CONFIG_1_ACCEL_DLPFCFG_MASK
        if enabled:
            value |= 0b1
        value |= (mode & 0x07) << 4
        self.write(2, ICM_ACCEL_CONFIG_1, value)

    #Set the gyro sample rate in Hz
    def set_gyro_sample_rate(self, rate=125):
        # Sample_rate = 1125 Hz / (1 + Gyro_sample_rate_divider)
        # So Gyro_sample_rate_divider = (1125 / sample_rate) - 1
        rate = int((1125.0 / rate) - 1)
        self.write(2, ICM_GYRO_SMPLRT_DIV, rate & 0xff)
       
    #Set the gyro full scale range to +- supplied value
    #uses GYRO_SCALE_RANGE = {250: 0b00, 500: 0b01, 1000: 0b10, 2000: 0b11}
    def set_gyro_full_scale(self, scale=250):
        
        #Set Gyro Full scale in ICM Registers
        value = self.read(2, ICM_GYRO_CONFIG_1) & ICM_GYRO_CONFIG_1_GYRO_FS_SEL_MASK
        gyro_range = GYRO_SCALE_RANGE[scale]
        value |= gyro_range << 1
        self.write(2, ICM_GYRO_CONFIG_1, value)
        self._gyro_s = self.get_gyro_sensitivity
        #Set DMP Gyro Full scale in DMP Memory
        if self._dmp :        
            #Inv library tells to write DMP_GYRO_SCALE = 0x10000000 = 2^28 for 2000 dps 
            #We extrapolated to 250 : 2^25 // 500 : 2^26 // 1000 : 2^27 // 2000 : 2^28      
            DMP_GYRO_SCALE_FACTOR = bytearray(4)
            value = 0x01 << (25 + gyro_range)
            DMP_GYRO_SCALE_FACTOR[0] = value >> 24
            DMP_GYRO_SCALE_FACTOR[1] = value >> 16
            DMP_GYRO_SCALE_FACTOR[2] = value >> 8
            DMP_GYRO_SCALE_FACTOR[3] = value & 0xFF
            self.DMP_write(DMP_GYRO_SCALE, DMP_GYRO_SCALE_FACTOR)
        #Finally Adjust sensitivity    
        self._gyro_s = 1 / GYRO_SENSITIVITY_FACTOR[gyro_range]

    #Configure the gyro low pass filter
    def set_gyro_low_pass(self, enabled=True, mode=5):
        value = self.read(2, ICM_GYRO_CONFIG_1) & ICM_GYRO_CONFIG_1_GYRO_DLPCFCFG_MASK
        if enabled:
            value |= 0b1
        value |= (mode & 0x07) << 4
        self.write(2, ICM_GYRO_CONFIG_1, value)

    #Configure the accelerometer low pass filter
    def set_temp_low_pass(self, enabled=True, mode=1):
        if enabled :
            value = mode & 0x07
        else :
            value = 0x00
        self.write(2, ICM_TEMP_CONFIG, value)

#=========== Below are all the sensor reading functions ===================================

    #Calibrate accelerometer
    def acc_cal(self, enable=True, timeout=2000):
        if enable :
            timeout *= 1000
            accmax = list(self.acc()) # Initialise max and min lists with current values
            accmin = accmax[:]
            self.lasttime = ticks_us()
            while ((ticks_us() - self.lasttime) < timeout) :
                accxyz = self.acc()
                for x in range(3):
                    accmax[x] = max(accmax[x], accxyz[x])
                    accmin[x] = min(accmin[x], accxyz[x])
            self._accbias = tuple(map(lambda a, b: (a +b)/2, accmin, accmax))
            self._accbias_en = True
            self._dbg(1, "Accelerometer calibration done",self._accbias)
        else :
            self._accbias = (0,0,0)
            self._accbias_en = False
            self._dbg(1, "Accelerometer calibration desactivated",self._accbias) 

    #Calibrate gyroscope
    def gyro_cal(self, enable=True, timeout=2000):
        if enable :
            timeout *= 1000
            gyrmax = list(self.gyro())
            gyrmin = gyrmax[:]
            self.lasttime = ticks_us()
            while ((ticks_us() - self.lasttime) < timeout) :
                gyrxyz = self.gyro()
                for x in range(3):
                    gyrmax[x] = max(gyrmax[x], gyrxyz[x])
                    gyrmin[x] = min(gyrmin[x], gyrxyz[x])
            self._gyrbias = tuple(map(lambda a, b: (a +b)/2, gyrmin, gyrmax))
            self._gyrbias_en = True
            #Proceed for DMP (need to be debugged)
            """if self._dmp_ready :
                dmp_gyrbias_x = int(2**15*self._gyrbias[0])
                dmp_gyrbias_y = int(2**15*self._gyrbias[1])
                dmp_gyrbias_z = int(2**15*self._gyrbias[2])
                DMP_GYR_X = bytearray(2)
                DMP_GYR_X[0] = dmp_gyrbias_x >> 16
                DMP_GYR_X[1] = dmp_gyrbias_x & 0xFF
                self.DMP_write(DMP_GYRO_BIAS_X, DMP_GYR_X)
                self.DMP_write(DMP_GYRO_BIAS_Y, DMP_GYR_X)
                self.DMP_write(DMP_GYRO_BIAS_Z, DMP_GYR_X)
                #Reset DMP
                self.reg_config(0, ICM_USER_CTRL, ICM_USER_CTRL_DMP_RST, True)"""    
            self._dbg(1,"Gyroscope calibration done",self._gyrbias)
        else :
            self._gyrbias = (0,0,0)
            self._gyrbias_en = False
            self._dbg(1,"Gyroscope calibration desactivated",self._gyrbias)  

    #Calibrate magnetometer
    def mag_cal(self, enable=True, timeout=2000):
        if enable :
            timeout *= 1000
            magmax = list(self.mag()) # Initialise max and min lists with current values
            magmin = magmax[:]
            self.lasttime = ticks_us()
            while ((ticks_us() - self.lasttime) < timeout) :
                magxyz = self.mag()
                for x in range(3):
                    magmax[x] = max(magmax[x], magxyz[x])
                    magmin[x] = min(magmin[x], magxyz[x])
            self._magbias = tuple(map(lambda a, b: (a +b)/2, magmin, magmax))
            self._magbias_en = True
            self._dbg(1,"Magnetometer calibration done",self._magbias)
        else :
            self._magbias = (0,0,0)
            self._magbias_en = False
            self._dbg(1,"Magnetometer calibration desactivated",self._magbias)  

    #Read the current IMU temperature
    def temp(self):
        # PWR_MGMT_1 defaults to leave temperature enabled
        temp_raw_bytes = self.read(0, ICM_TEMP_OUT_H, 2)
        temp_raw = struct.unpack('>h', bytearray(temp_raw_bytes))[0]
        temp_deg_c = ((temp_raw - ICM_ROOM_TEMP_OFFSET) / ICM_TEMPERATURE_SENSITIVITY) + ICM_TEMPERATURE_DEGREES_OFFSET
        return temp_deg_c

    #Read acceleration data
    def acc(self):
        if self._dmp_ready :
            self.DMP_fifo_proceed()
        else :
            data = self.read(0,ICM_ACCEL_XOUT_H, 6)
            ax, ay, az = unpack_from(">3h", data)
            ax *= self._acc_s
            ay *= self._acc_s
            az *= self._acc_s
            if self._accbias_en :
                ax -= self._accbias[0]
                ay -= self._accbias[1]
                az -= self._accbias[2]
            self._acc = ax, ay, az
        return self._acc

    #Read gyroscope data
    def gyro(self):
        if self._dmp_ready :
            self.DMP_fifo_proceed()
        else :
            data = self.read(0, ICM_GYRO_XOUT_H, 6)
            gx, gy, gz = unpack_from(">3h", data)
            gx *= self._gyro_s
            gy *= self._gyro_s      
            gz *= self._gyro_s
            if self._gyrbias_en :
                gx -= self._gyrbias[0]
                gy -= self._gyrbias[1]
                gz -= self._gyrbias[2]
            self._gyr = gx, gy, gz
        return self._gyr

    #Read magnetometer data straight for slave DATA_01 (linked to AK_HXL)
    def mag(self):
        if self._dmp_ready :
            self.DMP_fifo_proceed()
        else :
            data = self.read(0, ICM_EXT_SLV_SENS_DATA_01, 6)
            mx, my, mz = unpack_from("<3h", data)
            mx *= self._mag_s
            my *= self._mag_s
            mz *= self._mag_s
            if self._magbias_en :
                mx -= self._magbias[0]
                my -= self._magbias[1]
                mz -= self._magbias[2]
            self._mag = mx, my, mz
        return self._mag

#===========Below are all internal communication functions ===================================

    #Switch register bank
    def bank(self, bank):
        if not self._bank == bank:
            self._rw_buffer = bytearray(1)
            self._rw_buffer[0]=bank << 4
            self._bus.writeto_mem(self._addr, ICM_BANK_SEL, self._rw_buffer)
            self._dbg(16,"I2C Write\tSetBank", hex(bank))
            self._bank = bank

    #Configure Register
    def reg_config(self, bank, reg, ctrl, enable=True):
        self.bank(bank)
        value = self.read(bank, reg)
        if enable :
            value |= ctrl
        else :
            value &= ~ctrl
        self.write(bank, reg, value)
        sleep_ms(5)

    #Write single byte to the sensor
    def write(self, bank, reg, value):
        self.bank(bank)
        self._rw_buffer = bytearray(1)
        self._rw_buffer[0]=value
        self._bus.writeto_mem(self._addr, reg, self._rw_buffer)
        self._dbg(16,"I2C Write\tBank", hex(bank), "Reg",hex(reg),"Value",hex(value))

    #Write byte(s) to the sensor
    def write_bytes(self, bank, reg, value):
        self.bank(bank)
        self._bus.writeto_mem(self._addr, reg, value)
        self._dbg(16,"I2C Write\tBank", hex(bank), "Reg", hex(reg),"Value",bytes(value))

    #Read byte(s) from the ICM
    def read(self, bank, reg, length=1):
        self.bank(bank)
        self._rw_buffer = bytearray(length)
        self._bus.readfrom_mem_into(self._addr, reg , self._rw_buffer)
        self._dbg(16,"I2C Read \tBank", hex(bank), "Reg", hex(reg),"Value",bytes(self._rw_buffer))
        if length == 1 :
            return self._rw_buffer[0]
        else :
            return self._rw_buffer

    #I2C Master Slave configuration
    def slave_config(self, slave, addr, reg, length, RnW, En, Swp, Dis, Grp, DO = None):
        if (slave > 4) :
            return
        #Recalculate I2C_SLx_adress
        i2c_slv_addr = ICM_I2C_SLV0_ADDR + 4 * slave
        i2c_slv_reg = ICM_I2C_SLV0_REG + 4 * slave
        i2c_slv_ctrl = ICM_I2C_SLV0_CTRL + 4 * slave
        i2c_slv_do = ICM_I2C_SLV0_DO + 4 * slave
        
        if RnW :
            addr |= ICM_I2C_SLV_ADDR_RNW
        self.write(3, i2c_slv_addr, addr)
        
        if DO != None :
            self.write(3, i2c_slv_do, DO)
            
        self.write(3, i2c_slv_reg, reg)
        
        slv_ctrl = length
        if En :
            slv_ctrl |= ICM_I2C_SLV_CTRL_SLV_ENABLE
        if Swp :
            slv_ctrl |= ICM_I2C_SLV_CTRL_BYTE_SWAP
        if Dis :
            slv_ctrl |= ICM_I2C_SLV_CTRL_REG_DIS
        if Grp :
            slv_ctrl |= ICM_I2C_SLV_CTRL_REG_GROUP
        self.write(3, i2c_slv_ctrl, slv_ctrl)
        
        #Activate I2C Master so I2C_slave setup can be propagated to slave itself
        #self.reg_config(0, ICM_USER_CTRL, ICM_USER_CTRL_I2C_MST_EN, True)

    def reset_FIFO(self):
        self.write(0, ICM_FIFO_RST,0x1F)
        sleep_ms(5)
        self.write(0, ICM_FIFO_RST,0x1E)

    #ICM Mag config when not using DMP
    def ICM_config(self):
        
        #Enable the slave 0 to write (False) to AK_I2C magnetometer, by writing 1 byte MODE_100Hz to AK_CNTL2 REG
        self.slave_config(0, AK_I2C_ADDR, AK_CNTL2, 1 , False, True, False, False, False, AK_CNTL2_MODE_100HZ)
        
        #Enable the slave to read (True) from AK_ST1 to AK_ST2 (9 bytes) including magnetometer values AK_HXL
        self.slave_config(0, AK_I2C_ADDR, AK_ST1, 9, True, True, False, False, False)
        
        #Configure scales, SR, LP and get sensitivity values
        self.set_gyro_sample_rate()
        self.set_gyro_low_pass(enabled=True, mode=5)
        self.set_gyro_full_scale(2000)
        self.set_acc_sample_rate()
        self.set_acc_low_pass(enabled=True, mode=5)
        self.set_acc_full_scale(4)
        self.set_temp_low_pass(enabled=True, mode=1)

        #Set Low Power On
        self.reg_config(0,ICM_PWR_MGMT_1, ICM_PWR_MGMT_1_LP, True)
        
    #Status
    @property
    def ready(self):
        return self._ready

    #DMP Status
    @property
    def dmp_ready(self):
        return self._dmp_ready

    #Only for debugging purposes
    def _dbg(self, level,  *args, **kwargs):
        if level & self._debug:
            print("DBG:\t",LIBNAME,":\t", " ===>" if (level & 16) else "", *args, **kwargs)

#=================== Below are all DMP related functions ==============================

    #Switch memory bank
    def DMP_bank(self, dmp_bank):
        if not self._dmp_bank == dmp_bank :
            self.write(0, ICM_MEM_BANK_SEL, dmp_bank)
            self._dmp_bank = dmp_bank

    #Load the DMP blob to the Chipset
    def DMP_load_firmware(self, burstmode = True):
        
        from icm20948_img_dmp3a import dmp_img
        mem_bank = 0
        start_address = DMP_LOAD_START
        data_pos = 0
        
        # Write DMP firmware to memory
        while data_pos < len(dmp_img):
            write_len = min((DMP_MEM_BANK_SIZE - start_address, len(dmp_img[data_pos:])))
            data = dmp_img[data_pos:data_pos + write_len]
            address = start_address
            self.DMP_bank(mem_bank)
            
            #Write firmware Byte per byte (Original but slow)
            if not burstmode :
                for d in data:
                    self.write(0, ICM_MEM_START_ADDR, address)
                    self.write(0, ICM_MEM_R_W, d)
                    address += 1    
            #Write firmware in burst mode (up to 256 byte at a time : Damn fast)
            else :
                self.write(0, ICM_MEM_START_ADDR, address)
                self.write_bytes(0, ICM_MEM_R_W, bytes(data))
            
            data_pos += write_len
            mem_bank += 1
            start_address = 0
            text = "\rDBG:\t ICM20948 : \t Uploading DMP Microcode {:.0f}%".format(100*data_pos/len(dmp_img))
            print(text, end="\r")
        self._dbg(1,"DMP Firmware Upload Successfull !")

    #Configure Digital Motion Processor
    def DMP_config(self):
        
        #Enable (En = True) SLV0 to read (RnW = True and nothing in DO) 10 byte AK09916 RSV2 register
        #Reserved data is in Big Indian so let's swap byte (Swp True)
        #Data are in group of 2 bytes (Grp = True)
        #    slave_config(sl, addr,       reg,     len, RnW, En,   Swp,  Dis,   Grp, DO)
        self.slave_config(0, AK_I2C_ADDR, AK_RSV2, 10, True, True, True, False, True)
        
        #Enable (En = True) SLV1 to write (RnW = False) 1 byte to AK09916 CTLN2 register
        #telling to run Single Mode (DO = AK_CNTL2_MODE_SINGLE)
        #No need of grouped, big indian or whater,
        #    slave_config(sl, addr,       reg,      len, RnW,   En,   Swp,   Dis,   Grp,   DO)
        self.slave_config(1, AK_I2C_ADDR, AK_CNTL2, 1,   False, True, False, False, False, AK_CNTL2_MODE_SINGLE)
        
        #Configure ODR to 68,75 Hz = 1100/2**4
        self.write(3, ICM_I2C_MST_ODR_CONFIG, 0x04)
               
        #Place in LP Mode Only I2C_master and be sure ACC and GYRO are not in LP Mode
        self.reg_config(0, ICM_LP_CFG,  ICM_LP_CFG_MST, True)
        self.reg_config(0, ICM_LP_CFG,  ICM_LP_CFG_ACC | ICM_LP_CFG_GYRO , False)
        
        #Disable DMP and FIFO
        self.reg_config(0,ICM_USER_CTRL, ICM_USER_CTRL_DMP_EN | ICM_USER_CTRL_FIFO_EN , False)
        
        #Set Gyro full scale range 2000 dps
        #self.set_gyro_full_scale(2000) # Global setup below
        #Thing are now done after DMP setup
        
        #Set Acc full scale range 4g
        #self.set_acc_full_scale(4)  # Global setup below
        #Thing are now done after DMP setup
        
        #Enable Gyro DLPF
        self.set_gyro_low_pass(True, mode=0)
        
        #Turn Off whatever whould be configured as FIFO
        self.write(0, ICM_FIFO_EN_1, 0x00)
        self.write(0, ICM_FIFO_EN_2, 0x00)
        
        #Reset FIFO
        self.reset_FIFO()
        
        #Turn Off data ready interrupt
        self.reg_config(0,ICM_INT_ENABLE_1,0x1,False)
        
        #Upload DMP firmware
        self.DMP_load_firmware()
        
        #Write the 2 byte Firmware Start Value to ICM PRGM_STRT_ADDRH/PRGM_STRT_ADDRL
        self._buffer = bytearray(2)
        self._buffer[0] = DMP_START_ADDRESS >> 8
        self._buffer[1] = DMP_START_ADDRESS & 0xff
        self.write_bytes(2, ICM_PRGM_START_ADDRH, self._buffer)
        
        #Set the Hardware Fix Disable register to 0x48
        self.write(0, ICM_HW_FIX_DISABLE,0x48)
        
        #Set the Single FIFO Priority Select register to 0xE4
        self.write(0, ICM_SINGLE_FIFO_PRIORITY_SEL,0xE4)
        
        #Configure Acceleration and Gyroscope : Ranges, Samples Rates and DMP scaling factors 
        self.set_acc_full_scale(4)
        self.set_gyro_full_scale(2000)
        self.set_acc_sample_rate(56.25)
        self.set_gyro_sample_rate(56.25)
        self.DMP_gyro_scaling(56.25, 2000)
        self.DMP_acc_scaling(56.25)
        
        #Configure Compass Mount Matrix
        #As explained on top Matrix will be
        #  [[ 1  0  0]  =  [[ 00 01 02]
        #   [ 0 -1  0]      [ 10 11 12]
        #   [ 0  0 -1]]     [ 20 21 22]]
        # 1 value need to be scaled from hardware unit to uT
        #Inside DMP, AK9916 output 16bit signed (+/- 32752 corresponding to +/-4912uT
        #1unit = 0.15 uT
        #Max 2^30 * 0,15 = 161061273 = 0x9999999
        #-0x9999999 = 0xF6666667
        #DMP Compass Output will be in uT
        
        DMP_COMPAS_MOUNT_MATRIX_SCALED_ZERO = [0x00, 0x00, 0x00, 0x00]
        DMP_COMPAS_MOUNT_MATRIX_SCALED_PLUS1 = [0x09, 0x99, 0x99, 0x99]
        DMP_COMPAS_MOUNT_MATRIX_SCALED_MINUS1 = [0xF6, 0x66, 0x66, 0x67]
        self.DMP_write(DMP_CPASS_MTX_00, DMP_COMPAS_MOUNT_MATRIX_SCALED_PLUS1)
        self.DMP_write(DMP_CPASS_MTX_01, DMP_COMPAS_MOUNT_MATRIX_SCALED_ZERO)
        self.DMP_write(DMP_CPASS_MTX_02, DMP_COMPAS_MOUNT_MATRIX_SCALED_ZERO)
        self.DMP_write(DMP_CPASS_MTX_10, DMP_COMPAS_MOUNT_MATRIX_SCALED_ZERO)
        self.DMP_write(DMP_CPASS_MTX_11, DMP_COMPAS_MOUNT_MATRIX_SCALED_MINUS1)
        self.DMP_write(DMP_CPASS_MTX_12, DMP_COMPAS_MOUNT_MATRIX_SCALED_ZERO)
        self.DMP_write(DMP_CPASS_MTX_20, DMP_COMPAS_MOUNT_MATRIX_SCALED_ZERO)
        self.DMP_write(DMP_CPASS_MTX_21, DMP_COMPAS_MOUNT_MATRIX_SCALED_ZERO)
        self.DMP_write(DMP_CPASS_MTX_22, DMP_COMPAS_MOUNT_MATRIX_SCALED_MINUS1)
        
        #Configure B2S Mounting Matrix
        # Values taken grom InvenSense Nucleo Example thank Sparkfun
        DMP_B2S_MOUNT_MATRIX_SCALED_ZERO = [0x00, 0x00, 0x00, 0x00]
        DMP_B2S_MOUNT_MATRIX_SCALED_PLUS1 = [0x40, 0x00, 0x00, 0x00]
        
        self.DMP_write(DMP_B2S_MTX_00, DMP_B2S_MOUNT_MATRIX_SCALED_PLUS1)
        self.DMP_write(DMP_B2S_MTX_01, DMP_B2S_MOUNT_MATRIX_SCALED_ZERO)
        self.DMP_write(DMP_B2S_MTX_02, DMP_B2S_MOUNT_MATRIX_SCALED_ZERO)
        self.DMP_write(DMP_B2S_MTX_10, DMP_B2S_MOUNT_MATRIX_SCALED_ZERO)
        self.DMP_write(DMP_B2S_MTX_11, DMP_B2S_MOUNT_MATRIX_SCALED_PLUS1)
        self.DMP_write(DMP_B2S_MTX_12, DMP_B2S_MOUNT_MATRIX_SCALED_ZERO)
        self.DMP_write(DMP_B2S_MTX_20, DMP_B2S_MOUNT_MATRIX_SCALED_ZERO)
        self.DMP_write(DMP_B2S_MTX_21, DMP_B2S_MOUNT_MATRIX_SCALED_ZERO)
        self.DMP_write(DMP_B2S_MTX_22, DMP_B2S_MOUNT_MATRIX_SCALED_PLUS1)
                      
        #Configure the Compass Time Buffer
        #The I2C Master ODR Configuration (see above) sets the magnetometer read rate to 68.75Hz.
        #Let's set the Compass Time Buffer to 69 (Hz) = 0x45.
        DMP_CPASS_TIME_BUFFER_FACTOR = [0x00, 0x45]
        self.DMP_write(DMP_CPASS_TIME_BUFFER, DMP_CPASS_TIME_BUFFER_FACTOR)
        
        #If needed can set DMP Output Data Rate ODR = (DMP running rate / ODR ) - 1
        # Here 0 means Compass is running same speed than DMP (1 would be half speed)
        self.DMP_odr(DMP_ODR_CPASS,0)
        #self.DMP_write(DMP_ODR_CNTR_CPASS_CALIBR, 0)

    #Read fifo count
    def DMP_fifo_count(self):
        data = self.read(0,ICM_FIFO_COUNTH, 2)
        count = ((data[0] & 0x1F ) << 8 ) | data[1]
        self._dbg(4, "FIFO {:.0f} bytes".format(count))
        return count

    #Read and decode Fifo
    def DMP_fifo_read(self):
        
        #Read FIFO Count
        fcount = self.DMP_fifo_count()
        if(fcount == 0) :
            return
        self.newtime = ticks_ms()
        
        #Read Header
        if (fcount < DMP_Header_Bytes) :
            return
        self._data = self.read(0, ICM_FIFO_R_W, DMP_Header_Bytes)
        header = self._data[0]<<8 | self._data[1]
        fcount -= DMP_Header_Bytes  #Decrease of Header size
        self._dbg(4, "\tHeader is",hex(header))
        
        #Read Header2
        header2 = 0
        if (header & DMP_DO_Ctrl_1_Header2) != 0 : #Check if header contains header 2 bit
            if (fcount < DMP_header2_Bytes) : #Check FIFO has enough data, read again if not
                fcount = self.DMP_fifo_count()
            if (fcount < DMP_header2_Bytes) :
                return
            for i in range(DMP_header2_Bytes) :  #Read header 2 (2 Bytes)
                data = self.read(0, ICM_FIFO_R_W)
                header2 |= data << (8 - (i * 8))
            fcount -= DMP_header2_Bytes  #Decrease of Header 2 size
            self._dbg(4, "\tHeader 2 is",hex(header2))
        
        #Check bit per bit header in order
        #Acceleration
        if (header & DMP_DO_Ctrl_1_Accel) != 0 :
            if (fcount < DMP_Raw_Accel_Bytes) :
                fcount = self.DMP_fifo_count()
            if (fcount < DMP_Raw_Accel_Bytes) :
                return
            self._data = self.read(0, ICM_FIFO_R_W, DMP_Raw_Accel_Bytes)
            ax,ay,az = unpack_from(">3h", self._data)
            ax *= self._acc_s #adjust result with sensiticity
            ay *= self._acc_s
            az *= self._acc_s
            self._acc = ax, ay, az
            self._dbg(8, "FIFO Acceleration\tax {:.4f}\tay {:.4f}\taz {:.4f}".format(ax,ay,az))
            fcount -= DMP_Raw_Accel_Bytes  #Decrease of Acc
        
        #Gyroscope
        if (header & DMP_DO_Ctrl_1_Gyro) != 0 :
            if (fcount < (DMP_Raw_Gyro_Bytes + DMP_Gyro_Bias_Bytes)) :
                fcount = self.DMP_fifo_count()
            if (fcount < (DMP_Raw_Gyro_Bytes + DMP_Gyro_Bias_Bytes)) :
                return
            self._data = self.read(0, ICM_FIFO_R_W, DMP_Raw_Gyro_Bytes + DMP_Gyro_Bias_Bytes)
            gx,gy,gz,gbx,gby,gbz = unpack_from(">6h", self._data)
            gx *= self._gyro_s #adjust result with sensiticity
            gy *= self._gyro_s
            gz *= self._gyro_s
            self._gyr = gx, gy, gz
            self._dbg(8, "FIFO Gyroscope\tgx {:.4f}\tgy {:.4f}\tgz {:.4f}".format(gx,gy,gz))
            self._dbg(8, "FIFO Gyro Bias\tgbx {:.4f}\tgby {:.4f}\tgbz {:.4f}".format(gbx,gby,gbz))
            fcount -= DMP_Raw_Gyro_Bytes + DMP_Gyro_Bias_Bytes
        
        #Compass
        if (header & DMP_DO_Ctrl_1_Compass) != 0 :
            if (fcount < DMP_Compass_Bytes) :
                fcount = self.DMP_fifo_count()
            if (fcount < DMP_Compass_Bytes) :
                return
            self._data = self.read(0, ICM_FIFO_R_W, DMP_Compass_Bytes)
            mx,my,mz = unpack_from(">3h", self._data)
            mx *= self._mag_s
            my *= self._mag_s
            mz *= self._mag_s
            self._mag = mx, my, mz
            self._dbg(8, "FIFO Compass\t\tmx {:.4f}\tmy {:.4f}\tmz {:.4f}\tTime {:.0f}".format(mx,my,mz,self.newtime))
            fcount -= DMP_Compass_Bytes
        
        #ALS
        if (header & DMP_DO_Ctrl_1_ALS) != 0 :
            if (fcount < DMP_ALS_Bytes) :
                fcount = self.DMP_fifo_count()
            if (fcount < DMP_ALS_Bytes) :
                return
            self._als = self.read(0, ICM_FIFO_R_W, DMP_ALS_Bytes)
            #To do process
            self._dbg(8, "FIFO ALS not implemented", self._als)
            fcount -= DMP_ALS_Bytes
        
        #Quaternion 6
        if (header & DMP_DO_Ctrl_1_Quat6) != 0 :
            if (fcount < DMP_Quat6_Bytes) :
                fcount = self.DMP_fifo_count()
            if (fcount < DMP_Quat6_Bytes) :
                return
            self._data = self.read(0, ICM_FIFO_R_W, DMP_Quat6_Bytes)
            q1,q2,q3 = unpack_from(">3l", self._data)
            q1 /= 2**30  # The quaternion data is scaled by 2^30.
            q2 /= 2**30
            q3 /= 2**30
            self._quat6 = q1, q2, q3
            #To do process
            self._dbg(8, "FIFO Quaternion_6\tq1 {:.4f}\tq2 {:.4f}\tq3 {:.4f}".format(q1,q2,q3))
            fcount -= DMP_Quat6_Bytes
        
        #Quaternion 9
        if (header & DMP_DO_Ctrl_1_Quat9) != 0 :
            if (fcount < DMP_Quat9_Bytes) :
                fcount = self.DMP_fifo_count()
            if (fcount < DMP_Quat9_Bytes) :
                return
            self._data = self.read(0, ICM_FIFO_R_W, DMP_Quat9_Bytes)
            q1,q2,q3,acc = unpack_from(">3lh", self._data)
            q1 /= 2**30  # The quaternion data is scaled by 2^30.
            q2 /= 2**30
            q3 /= 2**30
            acc /= 2**16
            self._quat9 = q1, q2, q3
            self._quat9_ac = acc
            self._dbg(8, "FIFO Quaternion_9\tq1 {:.4f}\tq2 {:.4f}\tq3 {:.4f}\taccuracy {:.4f}".format(q1,q2,q3,acc))
            fcount -= DMP_Quat9_Bytes
            
        #Pedom Quaternion 6
        if (header & DMP_DO_Ctrl_1_Pedom_Quat6) != 0 :
            if (fcount < DMP_Pedom_Quat6_Bytes) :
                fcount = self.DMP_fifo_count()
            if (fcount < DMP_Pedom_Quat6_Bytes) :
                return
            self._data = self.read(0, ICM_FIFO_R_W, DMP_Pedom_Quat6_Bytes)
            q1,q2,q3 = unpack_from(">3h", self._data)
            q1 /= 2**30  # The quaternion data is scaled by 2^30.
            q2 /= 2**30
            q3 /= 2**30
            self._pquat6 = q1, q2, q3
            #To do process
            self._dbg(8, "FIFO PQuaternion_6\tq1 {:.4f}\tq2 {:.4f}\tq3 {:.4f}".format(q1,q2,q3))
            fcount -= DMP_Pedom_Quat6_Bytes
        
        #Geomag
        if (header & DMP_DO_Ctrl_1_Geomag) != 0 :
            if (fcount < DMP_Geomag_Bytes) :
                fcount = self.DMP_fifo_count()
            if (fcount < DMP_Geomag_Bytes) :
                return
            self._data = self.read(0, ICM_FIFO_R_W, DMP_Geomag_Bytes)
            q1,q2,q3,acc = unpack_from(">3lh", self._data)
            q1 /= 2**30
            q2 /= 2**30
            q3 /= 2**30
            acc /= 2**16
            self._gquat6 = q1, q2, q3
            self._gquat6_ac = acc
            self._dbg(8, "FIFO Geomag\tq1 {:.4f}\tq2 {:.4f}\tq3 {:.4f}\taccuracy {:.4f}".format(q1,q2,q3,acc))
            fcount -= DMP_Geomag_Bytes
            
        #Pressure
        if (header & DMP_DO_Ctrl_1_Pressure) != 0 :
            if (fcount < DMP_Pressure_Bytes) :
                fcount = self.DMP_fifo_count()
            if (fcount < DMP_Pressure_Bytes) :
                return
            self._data = self.read(0, ICM_FIFO_R_W, DMP_Pressure_Bytes)
            self._press = self._data[0] | (self._data[1] << 8) | (self._data[2] << 16)
            self._temp = self._data[3] | (self._data[4] << 8) | (self._data[5] << 16)
            self._dbg(8, "FIFO Press&Temp\tPress {:.1f}\tTemp {:.1f}".format(self._press,self._temp))
            fcount -= DMP_Pressure_Bytes
        
        #Gyro calibration 
        if (header & DMP_DO_Ctrl_1_Gyro_Calibr) != 0 :
            if (fcount < DMP_Gyro_Calibr_Bytes) :
                fcount = self.DMP_fifo_count()
            if (fcount < DMP_Gyro_Calibr_Bytes) :
                return
            self._data = self.read(0, ICM_FIFO_R_W, DMP_Gyro_Calibr_Bytes)
            gx_cal, gy_cal, gz_cal = unpack_from(">3l",self._data)
            gx_cal /= 2**30
            gy_cal /= 2**30
            gz_cal /= 2**30
            self._gyr_cal = gx_cal, gy_cal, gz_cal
            self._dbg(8, "FIFO Gyro Calibration\tgxc {:.4f}\tgyc {:.4f}\tgzc {:.4f}".format(gx_cal, gy_cal ,gz_cal))
            fcount -= DMP_Gyro_Calibr_Bytes
        
        #Compass calibration 
        if (header & DMP_DO_Ctrl_1_Compass_Calibr) != 0 :
            if (fcount < DMP_Compass_Calibr_Bytes) :
                fcount = self.DMP_fifo_count()
            if (fcount < DMP_Compass_Calibr_Bytes) :
                return
            self._data = self.read(0, ICM_FIFO_R_W, DMP_Compass_Calibr_Bytes)
            mx_cal, my_cal ,mz_cal = unpack_from(">3l", self._data)
            mx_cal /= 2**30
            my_cal /= 2**30
            mz_cal /= 2**30
            self._mag_cal = mx_cal, my_cal, mz_cal
            self._dbg(8, "FICO Compass Calibration\tmxc {:.4f}\tmyc {:.4f}\tmzc {:.4f}".format(mx_cal, my_cal ,mz_cal))
            fcount -= DMP_Compass_Calibr_Bytes
        
        #Steps
        if (header & DMP_DO_Ctrl_1_Step_Detector) != 0 :
            if (fcount < DMP_Step_Detector_Bytes) :
                fcount = self.DMP_fifo_count()
            if (fcount < DMP_Step_Detector_Bytes) :
                return
            self._data = self.read(0, ICM_FIFO_R_W, DMP_Step_Detector_Bytes)
            self._steps = self._data[0]<<24 | self._data[1]<<16 | self._data[2] << 8 | self._data[3] # MSB first
            #To do process
            self._dbg(8, "FIFO Step Detector", self._steps)
            fcount -= DMP_Step_Detector_Bytes
        
        #Check bit per bit header2
        
        #Accelerometer Accuracy
        if (header2 & DMP_DO_Ctrl_2_Accel_Accuracy) != 0 :
            if (fcount < DMP_Accel_Accuracy_Bytes) :
                fcount = self.DMP_fifo_count()
            if (fcount < DMP_Accel_Accuracy_Bytes) :
                return
            self._data = self.read(0, ICM_FIFO_R_W, DMP_Accel_Accuracy_Bytes)
            self._acc_ac = self._data[0]<<8 | self._data[1]
            self._dbg(8, "FIFO Accel Accuracy", self._acc_ac)
            fcount -= DMP_Accel_Accuracy_Bytes
        
        #Gyro Accuracy
        if (header2 & DMP_DO_Ctrl_2_Gyro_Accuracy) != 0 :
            if (fcount < DMP_Gyro_Accuracy_Bytes) :
                fcount = self.DMP_fifo_count()
            if (fcount < DMP_Gyro_Accuracy_Bytes) :
                return
            self._data = self.read(0, ICM_FIFO_R_W, DMP_Gyro_Accuracy_Bytes)
            self._gyr_ac = self._data[0]<<8 | self._data[1]
            self._dbg(8, "FIFO Gyro Accuracy", self._gyr_ac)
            fcount -= DMP_Gyro_Accuracy_Bytes
        
        #Compass Accuracy
        if (header2 & DMP_DO_Ctrl_2_Compass_Accuracy) != 0 :
            if (fcount < DMP_Compass_Accuracy_Bytes) :
                fcount = self.DMP_fifo_count()
            if (fcount < DMP_Compass_Accuracy_Bytes) :
                return
            self._data = self.read(0, ICM_FIFO_R_W, DMP_Compass_Accuracy_Bytes)
            self._mag_ac = self._data[0]<<8 | self._data[1]
            self._dbg(8, "FIFO Compass Accuracy", self._mag_ac)
            fcount -= DMP_Compass_Accuracy_Bytes
        
        #Fsynch Detection
        if (header2 & DMP_DO_Ctrl_2_Fsync) != 0 :
            if (fcount < DMP_Fsync_Detection_Bytes) :
                fcount = self.DMP_fifo_count()
            if (fcount < DMP_Fsync_Detection_Bytes) :
                return
            self._data = self.read(0, ICM_FIFO_R_W, DMP_Fsync_Detection_Bytes)
            self._fsynch = self._data[0]<<8 | self._data[1]
            #To do process
            self._dbg(8, "FIFO FSynch Detection", self._fsynch)
            fcount -= DMP_Fsync_Detection_Bytes
        
        #Pickup
        if (header2 & DMP_DO_Ctrl_2_Pickup) != 0 :
            if (fcount < DMP_Pickup_Bytes) :
                fcount = self.DMP_fifo_count()
            if (fcount < DMP_Pickup_Bytes) :
                return
            self._data = self.read(0, ICM_FIFO_R_W, DMP_Pickup_Bytes)
            self._pickup = self._data[0]<<8 | self._data[1]
            self._dbg(8, "FIFO Pickup not implemented", self._pickup)
            fcount -= DMP_Pickup_Bytes
        
        #Activity recog
        if (header2 & DMP_DO_Ctrl_2_Activity_Recog) != 0 :
            if (fcount < DMP_Activity_Recognition_Bytes) :
                fcount = self.DMP_fifo_count()
            if (fcount < DMP_Activity_Recognition_Bytes) :
                return
            self._data = self.read(0, ICM_FIFO_R_W, DMP_Activity_Recognition_Bytes)
            self._act_recog = unpack_from(">2bl", self._data)
            #To do process
            self._dbg(8, "FIFO Activity Recognition not implemented",self._act_recog)
            fcount -= DMP_Activity_Recognition_Bytes
        
        #Secondary On Off
        if (header2 & DMP_DO_Ctrl_2_Secondary_On_Off) != 0 :
            if (fcount < DMP_Secondary_On_Off_Bytes) :
                fcount = self.DMP_fifo_count()
            if (fcount < DMP_Secondary_On_Off_Bytes) :
                return
            self._data = self.read(0, ICM_FIFO_R_W, DMP_Secondary_On_Off_Bytes)
            self._sec_onoff = unpack_from(">h", self._data)
            #To do process
            self._dbg(8, "FIFO Secondary On-Off not implemented", self._sec_onoff)
            fcount -= DMP_Secondary_On_Off_Bytes
        
        #Finally process the footer
        if (fcount < DMP_Footer_Bytes) :
                fcount = self.DMP_fifo_count()
        if (fcount < DMP_Footer_Bytes) :
                return
        self._data = self.read(0, ICM_FIFO_R_W, DMP_Footer_Bytes)
        self._gyr_count = self._data[0]<<8 | self._data[1]
        self._dbg(4, "FIFO Gyro Count", self._gyr_count)
        fcount -= DMP_Footer_Bytes

    #Proceed the whole FIFO
    def DMP_fifo_proceed(self):
        while(self.DMP_fifo_count() > 0):
            self.DMP_fifo_read()
        return

    #Write to DMP register       
    def DMP_write(self, register, data):
        dmp_bank = register >> 8
        self.DMP_bank(dmp_bank)
        dmp_address = register & 0xFF
        self._dbg(2, "Writing DMP Bank", hex(dmp_bank),"Adress", hex(dmp_address),"Data",bytes(data))
        self.write(0, ICM_MEM_START_ADDR, dmp_address)
        self.write_bytes(0, ICM_MEM_R_W, bytes(data))
        
    #Read to DMP register
    def DMP_read(self, register, length = 1):
        dmp_bank = register >> 8
        self.DMP_bank(dmp_bank)
        dmp_address = register & 0xFF
        self._dbg(2, "Reading DMP Bank", hex(dmp_bank),"Adress", hex(dmp_address))
        self.write(0, ICM_MEM_START_ADDR, dmp_address)
        self._buffer = bytearray (length)
        self._bus.readfrom_mem_into(self._addr, ICM_MEM_R_W , self._buffer)
        return self._buffer

    #Set ODR for DMP
    def DMP_odr(self, dmp_ord_sensor, interval):
        #Sleep Out
        self.reg_config(0, ICM_PWR_MGMT_1, ICM_PWR_MGMT_1_SLEEP, enable=False)
        self.reg_config(0, ICM_PWR_MGMT_1, ICM_PWR_MGMT_1_LP, enable=False)
        
        odr_val = bytearray(2)
        odr_val[0] = interval >> 8
        odr_val[1] = interval & 0xFF
        odr_zero_count = bytearray([0x00, 0x00])
        self.DMP_write(dmp_ord_sensor, odr_val)
        #Write 0 to related counter (they are 32 bytes earlier in DMP mem)
        self.DMP_write(dmp_ord_sensor - 0x20, odr_zero_count)
        
        #Set LP again
        self.reg_config(0, ICM_PWR_MGMT_1, ICM_PWR_MGMT_1_LP, enable=True)


    #Activate specific DMP sensor 
    def DMP_enable_sensor(self, icm_sensor, enable=True):
        #Convert strind ICM_sensor to android_sensor number
        android_sensor = DMP_SENSORS_2_ANDROID[DMP_SENSORS[icm_sensor]]
        #Get Android Control Bits
        android_ctl_bits = ANDROID_SENSORS_CTRL_BITS[android_sensor]
        self._dbg(4, "Activation ICM sensor",icm_sensor, "- Android CTRL Bits", hex(android_ctl_bits))
        #Store Android sensor to objects _android_sensor_bitmask_0 and 1
        if android_sensor < 32 :
            _bitmask = 0x1 << android_sensor
            if enable :
                self._android_sensor_bitmask_0 |= _bitmask
            else :
                self._android_sensor_bitmask_0 &= ~_bitmask
        else :
            _bitmask = 0x1 << (android_sensor - 32)
            if enable :
                self._android_sensor_bitmask_1 |= _bitmask
            else :
                self._android_sensor_bitmask_1 &= ~_bitmask
        #self._dbg("Android Sensor bitmask", self._android_sensor_bitmask_0, self._android_sensor_bitmask_1)        
        #Reconstruc DATA_OUT_CTL1 from _android_sensor_bitmask_0 & 1
        _data_out_ctl = 0
        _data_out_ctl2 = 0
        _data_rdy_status = 0
        _inv_event_ctl = 0
        for x in range(32) :
            _bitmask = 0x1 << x
            if self._android_sensor_bitmask_0 & _bitmask > 0 :
                _data_out_ctl |= ANDROID_SENSORS_CTRL_BITS[x]
            if self._android_sensor_bitmask_1 & _bitmask > 0 :
                _data_out_ctl |= ANDROID_SENSORS_CTRL_BITS[x+32]
        #Reconstruct DATA_RDY_STATUS and INV_EVENT_CTRL
            #Case Acceleartion
            if ((self._android_sensor_bitmask_0 & _bitmask & INV_NEEDS_ACCEL_MASK0) | (self._android_sensor_bitmask_1 & _bitmask & INV_NEEDS_ACCEL_MASK1) > 0) :
                _data_rdy_status |= DMP_Data_Ready_Accel
                _inv_event_ctl |= DMP_Motion_Event_Control_Accel_Calibr
            #Case Gyro
            if ((self._android_sensor_bitmask_0 & _bitmask & INV_NEEDS_GYRO_MASK0) | (self._android_sensor_bitmask_1 & _bitmask & INV_NEEDS_GYRO_MASK1) > 0) :
                _data_rdy_status |= DMP_Data_Ready_Gyro
                _inv_event_ctl |= DMP_Motion_Event_Control_Gyro_Calibr
            #Case Compass
            if ((self._android_sensor_bitmask_0 & _bitmask & INV_NEEDS_COMPAS_MASK0) | (self._android_sensor_bitmask_1 & _bitmask & INV_NEEDS_COMPAS_MASK1) > 0) :
                _data_rdy_status |= DMP_Data_Ready_Secondary_Compass
                _inv_event_ctl |= DMP_Motion_Event_Control_Compass_Calibr
        #Reconstruc DATA_OUT_CTL2
        if ((_data_out_ctl & DMP_DO_Ctrl_1_Accel ) > 0) :
            _data_out_ctl2 |= DMP_DO_Ctrl_2_Accel_Accuracy
        if ((_data_out_ctl & DMP_DO_Ctrl_1_Gyro) | (_data_out_ctl & DMP_DO_Ctrl_1_Gyro_Calibr) > 0) :
            _data_out_ctl2 |= DMP_DO_Ctrl_2_Gyro_Accuracy
        if ((_data_out_ctl & DMP_DO_Ctrl_1_Compass) | (_data_out_ctl & DMP_DO_Ctrl_1_Compass_Calibr) | (_data_out_ctl & DMP_DO_Ctrl_1_Quat9) | (_data_out_ctl & DMP_DO_Ctrl_1_Geomag) > 0) :
            _data_out_ctl2 |= DMP_DO_Ctrl_2_Compass_Accuracy
        #Reconstruc INV_EVENT_CTRL
        if ((_data_out_ctl & DMP_DO_Ctrl_1_Quat9 ) > 0) :
            _inv_event_ctl |= DMP_Motion_Event_Control_9axis
        if ((_data_out_ctl & DMP_DO_Ctrl_1_Geomag) > 0) :
            _inv_event_ctl |= DMP_Motion_Event_Control_Geomag
        if ((_data_out_ctl & DMP_DO_Ctrl_1_Step_Detector) | (_data_out_ctl & DMP_DO_Ctrl_1_Step_Ind_0) | (_data_out_ctl & DMP_DO_Ctrl_1_Step_Ind_1) | (_data_out_ctl & DMP_DO_Ctrl_1_Step_Ind_2) > 0) :
            _inv_event_ctl |= DMP_Motion_Event_Control_Pedometer_Interrupt
            
        #Make sure the chip is not in Low Power mode nor in Sleep mode
        self.reg_config(0, ICM_PWR_MGMT_1, ICM_PWR_MGMT_1_SLEEP, enable=False)
        self.reg_config(0, ICM_PWR_MGMT_1, ICM_PWR_MGMT_1_LP, enable=False)
        
        #Write datas
        self._buffer = bytearray(2)
        #Write DATA_OUT_CTL1
        self._buffer[0] = _data_out_ctl >> 8
        self._buffer[1] = _data_out_ctl & 0xFF
        self._dbg(2, "DATA_OUT_CTRL1", hex(_data_out_ctl))
        self.DMP_write(DMP_DATA_OUT_CTL1, self._buffer)
        #Write DATA_OUT_CTL2
        self._buffer[0] = _data_out_ctl2 >> 8
        self._buffer[1] = _data_out_ctl2 & 0xFF
        self._dbg(2, "DATA_OUT_CTRL2", hex(_data_out_ctl2))
        self.DMP_write(DMP_DATA_OUT_CTL2, self._buffer)
        #Write DATA_RDY_STATUS
        self._buffer[0] = _data_rdy_status >> 8
        self._buffer[1] = _data_rdy_status & 0xFF
        self._dbg(2, "DATA_RDY_STATUS", hex(_data_rdy_status))
        self.DMP_write(DMP_DATA_RDY_STATUS, self._buffer)
        #Write MOTION_EVENT_CTL
        self._buffer[0] = _inv_event_ctl >> 8
        self._buffer[1] = _inv_event_ctl & 0xFF
        self._dbg(2, "INV_EVENT_CTRL", hex(_inv_event_ctl))
        self.DMP_write(DMP_DATA_MOTION_EVENT_CTRL, self._buffer)
        
        #Enable FIFO and DMP
        self.reg_config(0, ICM_USER_CTRL, ICM_USER_CTRL_DMP_EN | ICM_USER_CTRL_FIFO_EN, True)
        
        #Reset DMP
        self.reg_config(0, ICM_USER_CTRL, ICM_USER_CTRL_DMP_RST, True)
        
        #Reset FIFO
        self.reset_FIFO()
        
    def DMP_gyro_scaling(self, gyro_rate=56.25, gyro_range = 2000) :
                
        div = int((1125.0 / gyro_rate) - 1)
        gyro_scale = GYRO_SCALE_RANGE[gyro_range]

        #Read Timebase_correction_PLL register from bank 1
        pll = self.read(1, ICM_TIMEBASE_CORRECTION_PLL)
        self._gyro_sf_pll = pll
        
        MagicConstant = 264446880937391
        MagicConstantScale = 100000
        
        if (pll & 0x80) :
            result =  MagicConstant * (0x01 << gyro_scale) * (1 + div) / (1270 - (pll & 0x7F)) / MagicConstantScale
        else :
            result = MagicConstant * (0x01 << gyro_scale) * (1 + div) / (1270 + pll) / MagicConstantScale

        if (result > 0x7FFFFFFF) :
            self._gyro_sf = 0x7FFFFFFF
        else :
            self._gyro_sf = int(result)
            
        self._buffer = bytearray(4)
        self._buffer[0] = self._gyro_sf >> 24
        self._buffer[1] = self._gyro_sf >> 16
        self._buffer[2] = self._gyro_sf >> 8
        self._buffer[3] = self._gyro_sf & 0xFF
        self._dbg(2, "DMP_SET_GYRO_SF PLL", pll, "DMP_GYRO_SF", self._gyro_sf)
        self.DMP_write(DMP_GYRO_SF, self._buffer)
        
    def DMP_acc_scaling(self, acc_rate=56.25) :
                
        if (acc_rate == 225) :
            DMP_ACC_ONLY_GAIN_FACTOR = [0x00, 0xE8, 0xBA, 0x2E]
            DMP_ACCEL_ALPHA_VAR_FACTOR = [0x3D, 0x27, 0xD2, 0x7D]
            DMP_ACCEL_A_VAR_FACTOR = [0x02, 0xD8, 0x2D, 0x83]
        elif (acc_rate == 112.5) :
            DMP_ACC_ONLY_GAIN_FACTOR = [0x01, 0xD1, 0x74, 0x5D]
            DMP_ACCEL_ALPHA_VAR_FACTOR = [0x3A, 0x49, 0x24, 0x92]
            DMP_ACCEL_A_VAR_FACTOR = [0x02, 0xD8, 0x2D, 0x83]
        else : #general case will assume acc_rate = 56.25
            DMP_ACC_ONLY_GAIN_FACTOR = [0x03, 0xA4, 0x92, 0x49]
            DMP_ACCEL_ALPHA_VAR_FACTOR = [0x34, 0x92, 0x49, 0x25]
            DMP_ACCEL_A_VAR_FACTOR = [0x0B, 0x6D, 0xB6, 0xDB]    
                
        self.DMP_write(DMP_ACCEL_ONLY_GAIN, DMP_ACC_ONLY_GAIN_FACTOR)
        self.DMP_write(DMP_ACCEL_ALPHA_VAR, DMP_ACCEL_ALPHA_VAR_FACTOR)
        self.DMP_write(DMP_ACCEL_A_VAR, DMP_ACCEL_A_VAR_FACTOR)
            
        #Configure the Accel Cal Rate
        DMP_ACCEL_CAL_RATE_FACTOR = [0x00, 0x00]
        self.DMP_write(DMP_ACCEL_CAL_RATE, DMP_ACCEL_CAL_RATE_FACTOR)
        
        
#===== Below are all general function not linked directly with ICM20948 but usefull =========

    def Q_update_nomag(self, accel, gyro):    # 3-tuples (x, y, z) for accel, gyro
        self.newtime = ticks_us()
        ax, ay, az = accel                  # Units G (but later normalised)
        gx, gy, gz = (radians(x) for x in gyro) # Units deg/s
        q1, q2, q3, q4 = (self.q[x] for x in range(4))   # short name local variable for readability
        # Auxiliary variables to avoid repeated arithmetic
        _2q1 = 2 * q1
        _2q2 = 2 * q2
        _2q3 = 2 * q3
        _2q4 = 2 * q4
        _4q1 = 4 * q1
        _4q2 = 4 * q2
        _4q3 = 4 * q3
        _8q2 = 8 * q2
        _8q3 = 8 * q3
        q1q1 = q1 * q1
        q2q2 = q2 * q2
        q3q3 = q3 * q3
        q4q4 = q4 * q4

        # Normalise accelerometer measurement
        norm = sqrt(ax * ax + ay * ay + az * az)
        if (norm == 0):
            return # handle NaN
        norm = 1 / norm        # use reciprocal for division
        ax *= norm
        ay *= norm
        az *= norm

        # Gradient decent algorithm corrective step
        s1 = _4q1 * q3q3 + _2q3 * ax + _4q1 * q2q2 - _2q2 * ay
        s2 = _4q2 * q4q4 - _2q4 * ax + 4 * q1q1 * q2 - _2q1 * ay - _4q2 + _8q2 * q2q2 + _8q2 * q3q3 + _4q2 * az
        s3 = 4 * q1q1 * q3 + _2q1 * ax + _4q3 * q4q4 - _2q4 * ay - _4q3 + _8q3 * q2q2 + _8q3 * q3q3 + _4q3 * az
        s4 = 4 * q2q2 * q4 - _2q2 * ax + 4 * q3q3 * q4 - _2q3 * ay
        norm = 1 / sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4)    # normalise step magnitude
        s1 *= norm
        s2 *= norm
        s3 *= norm
        s4 *= norm

        # Compute rate of change of quaternion
        qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - self.beta * s1
        qDot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - self.beta * s2
        qDot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - self.beta * s3
        qDot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - self.beta * s4

        # Integrate to yield quaternion
        deltat = (self.newtime - self.lasttime) / 1000000
        self.lasttime = self.newtime
        q1 += qDot1 * deltat
        q2 += qDot2 * deltat
        q3 += qDot3 * deltat
        q4 += qDot4 * deltat
        norm = 1 / sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4)    # normalise quaternion
        self.q = q1 * norm, q2 * norm, q3 * norm, q4 * norm
        self.heading = 0
        self.pitch = degrees(-asin(2.0 * (self.q[1] * self.q[3] - self.q[0] * self.q[2])))
        self.roll = degrees(atan2(2.0 * (self.q[0] * self.q[1] + self.q[2] * self.q[3]),
            self.q[0] * self.q[0] - self.q[1] * self.q[1] - self.q[2] * self.q[2] + self.q[3] * self.q[3]))

    def Q_update_full(self, accel, gyro, mag):   						# 3-tuples (x, y, z) for accel, gyro and mag data
        self.newtime = ticks_us()
        mx, my, mz = (mag[x] - self._magbias[x] for x in range(3))  # Units irrelevant (normalised)
        ax, ay, az = accel  										# Units irrelevant (normalised)
        gx, gy, gz = (radians(x) for x in gyro) 					# Units deg/s
        q1, q2, q3, q4 = (self.q[x] for x in range(4))  			# short name local variable for readability
        # Auxiliary variables to avoid repeated arithmetic
        _2q1 = 2 * q1
        _2q2 = 2 * q2
        _2q3 = 2 * q3
        _2q4 = 2 * q4
        _2q1q3 = 2 * q1 * q3
        _2q3q4 = 2 * q3 * q4
        q1q1 = q1 * q1
        q1q2 = q1 * q2
        q1q3 = q1 * q3
        q1q4 = q1 * q4
        q2q2 = q2 * q2
        q2q3 = q2 * q3
        q2q4 = q2 * q4
        q3q3 = q3 * q3
        q3q4 = q3 * q4
        q4q4 = q4 * q4

        # Normalise accelerometer measurement
        norm = sqrt(ax * ax + ay * ay + az * az)
        if (norm == 0):
            return  		# handle NaN
        norm = 1 / norm 	# use reciprocal for division
        ax *= norm
        ay *= norm
        az *= norm

        # Normalise magnetometer measurement
        norm = sqrt(mx * mx + my * my + mz * mz)
        if (norm == 0):
            return  		# handle NaN
        norm = 1 / norm 	# use reciprocal for division
        mx *= norm
        my *= norm
        mz *= norm

        # Reference direction of Earth's magnetic field
        _2q1mx = 2 * q1 * mx
        _2q1my = 2 * q1 * my
        _2q1mz = 2 * q1 * mz
        _2q2mx = 2 * q2 * mx
        hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4
        hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4
        _2bx = sqrt(hx * hx + hy * hy)
        _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4
        _4bx = 2 * _2bx
        _4bz = 2 * _2bz

        # Gradient descent algorithm corrective step
        s1 = (-_2q3 * (2 * q2q4 - _2q1q3 - ax) + _2q2 * (2 * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5 - q3q3 - q4q4)
             + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
             + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz))

        s2 = (_2q4 * (2 * q2q4 - _2q1q3 - ax) + _2q1 * (2 * q1q2 + _2q3q4 - ay) - 4 * q2 * (1 - 2 * q2q2 - 2 * q3q3 - az)
             + _2bz * q4 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4)
             + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz))

        s3 = (-_2q1 * (2 * q2q4 - _2q1q3 - ax) + _2q4 * (2 * q1q2 + _2q3q4 - ay) - 4 * q3 * (1 - 2 * q2q2 - 2 * q3q3 - az)
             + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
             + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
             + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz))

        s4 = (_2q2 * (2 * q2q4 - _2q1q3 - ax) + _2q3 * (2 * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5 - q3q3 - q4q4)
              + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
              + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz))

        norm = 1 / sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4)    # normalise step magnitude
        s1 *= norm
        s2 *= norm
        s3 *= norm
        s4 *= norm

        # Compute rate of change of quaternion
        qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - self.beta * s1
        qDot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - self.beta * s2
        qDot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - self.beta * s3
        qDot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - self.beta * s4

        # Integrate to yield quaternion
        deltat = (self.newtime - self.lasttime) / 1000000
        self.lasttime = self.newtime
        q1 += qDot1 * deltat
        q2 += qDot2 * deltat
        q3 += qDot3 * deltat
        q4 += qDot4 * deltat
        norm = 1 / sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4)    # normalise quaternion
        self.q = q1 * norm, q2 * norm, q3 * norm, q4 * norm
        self.heading = self._north_angle + degrees(atan2(2.0 * (self.q[1] * self.q[2] + self.q[0] * self.q[3]),
            self.q[0] * self.q[0] + self.q[1] * self.q[1] - self.q[2] * self.q[2] - self.q[3] * self.q[3]))
        self.pitch = degrees(-asin(2.0 * (self.q[1] * self.q[3] - self.q[0] * self.q[2])))
        self.roll = degrees(atan2(2.0 * (self.q[0] * self.q[1] + self.q[2] * self.q[3]),
            self.q[0] * self.q[0] - self.q[1] * self.q[1] - self.q[2] * self.q[2] + self.q[3] * self.q[3]))

    #Read Euler Angle
    def euler(self,acc):
        #X (roll) axis
        zy_axis = degrees(atan2(acc[2], acc[1]))  #Calculate the angle with z and y 
        #_gyro_angle[0] = _angle[0] + gr*dt; //Use roll axis (X axis)
        #Y (pitch) axis
        zx_axis = degrees(atan2(acc[2], acc[0]))  #Calculate the angle with z and x 
        #_gyro_angle[1] = _angle[1] + gp*dt; //Use pitch axis (Y axis)
        #Z (yaw) axis
        xy_axis = degrees(atan2(acc[0], acc[1]))  #Calculate the angle with z and x
        return zy_axis, zx_axis, xy_axis

    #3-tupple representing the current Pan Tilt and Roll euler angle in degree
    #comming for 3 q1,q2,q3 quatenion
    def euler_q(self,q):
        
        qx = q[0]
        qy = q[1]
        qz = q[2]
        qx2 = qx * qx #qx2 stands for qx^2
        qy2 = qy * qy
        qz2 = qz * qz
        q02 = 1 - (qx2 + qy2 + qz2)
        q0 = sqrt #q0 reconstruction done

        t0 = +2.0 * (qz * q0 + qx * qy)
        t1 = +1.0 - 2.0 * (q02 + qx2)
        Roll = degrees(atan2(t0, t1))

        t2 = +2.0 * (qz * qx - qy * q[0])
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Tilt = degrees(asin(t2))

        t3 = +2.0 * (qz * qy + q0 * qx)
        t4 = +1.0 - 2.0 * (qx2 + qy2)
        Pan = degrees(atan2(t3, t4))

        return (Roll, Tilt, Pan)
