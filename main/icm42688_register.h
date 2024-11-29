#ifndef ICM42688_REGISTER_H
#define ICM42688_REGISTER_H


#define IMU_DEVICE_CONFIG 0x11
#define IMU_DRIVE_CONFIG 0x13
#define IMU_INT_CONFIG 0x14
#define IMU_FIFO_CONFIG 0x16
#define IMU_TEMP_DATA1 0x1D
#define IMU_TEMP_DATA0 0x1E
#define IMU_ACCEL_DATA_X1 0x1F
#define IMU_ACCEL_DATA_X0 0x20
#define IMU_ACCEL_DATA_Y1 0x21
#define IMU_ACCEL_DATA_Y0 0x22
#define IMU_ACCEL_DATA_Z1 0x23
#define IMU_ACCEL_DATA_Z0 0x24
#define IMU_GYRO_DATA_X1 0x25
#define IMU_GYRO_DATA_X0 0x26
#define IMU_GYRO_DATA_Y1 0x27
#define IMU_GYRO_DATA_Y0 0x28
#define IMU_GYRO_DATA_Z1 0x29
#define IMU_GYRO_DATA_Z0 0x2A
#define IMU_TMST_FSYNCH 0x2B
#define IMU_TMST_FSYNCL 0x2C
#define IMU_INT_STATUS 0x2D
#define IMU_FIFO_COUNTH 0x2E
#define IMU_FIFO_COUNTL 0x2F
#define IMU_FIFO_DATA 0x30
#define IMU_APEX_DATA0 0x31
#define IMU_APEX_DATA1 0x32
#define IMU_APEX_DATA2 0x33
#define IMU_APEX_DATA3 0x34
#define IMU_APEX_DATA4 0x35
#define IMU_APEX_DATA5 0x36
#define IMU_INT_STATUS2 0x37
#define IMU_INT_STATUS3 0x38
#define IMU_SIGNAL_PATH_RESET 0x4B
#define IMU_INTF_CONFIG0 0x4C
#define IMU_INTF_CONFIG1 0x4D
#define IMU_PWR_MGMT0 0x4E
#define IMU_GYRO_CONFIG0 0x4F
#define IMU_ACCEL_CONFIG0 0x50
#define IMU_GYRO_CONFIG1 0x51
#define IMU_GYRO_ACCEL_CONFIG0 0x52
#define IMU_ACCEL_CONFIG1 0x53
#define IMU_TMST_CONFIG 0x54
#define IMU_APEX_CONFIG0 0x56
#define IMU_SMD_CONFIG 0x57
#define IMU_FIFO_CONFIG1 0x5F
#define IMU_FIFO_CONFIG2 0x60
#define IMU_FIFO_CONFIG3 0x61
#define IMU_FSYNC_CONFIG 0x62
#define IMU_INT_CONFIG0 0x63
#define IMU_INT_CONFIG1 0x64
#define IMU_INT_SOURCE0 0x65
#define IMU_INT_SOURCE1 0x66
#define IMU_INT_SOURCE3 0x68
#define IMU_INT_SOURCE4 0x69
#define IMU_FIFO_LOST_PKT0 0x6C
#define IMU_FIFO_LOST_PKT1 0x6D
#define IMU_SELF_TEST_CONFIG 0x70
#define IMU_WHO_AM_I 0x75
#define IMU_REG_BANK_SEL 0x76
#define IMU_SENSOR_CONFIG0 0x03
#define IMU_GYRO_CONFIG_STATIC2 0x0B
#define IMU_GYRO_CONFIG_STATIC3 0x0C
#define IMU_GYRO_CONFIG_STATIC4 0x0D
#define IMU_GYRO_CONFIG_STATIC5 0x0E
#define IMU_GYRO_CONFIG_STATIC6 0x0F
#define IMU_GYRO_CONFIG_STATIC7 0x10
#define IMU_GYRO_CONFIG_STATIC8 0x11
#define IMU_GYRO_CONFIG_STATIC9 0x12
#define IMU_GYRO_CONFIG_STATIC10 0x13
#define IMU_XG_ST_DATA 0x5F
#define IMU_YG_ST_DATA 0x60
#define IMU_ZG_ST_DATA 0x61
#define IMU_TMSTVAL0 0x62
#define IMU_TMSTVAL1 0x63
#define IMU_TMSTVAL2 0x64
#define IMU_INTF_CONFIG4 0x7A
#define IMU_INTF_CONFIG5 0x7B
#define IMU_INTF_CONFIG6 0x7C
#define IMU_ACCEL_CONFIG_STATIC2 0x03
#define IMU_ACCEL_CONFIG_STATIC3 0x04
#define IMU_ACCEL_CONFIG_STATIC4 0x05
#define IMU_XA_ST_DATA 0x3B
#define IMU_YA_ST_DATA 0x3C
#define IMU_ZA_ST_DATA 0x3D
#define IMU_APEX_CONFIG1 0x40
#define IMU_APEX_CONFIG2 0x41
#define IMU_APEX_CONFIG3 0x42
#define IMU_APEX_CONFIG4 0x43
#define IMU_APEX_CONFIG5 0x44
#define IMU_APEX_CONFIG6 0x45
#define IMU_APEX_CONFIG7 0x46
#define IMU_APEX_CONFIG8 0x47
#define IMU_APEX_CONFIG9 0x48
#define IMU_ACCEL_WOM_X_THR 0x4A
#define IMU_ACCEL_WOM_Y_THR 0x4B
#define IMU_ACCEL_WOM_Z_THR 0x4C
#define IMU_INT_SOURCE6 0x4D
#define IMU_INT_SOURCE7 0x4E
#define IMU_INT_SOURCE8 0x4F
#define IMU_INT_SOURCE9 0x50
#define IMU_INT_SOURCE10 0x51
#define IMU_OFFSET_USER0 0x77
#define IMU_OFFSET_USER1 0x78
#define IMU_OFFSET_USER2 0x79
#define IMU_OFFSET_USER3 0x7A
#define IMU_OFFSET_USER4 0x7B
#define IMU_OFFSET_USER5 0x7C
#define IMU_OFFSET_USER6 0x7D
#define IMU_OFFSET_USER7 0x7E
#define IMU_OFFSET_USER8 0x7F

/*---------------GYRO FULL SCALE RANGE------------------*/
#define GYRO_DPS2000 0x00
#define GYRO_DPS1000 0x01
#define GYRO_DPS500 0x02
#define GYRO_DPS250 0x03
#define GYRO_DPS125 0x04
#define GYRO_DPS62_5 0x05
#define GYRO_DPS31_25 0x06
#define GYRO_DPS15_625 0x07

/*---------------ACCEL FULL SCALE RANGE------------------*/
#define ACCEL_G16 0x00
#define ACCEL_G8 0x01
#define ACCEL_G4 0x02
#define ACCEL_G2 0x03

/*---------------IMU OUTPUT DATA RATE------------------*/
#define ODR_32K 0x01
#define ODR_16K 0x02
#define ODR_8K 0x03
#define ODR_4K 0x04
#define ODR_2K 0x05
#define ODR_1K 0x06
#define ODR_200 0x07
#define ODR_100 0x08
#define ODR_50 0x09
#define ODR_25 0x0A
#define ODR_12_5 0x0B
#define ODR_6_25 0x0C
#define ODR_3_125 0x0D
#define ODR_1_5625 0x0E
#define ODR_500 0x0F

/*---------------NOTCH FILTER BANDWDITH------------------*/
#define NOTCH_BW_1449 0x00
#define NOTCH_BW_680 0x01
#define NOTCH_BW_329 0x02
#define NOTCH_BW_162 0x03
#define NOTCH_BW_80 0x04
#define NOTCH_BW_40 0x05
#define NOTCH_BW_20 0x06
#define NOTCH_BW_10 0x07

/*---------------UI ORDER------------------*/
#define UI_ORDER1 0b00
#define UI_ORDER2 0b01
#define UI_ORDER3 0x10

#endif