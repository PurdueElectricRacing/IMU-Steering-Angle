#ifndef LSM303D_H
#define LSM303D_H

#include "stm32l4xx_hal.h"
#include "main.h"

#define LSM303D_ADDR 0x3A

//DEFINE REGISTERS
#define LSM303D_TEMP_OUT_L 0x05
#define LSM303D_TEMP_OUT_H 0x06
//MAGNETOMETER REGISTERS
#define LSM303D_STATUS_M 0x07     //ZYXMOR/TempOR | YMOR | XMOR | ZYXMDA / TempDA | ZMDA | YMDA | XDMA
#define LSM303D_OUT_X_L_M 0x08    //X-Axis Mag data, represented in 16-bit 2's complement
#define LSM303D_OUT_X_H_M 0x09
#define LSM303D_OUT_Y_L_M 0x0A    //Y-Axis Mag data, represented in 16-bit 2's complement
#define LSM303D_OUT_Y_H_M 0x0B
#define LSM303D_OUT_Z_L_M 0x0C    //Z-Axis Mag data, represented in 16-bit 2's complement
#define LSM303D_OUT_Z_H_M 0x0D

#define LSM303D_WHO_AM_I 0x0F     //device identification register

#define LSM303D_OFFSET_X_L_M 0x16   //magnetic offset for X-Axis, default: 0
#define LSM303D_OFFSET_X_H_M 0x17   //Expressed in 16-bit 2's complement

#define LSM303D_OFFSET_Y_L_M 0x18   //Magnetic offset for Z-axis. Default value: 0
#define LSM303D_OFFSET_Y_H_M 0x19   //The value is expressed in 16-bit as 2s complement.

#define LSM303D_OFFSET_Z_L_M 0x1A   //Magnetic offset for Y-axis. Default value: 0.
#define LSM303D_OFFSET_Z_H_M 0x1B   //The value is expressed in 16-bit as 2s complement.

#define LSM303D_REFERENCE_X 0x1C    //Reference value for high-pass filter for X-axis acceleration data.
#define LSM303D_REFERENCE_Y 0x1D    //Reference value for high-pass filter for Y-axis acceleration data.
#define LSM303D_REFERENCE_Z 0x1E    //Reference value for high-pass filter for Z-axis acceleration data.
#define LSM303D_CTRL0 0x1F

//ACCELEROMETER REGISTERS
#define STATUS_A 0x27
//X-axis acceleration data. The value is expressed in 16-bit as 2s complement.
#define LSM303D_OUT_X_L_A 0x28
#define LSM303D_OUT_X_H_A 0x29
//X-axis acceleration data. The value is expressed in 16-bit as 2s complement.
#define LSM303D_OUT_Y_L_A 0x2A
#define LSM303D_OUT_Y_H_A 0x2B
//Z-axis acceleration data. The value is expressed in 16-bit as 2s complement.
#define LSM303D_OUT_Z_L_A 0x2C
#define LSM303D_OUT_Z_H_A 0x2D

/* CTRL1 enables axes and sets data rate/continuous update
[7:4] Accel Data rate; Default: 000 |
[3] Block data update; Default: 0 (continuous update)|
[2] AZEN; Default: 0 (Z DISABLED); 1: Z ENABLED |
[1] AYEN; Default: 0 (Y DISABLED); 1: Y ENABLED|
[0] AXEN; Default: 0 (X DISABLED); 1: X ENABLED*/

#define LSM303D_CTRL1 0x20

/*CTRL2 anti-alias bandwidth filter / full-scale selections
[7:6] Anti-alias filter bandwidth. default: 0 (REFER TO ACCEL_AA_t) |
[5:3] Full-scale selection. Default: 0 (refer to ACCEL_FS_t) |
[2] : 0 - MUST BE SET TO 0 FOR CORRECT OPERATIONS |
[1] Acceleration self-test enable. Default: 0 (disabled) |
[0] SPI interface selection. Default: 0 (disabled)*/

#define LSM303D_CTRL2 0x21

/* CTRL5 temp sensor / magnetic res selection / magnetic data rate
[7] Temperature Enable. Default: 0 (disabled) |
[6:5] Magnetic Resolution. Default:00(low resolution); 11:high Resolution |
[4:2] Magnetic Data Rate. Default: 110 |
[1] Latch interrupt request on INT2_SRC register. not needed |
[0] Latch interrupt request on INT_SRC register. Not needed*/

#define LSM303D_CTRL5 0x24

/*CTRL6 Magnetometer Full Scale
[7] 0 |
[6:5] Magnetic Full-scale. Default: 01.
[4:0] 0*/

#define LSM303D_CTRL6 0x25

/*CTRL7 Accelerometer HIgh pass filter / filtered data selection / Magnetic low-power / Mag sensor mode selected
[7:6] Accel High Pass. Default: 0x00. Refer to LSM303D.pdf |
[5] Filtered accel data selection. Default: 0 (internal filter bypassed); 1 (data from internal filter sent to output registers) |
[4] Temperature sensor only. Default: 0 - NOT NEEDED |
[3] 0 - LEAVE 0 |
[2] Magnetic low-power. Default: 0. Data rate configured by CTRL5 |
[1:0] Magnetic sensor mode selection. Default: 0x02. refer to MAG_SENS_SEL_t*/

#define LSM303D_CTRL7 0x26


//data rate for the accelerometer
//Used in CTRL1 for init ACCELEROMETER and CTRL5 for Magnetometer
typedef enum ACCEL_DATA_RATE_t
{
	ACCEL_DR_3_125_Hz  = 0x00,
	ACCEL_DR_6_25_Hz   = 0x01,
	ACCEL_DR_2_5_Hz    = 0x02,
	ACCEL_DR_25_Hz     = 0x03,
	ACCEL_DR_50_Hz     = 0x04,
	ACCEL_DR_100_Hz    = 0x05,
	ACCEL_DR_200_Hz    = 0x06,
	ACCEL_DR_400_Hz    = 0x07,
	ACCEL_DR_800_Hz    = 0x08,
	ACCEL_DR_1600_Hz   = 0x09,
}ACCEL_DATA_RATE;


/*Accelerometer anti-alias filter bandwidth
  Used in CTRL2*/
typedef enum ACCEL_AA_FILTER_t
{
	AA_773_Hz    = 0x00,
	AA_194_Hz    = 0x01,
	AA_362_Hz    = 0x02,
	AA_50_Hz     = 0x03,
} ACCEL_AA_FILTER;

/*Acceleration full-scale selections
  Used in CTRL2
  LSB is measured in mg/LSB*/
typedef enum ACCEL_FS_t
{
	ACCEL_2G  = 0x00,   //conversion = 0.061
	ACCEL_4G  = 0x01,   //conversion = 0.122		DEFAULT
	ACCEL_6G  = 0x02,   //conversion = 0.183
	ACCEL_8G  = 0x03,   //conversion = 0.244
	ACCEL_16G = 0x04,   //conversion = 0.732
}ACCEL_FS;

//struct containing all the axes' data values
typedef struct ACCEL_LSM303D
{
	int16_t x_accel;
	int16_t y_accel;
	int16_t z_accel;

	uint8_t accel_x_low;
	uint8_t accel_x_high;
	uint8_t accel_y_low;
	uint8_t accel_y_high;
	uint8_t accel_z_low;
	uint8_t accel_z_high;

	uint8_t broke;

	float conversion;

} ACCEL_LSM303D;

/*Magnetic full-scale selections
  Used in CTRL6
  LSB is measured in mgauss/LSB*/
typedef enum MAG_FS_t
{
	MAG_2_GAUSS   = 0x00,   //conversion = 0.080
	MAG_4_GAUSS   = 0x01,   //conversion = 0.160		DEFAULT
	MAG_8_GAUSS   = 0x02,   //conversion = 0.320
	MAG_12_GAUSS  = 0x03,   //conversion = 0.479
}MAG_FS;

//Magnetic Sensor Mode Select - default: 0x02
typedef enum MAG_SENS_SEL_t
{
    CONTINUOUS_CONV  = 0x00,
    SINGLE_CONV      = 0x01,
    POWER_DOWN       = 0x02,
} MAG_SENS_SEL;

//data rate for MAGNETOMETER
//used in CTRL5
typedef enum MAG_DATA_RATE_t
{
	MAG_DR_3_125_Hz  = 0x00,
	MAG_DR_6_25_Hz   = 0x01,
	MAG_DR_2_5_Hz    = 0x02,
	MAG_DR_25_Hz     = 0x03,
	MAG_DR_50_Hz     = 0x04,
	MAG_DR_100_Hz    = 0x05,		//available only when accelerometer is off or in LOW_ODR
}MAG_DATA_RATE;


//struct containing all the axes' data values and conversion
typedef struct MAG_LSM303D
{
	int16_t x_mag;
	int16_t y_mag;
	int16_t z_mag;

} MAG_LSM303D;

//MAG_LSM303D mag;			//global magnetometer struct

//HAL_StatusTypeDef accel_init( I2C_HandleTypeDef *hi2c, ACCEL_LSM303D *accel, ACCEL_DATA_RATE data_rate, ACCEL_AA_FILTER aa_filter, ACCEL_FS full_scale);
HAL_StatusTypeDef accel_init( I2C_HandleTypeDef *hi2c, ACCEL_DATA_RATE data_rate, ACCEL_AA_FILTER aa_filter, ACCEL_FS full_scale);


HAL_StatusTypeDef mag_init( I2C_HandleTypeDef *hi2c, MAG_DATA_RATE data_rate, int filter_enable, MAG_FS full_scale,  MAG_SENS_SEL sens_sel);

//HAL_StatusTypeDef read_accel(I2C_HandleTypeDef *hi2c, ACCEL_LSM303D *accel);
HAL_StatusTypeDef read_accel(I2C_HandleTypeDef *hi2c);

HAL_StatusTypeDef read_mag(I2C_HandleTypeDef *hi2c);

#endif
