#ifndef L3GD20H_H
#define L3GD20H_H

#include "stm32l4xx_hal.h"
#include "main.h"

#define L3GD20H_ADDR   		0xD6

//DEFINE REGISTERS
//Device identification Register
#define L3GD20H_WHO_AM_I 	0x0F

/*L3GD20H_CTRL1
Output data rate / bandwidth selection / Power down mode /  enable XEN, YEN, ZEN
[7:6] Data Rate. |
[5:4] bandwidth Selection |
[3] Power Mode. Default: 0 (Power Down); 1: (Normal Mode) |
[2] Z axis enable. Default: 1 (Enabled); 0: (Disabled) |
[1] Y axis enable. Default: 1 (Enabled); 0: (Disabled) |
[0] X axis enable. Default: 1 (Enabled); 0: (Disabled)*/

#define L3GD20H_CTRL1 		0x20

//NOT USED
#define L3GD20H_CTRL2 		0x21
#define L3GD20H_CTRL3 		0x22

/*L3GD20H_CTRL4
[7] Block data update. Default: 0 (continuous update); 1: (output registers not updated until MSB and LSB reading) |
[6] Big/little endian selection. Default: 0 (Data LSB @ lower address); 1: (Data MSB @ lower address) |
[5:4] Full Scale Selection. Default: 0x00 (245 DPS); 0x01: (500 DPS); 0x02: (2000 DPS) |
[3] Level Sensitive latch enable - NOT NEEDED |
[2:1] Self-Test enable. Default: 0x00 (normal mode); 0x01: (self-test 0(+)); 0x02: (unused); 0x03: (self-test 1(-)) |
[0] SPI Mode selection. Default: 0 (4-Wire) - NOT NEEDED*/

#define L3GD20H_CTRL4 		0x23

/*L3GD20H_CTRL5
[7] Reboot memory content. Default: 0 (normal mode); 1: (reboot memory content) |
[6] FIFO enable. Default: 0 (disabled) |
[5] - NOT NEEDED |
[4] High pass filter enable. Default: 0 (Disabled); 1: (Enabled) |
[3:0] - NOT NEEDED*/

#define L3GD20H_CTRL5 		0x24

/*L3GD20H_REFERENCE
[7:0] Digital High Pass filter reference. Default: 0*/

#define L3GD20H_REFERENCE 	0x25

//#define L3GD20H_STATUS 0x27
/*X-axis angular rate data. Expressed as 2's complement*/

#define L3GD20H_OUT_X_L 	0x28
#define L3GD20H_OUT_X_H 	0x29

/*Y-axis angular rate data. Expressed as 2's complement*/

#define L3GD20H_OUT_Y_L		0x2A
#define L3GD20H_OUT_Y_H 	0x2B

/*Z-axis angular rate data. Expressed as 2's complement*/

#define L3GD20H_OUT_Z_L 	0x2C
#define L3GD20H_OUT_Z_H 	0x2D

/*L3GD20H_LOW_ODR - PROBABLY UNNECESSARY
[7:6] MUST BE 0 |
[5] Data ready/interrupt2 pin active level - NOT NEEDED |
[4] I2C and SPI. Default: 0 (Both I2C and SPI Enabled); 1: (SPI Only) |
[3] Software reset. Default: 0 (Normal Mode); 1: (Reset Device) |
[2] MUST BE 0 |
[1:0] Low Output data rate. Default: 0 (Disabled); 1: (Enabled)*/

#define L3GD20H_LOW_ODR 	0x39

//Gyro output data rate values

typedef enum GYRO_DATA_RATE_t
{
	GYRO_DR_100_Hz  = 0x00,
	GYRO_DR_200_Hz  = 0x01,
	GYRO_DR_400_Hz  = 0x02,
	GYRO_DR_800_Hz  = 0x03,
}GYRO_DATA_RATE;

//Gyro sensitivity range

typedef enum GYRO_FULL_SCALE_t
{
	FS_245_DPS	  	= 0x00,   //Sensitivity: 8.75  mdps/LSB		DEFUALT
	FS_500_DPS   	= 0x01,   //Sensitivity: 17.50 mdps/LSB
	FS_2000_DPS  	= 0x02,   //Sensitivity: 70.00 mdps/LSB
}GYRO_FULL_SCALE;

typedef struct L3GD20H_GYRO
{
	int16_t gyro_x_out;
	int16_t gyro_y_out;
	int16_t gyro_z_out;

	uint8_t gyro_x_low;
	uint8_t gyro_x_high;
	uint8_t gyro_y_low;
	uint8_t gyro_y_high;
	uint8_t gyro_z_low;
	uint8_t gyro_z_high;

	uint8_t broke;

	float sensitivity;
}L3GD20H_GYRO;


/*Initialize the gyroscope
 * I2C_HandleTypeDef *hi2c 		-- I2C pointer
 * GYRO_DATA_RATE data rate 	-- the gyro data rate, range: [100hz, 800hz]
 * GYRO_MEASUREMENT_RANGE range -- the full-scale range of the gyroscope, range: [245 degrees/sec, 2000 degrees/sec]
 * int high_pass_filter			-- enable high-pass filter, default 0 (disabled)*/

//HAL_StatusTypeDef gyro_init(I2C_HandleTypeDef *hi2c, L3GD20H_GYRO *gyro, GYRO_DATA_RATE data_rate, GYRO_FULL_SCALE full_scale, int high_pass_filter);
HAL_StatusTypeDef gyro_init(I2C_HandleTypeDef *hi2c, GYRO_DATA_RATE data_rate, GYRO_FULL_SCALE full_scale, int high_pass_filter);

//HAL_StatusTypeDef read_gyro(I2C_HandleTypeDef *hi2c, L3GD20H_GYRO *gyro);
HAL_StatusTypeDef read_gyro(I2C_HandleTypeDef *hi2c);



#endif
