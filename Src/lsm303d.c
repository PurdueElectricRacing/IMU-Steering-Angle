/*
 * lsm303d.c
 *
 *  Created on: Sep 15, 2018
 *      Author: Chris
 */
#include "lsm303d.h"

extern ACCEL_LSM303D g_accel;		//global accelerometer struct

HAL_StatusTypeDef accel_init(I2C_HandleTypeDef *hi2c, ACCEL_DATA_RATE data_rate, ACCEL_AA_FILTER aa_filter, ACCEL_FS full_scale)
{
	HAL_StatusTypeDef status;

	if ((status = HAL_I2C_IsDeviceReady(hi2c, LSM303D_ADDR, 2, 100)) != HAL_OK)
	{
		g_accel.broke = 1;
		return status;
	}


	/*Set the conversion rate for the accelerometer readings based on the full-scale selection
	 * Specified in LSB/mG*/

	switch (full_scale)
	{
		case (ACCEL_2G):
				g_accel.conversion = 0.061;
				break;
		case(ACCEL_4G):
				g_accel.conversion = 0.122;
				break;
		case(ACCEL_6G):
				g_accel.conversion = 0.183;
				break;
		case(ACCEL_8G):
				g_accel.conversion = 0.244;
				break;
		case(ACCEL_16G):
				g_accel.conversion = 0.732;
				break;
		default:
			g_accel.conversion = 0.122;
			break;
	}
	g_accel.conversion = g_accel.conversion / 1000;


	/*INIT CTRL1
	 * Initialize CTRL1 to use selected data rate, continuous update, and enable XYZ axes
	 *
	 *CTRL1 enables axes and sets data rate/continuous update
	 *[7:4] Accel Data rate; Default: 000 |
	 *[3] Block data update; Default: 0 (continuous update)|
	 *[2] AZEN; Default: 0 (Z DISABLED); 1: Z ENABLED |
	 *[1] AYEN; Default: 0 (Y DISABLED); 1: Y ENABLED|
	 *[0] AXEN; Default: 0 (X DISABLED); 1: X ENABLED*/

	uint8_t init = data_rate<<4;
	init |= 0x07;
	if ((status = HAL_I2C_Mem_Write(hi2c, LSM303D_ADDR, LSM303D_CTRL1, 1, &init, 1, 10) != HAL_OK))
	{
		g_accel.broke = 1;
		return status;
	}

	/*INIT CTRL2
	 * Initialize CTRL2 to use the selected anti-alias filter and full-scale selection
	 *
	 * CTRL2 anti-alias bandwidth filter / full-scale selections
	 * [7:6] Anti-alias filter bandwidth. default: 0 (REFER TO ACCEL_AA_t) |
	 * [5:3] Full-scale selection. Default: 0 (refer to ACCEL_FS_t) |
	 * [2] : 0 - MUST BE SET TO 0 FOR CORRECT OPERATIONS |
	 * [1] Acceleration self-test enable. Default: 0 (disabled) |
	 * [0] SPI interface selection. Default: 0 (disabled)*/

	//reset init to 0
	init = 0x00;
	//or with the aa_filter
	init = aa_filter<<6;
	//or with full_scale
	init |= full_scale<<3;
	if ((status = HAL_I2C_Mem_Write(hi2c, LSM303D_ADDR, LSM303D_CTRL2, 1, &init, 1, 10) != HAL_OK))
	{
		g_accel.broke = 1;
		return status;
	}

	g_accel.broke = 0;
	return status;
}



///*static HAL_StatusTypeDef read_accel_reg	(I2C_HandleTypeDef *hi2c, uint8_t addr_high, uint8_t addr_low, int16_t *output)	-- reads the 2 data registers and puts the raw value into output
// *
// * NOTE: REGISTERS ARE LITTLE ENDIAN, THEREFORE THE HIGH REGISTER IS THE MSB AND LOW REGISTER IS LSB
// *
// *I2C_HandleTypeDef *hi2c		-- I2C pointer
// *uint8_t addr_high				-- address of the high output register
// *uint8_t addr_low				-- address of the low output register
// *int16_t *output				-- pointer to the raw data output value
// *retvalue: HAL_StatusTypeDef		--returns HAL_OK if no errors
// **/
//
//static HAL_StatusTypeDef read_accel_reg(I2C_HandleTypeDef *hi2c, uint8_t addr_high, uint8_t addr_low, uint8_t *out_high, uint8_t *out_low)
//{
//	HAL_StatusTypeDef status;
//
//	//read the low register for the axis, return status if error
//	if ((status = HAL_I2C_Mem_Read(hi2c, LSM303D_ADDR, addr_low,  1, out_low,  1, 10)) != HAL_OK)
//	{
//		return status;
//	}
//	//read the high register for the axis, return status if error
//	if ((status = HAL_I2C_Mem_Read(hi2c, LSM303D_ADDR, addr_high, 1, out_high, 1, 10)) != HAL_OK)
//	{
//		return status;
//	}
//
//	return HAL_OK;
//}

/*Read the acceleration data for each axis and put that value into the struct's storage values
 * I2C_HandleTypeDef *hi2c -- pointer to i2c HandleTypeDef
 * retval: HAL_StatusTypeDef*/

HAL_StatusTypeDef read_accel(I2C_HandleTypeDef *hi2c)
{
	HAL_StatusTypeDef status;

	//read the high register for the x axis, return status if error
	if ((status = daq_read_imu_reg(hi2c, LSM303D_ADDR, LSM303D_OUT_X_H_A, LSM303D_OUT_X_L_A, &g_accel.accel_x_high, &g_accel.accel_x_low)) != HAL_OK)
	{
		g_accel.broke = 1;
		return status;
	}

	//read the high register for the y axis, return status if error
	if ((status = daq_read_imu_reg(hi2c, LSM303D_ADDR, LSM303D_OUT_Y_H_A, LSM303D_OUT_Y_L_A, &g_accel.accel_y_high, &g_accel.accel_y_low)) != HAL_OK)
	{
		g_accel.broke = 1;
		return status;
	}

	//read the high register for the z axis, return status if error
	if ((status = daq_read_imu_reg(hi2c, LSM303D_ADDR, LSM303D_OUT_Z_H_A, LSM303D_OUT_Z_L_A, &g_accel.accel_z_high, &g_accel.accel_z_low)) != HAL_OK)
	{
		g_accel.broke = 1;
		return status;
	}

	return HAL_OK;
}






