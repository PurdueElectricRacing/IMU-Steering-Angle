/*
 * daq.c

 *
 *  Created on: Sep 29, 2018
 *      Author: Chris
 *
 *  Edited on: Jan 25, 209
 *  		Editor: Chris
 */


#include "daq.h"

extern L3GD20H_GYRO g_gyro;
extern ACCEL_LSM303D g_accel;

DAQ_Status_TypeDef daq_init(I2C_HandleTypeDef *hi2c, ADC_HandleTypeDef *hadc, CAN_HandleTypeDef *hcan, DAQ_TypeDef *daq)
{

	daq->hcan   = hcan;
	daq->hadc   = hadc;
	daq->hi2c 	= hi2c;

	//initialize the accelerometer, return accelerometer error if failed
	if (accel_init(daq->hi2c, ACCEL_DATARATE, ACCEL_AA, ACCEL_RANGE) != HAL_OK)
	{
		return ACCEL_ERROR;
	}
	HAL_Delay(200);

	//initialize the gyroscope, return gyro error if failed
	if (gyro_init(daq->hi2c, GYRO_DATARATE, GYRO_RANGE, 0) != HAL_OK)
	{
		return GYRO_ERROR;
	}
	HAL_Delay(200);

	//Start ADC, return ADC error if failed
	if (HAL_ADC_Start(daq->hadc) != HAL_OK)
	{
		return ADC_ERROR;
	}

	//Start CAN, return CAN error if failed
	if (HAL_CAN_Start(daq->hcan))
	{
		return CAN_ERROR;
	}

	//all went well, return OK
	return DAQ_OK;
}

/*
 * @brief: reads the 2 data registers and puts the raw value into output_high and output_low
 *
 * NOTE: REGISTERS ARE LITTLE ENDIAN, THEREFORE THE HIGH REGISTER IS THE MSB AND LOW REGISTER IS LSB
 *
 * @param I2C_HandleTypeDef *hi2c:  I2C pointer
 * @param uint32_t dev_addr: device address to get data from
 * @param uint8_t addr_high: address of the high output register
 * @param uint8_t addr_low:	 address of the low output register
 * @param int16_t *output:   pointer to the raw data output value
 *
 * @return HAL_StatusTypeDef: returns HAL_OK if no errors
 **/
HAL_StatusTypeDef  daq_read_imu_reg(I2C_HandleTypeDef *hi2c, uint32_t dev_addr, uint8_t addr_high, uint8_t addr_low, uint8_t *output_high, uint8_t *output_low)
{

	HAL_StatusTypeDef status;
	//if there was an error reading the high storage address, set output to -10000 and return the error

	if ((status = HAL_I2C_Mem_Read(hi2c, dev_addr, addr_high, 1, output_high, 1, 100)) != HAL_OK)
	{
		return status;
	}
	//left shift the data and OR with the data from the high register

	//if there was an error reading the low storage address, set output to -10000 and return the error

	if ((status = HAL_I2C_Mem_Read(hi2c, dev_addr, addr_low, 1, output_low, 1, 100)) != HAL_OK)
	{
		return status;
	}
	//OR the MSB with the data read from the register
	return HAL_OK;
}

DAQ_Status_TypeDef daq_read_data(DAQ_TypeDef *daq)
{
	//read the data from accelerometer, return error if failed
	if (read_accel(daq->hi2c) != HAL_OK)
	{
		return ACCEL_ERROR;
	}

	//read the data from gyro , return error if failed
	if (read_gyro(daq->hi2c) != HAL_OK)
	{
		return GYRO_ERROR;
	}

	//read the data from ADC, return error if failed
	if (HAL_ADC_PollForConversion(daq->hadc, 1000) != HAL_OK)
	{
		return ADC_ERROR;
	}
	daq->adc = HAL_ADC_GetValue(daq->hadc);

	// changing output orientation to match the orientation which it is when in the car

	//convert the raw 8bit values to 16 bit signed data for accelerometer
	if (!g_accel.broke)
	{
		g_accel.z_accel = ( (int16_t) g_accel.accel_x_high << 8 ) | ( (int16_t) g_accel.accel_x_low );
		g_accel.x_accel = ( (int16_t) g_accel.accel_y_high << 8 ) | ( (int16_t) g_accel.accel_y_low );
		g_accel.y_accel = ( (int16_t) g_accel.accel_z_high << 8 ) | ( (int16_t) g_accel.accel_z_low );
	}


	//convert the raw 8bit values to 16 bit signed data for gyroscope
	if (!g_gyro.broke)
	{
		g_gyro.gyro_z_out = ( (int16_t) g_gyro.gyro_x_high << 8 ) | ( (int16_t) g_gyro.gyro_x_low );
		g_gyro.gyro_x_out = ( (int16_t) g_gyro.gyro_y_high << 8 ) | ( (int16_t) g_gyro.gyro_y_low );
		g_gyro.gyro_y_out = ( (int16_t) g_gyro.gyro_z_high << 8 ) | ( (int16_t) g_gyro.gyro_z_low );
	}

	return DAQ_OK;
}

DAQ_Status_TypeDef daq_send_adc_data(DAQ_TypeDef *daq)
{

	uint32_t mailbox;
	uint8_t data[4];
	uint32_t temp_tick = uwTick;

	daq->tick = temp_tick;

	CAN_TxHeaderTypeDef header;
	header.StdId = ADC_ADDR;
	header.IDE= CAN_ID_STD;
	header.RTR = CAN_RTR_DATA;
	header.DLC = 4;
	header.TransmitGlobalTime = DISABLE;

	data[3] = (uint8_t) (daq->adc >> 24);
	data[2] = (uint8_t) (daq->adc >> 16);
	data[1] = (uint8_t) (daq->adc >> 8);
	data[0] = (uint8_t) (daq->adc);

	while (HAL_CAN_GetTxMailboxesFreeLevel(daq->hcan) == 0); // while mailboxes not free

	if ( HAL_CAN_AddTxMessage(daq->hcan, &header, data, &mailbox) != HAL_OK)
	{
		return CAN_ERROR;
	}

	return DAQ_OK;
}

DAQ_Status_TypeDef daq_send_imu_data(DAQ_TypeDef *daq, IMU_Data_TypeDef data_type)
{
	uint32_t mailbox;

	uint8_t data[8];
	data[6] = 0;
	data[7] = 0;

	CAN_TxHeaderTypeDef header;
	header.StdId = IMU_ADDR;
	header.IDE= CAN_ID_STD;
	header.RTR = CAN_RTR_DATA;
	header.DLC = 7;
	header.TransmitGlobalTime = DISABLE;


	data[0] = data_type;

	//determine which data values to get based on the data type
	switch (data_type) {
		case (ACCEL):
				if (!g_accel.broke)
				{
					data[1] = g_accel.accel_x_high;
					data[2] = g_accel.accel_x_low;
					data[3] = g_accel.accel_y_high;
					data[4] = g_accel.accel_y_low;
					data[5] = g_accel.accel_z_high;
					data[6] = g_accel.accel_z_low;
					break;
				}
				else
				{
					return ACCEL_ERROR;
				}
		case (GYRO):
				if (!g_gyro.broke)
				{
					data[1] = g_gyro.gyro_x_high;
					data[2] = g_gyro.gyro_x_low;
					data[3] = g_gyro.gyro_y_high;
					data[4] = g_gyro.gyro_y_low;
					data[5] = g_gyro.gyro_z_high;
					data[6] = g_gyro.gyro_z_low;
					break;
				}
				else
				{
					return GYRO_ERROR;
				}
	}

	while (HAL_CAN_GetTxMailboxesFreeLevel(daq->hcan) == 0); // while mailboxes not free

	if (HAL_CAN_AddTxMessage(daq->hcan, &header, data, &mailbox) != HAL_OK)
	{
		return CAN_ERROR;
	}

	return DAQ_OK;

}
