/*
 * daq.c

 *
 *  Created on: Sep 29, 2018
 *      Author: Chris
 */


#include "daq.h"

DAQ_Status_TypeDef daq_init(I2C_HandleTypeDef *hi2c, ADC_HandleTypeDef *hadc, CAN_HandleTypeDef *hcan, DAQ_TypeDef *daq)
{

	daq->hcan   = hcan;
	daq->hadc   = hadc;
	daq->hi2c 	= hi2c;

	//initialize the accelerometer, return accelerometer error if failed
	if (accel_init(daq->hi2c, ACCEL_DR_100_Hz, AA_50_Hz, ACCEL_4G) != HAL_OK)
	{
		return ACCEL_ERROR;
	}
	HAL_Delay(200);

	//initialize the gyroscope, return gyro error if failed
	if (gyro_init(daq->hi2c, GYRO_DR_100_Hz, FS_245_DPS, 0) != HAL_OK)
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


	//convert the raw 8bit values to 16 bit signed data for accelerometer
	accel.x_accel = ( (int16_t) accel.accel_x_high << 8 ) | ( (int16_t) accel.accel_x_low ) ;
	accel.y_accel = ( (int16_t) accel.accel_y_high << 8 ) | ( (int16_t) accel.accel_y_low ) ;
	accel.z_accel = ( (int16_t) accel.accel_z_high << 8 ) | ( (int16_t) accel.accel_z_low ) ;

	//convert the raw 8bit values to 16 bit signed data for gyroscope
	gyro.gyro_x_out = ( (int16_t) gyro.gyro_x_high << 8 ) | ( (int16_t) gyro.gyro_x_low );
	gyro.gyro_y_out = ( (int16_t) gyro.gyro_y_high << 8 ) | ( (int16_t) gyro.gyro_y_low );
	gyro.gyro_z_out = ( (int16_t) gyro.gyro_z_high << 8 ) | ( (int16_t) gyro.gyro_z_low );

	return DAQ_OK;
}

DAQ_Status_TypeDef daq_send_adc_data(DAQ_TypeDef *daq)
{

	uint32_t mailbox;
	uint8_t data[7];
	uint32_t temp_tick = uwTick;

	daq->tick = temp_tick;

	CAN_TxHeaderTypeDef header;
	header.StdId = ADC_ADDR;
	header.IDE= CAN_ID_STD;
	header.RTR = CAN_RTR_DATA;
	header.DLC = 7;
	header.TransmitGlobalTime = DISABLE;

	data[3] = (uint8_t) (daq->adc >> 24);
	data[2] = (uint8_t) (daq->adc >> 16);
	data[1] = (uint8_t) (daq->adc >> 8);
	data[0] = (uint8_t) (daq->adc);

	data[6] = (uint8_t) (temp_tick >> 16);
	data[5] = (uint8_t) (temp_tick >> 8);
	data[4] = (uint8_t) (temp_tick);

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
	header.DLC = 6;
	header.TransmitGlobalTime = DISABLE;


	data[0] = data_type;

	//determine which data values to get based on the data type
	switch (data_type)
	{
	case (ACCEL_X):
			data[1] = accel.accel_x_high;
			data[2] = accel.accel_x_low;
			break;
	case (ACCEL_Y):
			data[1] = accel.accel_y_high;
			data[2] = accel.accel_y_low;
			break;
	case (ACCEL_Z):
			data[1] = accel.accel_z_high;
			data[2] = accel.accel_z_low;
			break;
	case (GYRO_X):
			data[1] = gyro.gyro_x_high;
			data[2] = gyro.gyro_x_low;
			break;
	case (GYRO_Y):
			data[1] = gyro.gyro_y_high;
			data[2] = gyro.gyro_y_low;
			break;
	case (GYRO_Z):
			data[1] = gyro.gyro_z_high;
			data[2] = gyro.gyro_z_low;
			break;
	}

	//temp variable to "snapshot" the current tick
	uint32_t temp_tick = uwTick;
	daq->tick = temp_tick;

	//parse the tick into 3 bytes, big endian
	data [5] = (uint8_t) temp_tick;
	data [4] = (uint8_t) (temp_tick >> 8);
	data [3] = (uint8_t) (temp_tick >> 16);

	while (HAL_CAN_GetTxMailboxesFreeLevel(daq->hcan) == 0); // while mailboxes not free

	if (HAL_CAN_AddTxMessage(daq->hcan, &header, data, &mailbox) != HAL_OK)
	{
		return CAN_ERROR;
	}

	return DAQ_OK;

}
