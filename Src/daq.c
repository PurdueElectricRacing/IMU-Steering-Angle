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

void initCompleteFlash()
{
  HAL_Delay(500);
  for (int i = 0; i < 6; i++)
  {
    HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    HAL_Delay(150);
  }
}


DAQ_Status_TypeDef daq_init(I2C_HandleTypeDef *hi2c, CAN_HandleTypeDef *hcan, DAQ_TypeDef *daq)
{
	daq->hcan   = hcan;
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

	//Start CAN, return CAN error if failed
	if (HAL_CAN_Start(daq->hcan))
	{
		return CAN_ERROR;
	}

	//all went well, return OK
	return DAQ_OK;
}


DAQ_Status_TypeDef daqReadData(DAQ_TypeDef *daq)
{
	//read the data from accelerometer, return error if failed
	if (readAccel(daq->hi2c) != HAL_OK)
	{
		return ACCEL_ERROR;
	}

	//read the data from gyro , return error if failed
	if (readGyro(daq->hi2c) != HAL_OK)
	{
		return GYRO_ERROR;
	}

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


DAQ_Status_TypeDef daqSendImuData(DAQ_TypeDef *daq, IMU_Data_TypeDef data_type)
{
	uint32_t mailbox;

	uint8_t data[8];
	data[6] = 0;
	data[7] = 0;

	CAN_TxHeaderTypeDef header;
	header.StdId = IMU_ADDR;
	header.IDE= CAN_ID_STD;
	header.RTR = CAN_RTR_DATA;
	header.DLC = 8;
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
		default:
		  return GENERIC_ERROR;
	}

	while (HAL_CAN_GetTxMailboxesFreeLevel(daq->hcan) == 0); // while mailboxes not free

	if (HAL_CAN_AddTxMessage(daq->hcan, &header, data, &mailbox) != HAL_OK)
	{
		return CAN_ERROR;
	}

	return DAQ_OK;

}
