/*
 * daq.h
 *
 *  Created on: Oct 05, 2018
 *      Author: Chris
 */

#ifndef DAQ_H_
#define DAQ_H_

#include "lsm303d.h"
#include "l3gd20h.h"
#include "stm32l4xx_hal.h"
#include "main.h"


#define IMU_ADDR 0x421;
#define ADC_ADDR 0x422;

typedef struct
{
	I2C_HandleTypeDef *hi2c;
	ADC_HandleTypeDef *hadc;
	CAN_HandleTypeDef *hcan;
	uint32_t adc;
	uint32_t tick;

}DAQ_TypeDef;

typedef enum
{
	DAQ_OK,
	ACCEL_ERROR,
	GYRO_ERROR,
	ADC_ERROR,
	CAN_ERROR
}DAQ_Status_TypeDef;

typedef enum
{
	ACCEL_X = 0,
	ACCEL_Y = 1,
	ACCEL_Z = 2,
	GYRO_X 	= 3,
	GYRO_Y 	= 4,
	GYRO_Z 	= 5,
}IMU_Data_TypeDef;



DAQ_Status_TypeDef daq_init(I2C_HandleTypeDef *hi2c, ADC_HandleTypeDef *hadc, CAN_HandleTypeDef *hcan, DAQ_TypeDef *daq);
DAQ_Status_TypeDef daq_read_data(DAQ_TypeDef *daq);
DAQ_Status_TypeDef daq_send_imu_data(DAQ_TypeDef *daq, IMU_Data_TypeDef data_type);
DAQ_Status_TypeDef daq_send_adc_data(DAQ_TypeDef *daq);

#endif /* DAQ_H_ */
