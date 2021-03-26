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


#define IMU_ADDR 0x104;

#define ACCEL_DATARATE ACCEL_DR_100_Hz
#define ACCEL_RANGE		 ACCEL_4G
#define ACCEL_AA			 AA_50_Hz

#define GYRO_DATARATE GYRO_DR_100_Hz
#define GYRO_RANGE    FS_245_DPS

typedef struct
{
	I2C_HandleTypeDef *hi2c;
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
	CAN_ERROR,
	GENERIC_ERROR,
}DAQ_Status_TypeDef;

typedef enum
{
	ACCEL = 0,
	GYRO = 1,
	IMU_TYPE_MAX,
}IMU_Data_TypeDef;



DAQ_Status_TypeDef daq_init(I2C_HandleTypeDef *hi2c, CAN_HandleTypeDef *hcan, DAQ_TypeDef *daq);
DAQ_Status_TypeDef daq_read_data(DAQ_TypeDef *daq);
DAQ_Status_TypeDef daq_send_imu_data(DAQ_TypeDef *daq, IMU_Data_TypeDef data_type);
HAL_StatusTypeDef  daq_read_imu_reg(I2C_HandleTypeDef *hi2c, uint16_t dev_addr, uint8_t addr_high, uint8_t addr_low, uint8_t *output_high, uint8_t *output_low);

void initCompleteFlash();

#endif /* DAQ_H_ */
