#include "lsm6ds33.h"



/*
 * @brief: reads the 2 data registers and puts the raw value into output_high and output_low
 *
 * NOTE: REGISTERS ARE LITTLE ENDIAN, THEREFORE THE HIGH REGISTER IS THE MSB AND LOW REGISTER IS LSB
 *
 * @param I2C_HandleTypeDef *hi2c:  I2C pointer
 * @param uint32_t dev_addr: device address to get data from
 * @param uint8_t addr_high: address of the high output register
 * @param uint8_t addr_low:	 address of the low output register
 * @param AxisData_t * axis: output variable as a union
 *
 * @return HAL_StatusTypeDef: returns HAL_OK if no errors
 **/
static HAL_StatusTypeDef readImuReg(IMU_t * imu, uint16_t dev_addr,
                                 uint8_t addr_high, uint8_t addr_low, 
                                 AxisData_t * output)
{

	HAL_StatusTypeDef status;

	if ((status = HAL_I2C_Mem_Read(imu->i2c, dev_addr, addr_high, 1, 
                                 output->high, 1, 100)) != HAL_OK)
	{
		return status;
	}

	if ((status = HAL_I2C_Mem_Read(imu->i2c, dev_addr, addr_low, 1, 
                                 output->low, 1, 100)) != HAL_OK)
	{
		return status;
	}
	//OR the MSB with the data read from the register
	return HAL_OK;
}



HAL_StatusTypeDef gyroInit(IMU_t * imu, GYRO_DATA_RATE_t data_rate,
                           GYRO_FULL_SCALE_t full_scale, int high_pass_filter)
{
	HAL_StatusTypeDef status;
  ImuSensor * gyro = imu->gyro;
  //	if the device is not ready return error
	if ((status = HAL_I2C_IsDeviceReady(hi2c, LSM6DS33_ADDR, 2, 100)) != HAL_OK)
	{
		gyro->broke = 1;
		return status;
	}

	//set the sensitivity based on the full-scale selection
	switch (full_scale)
	{
		case (FS_245_DPS):
			gyro->sensitivity = 8.75;
			break;
		case (FS_500_DPS):
			gyro->sensitivity = 17.5;
			break;
    case (FS_1000_DPS):
			gyro->sensitivity = 35.0;
			break;
		case(FS_2000_DPS):
			gyro->sensitivity = 70.0;
			break;
	}
	gyro->sensitivity = gyro->sensitivity / 1000;


  uint8_t init = data_rate << 4 | full_scale << 2;
	/*Write to Gyro CTRL2
	 * Set bandwidth to max speed, enable XYZ axes, and enable Normal Mode*/

	if ((status = HAL_I2C_Mem_Write(hi2c, LSM6DS33_ADDR, CTRL2_G, 1, 
                                  &init, 1, 100)) != HAL_OK)
	{
		gyro->broke = 1;
		return status;
	}


	/*If the high_pass_filter flag is set, then write to CTRL7 to enable it;
	 * */

	if (high_pass_filter)
	{
    init = 0x01 << 6;
		if ((status = HAL_I2C_Mem_Write(hi2c, LSM6DS33_ADDR, CTRL7_G, 1, &init, 1, 100)) != HAL_OK)
			{
				gyro->broke = 1;
				return status;
			}
	}
	gyro->broke = 0;
	return HAL_OK;
}

/*Read the gyro data for each axis and put that value into the struct's storage values
 * I2C_HandleTypeDef *hi2c -- pointer to i2c HandleTypeDef
 * retval: HAL_StatusTypeDef*/

HAL_StatusTypeDef readGyro(ImuSensor * gyro, I2C_HandleTypeDef *hi2c);
{
	HAL_StatusTypeDef status;

	//read the X registers and write the raw values to the struct's x data point
	if ((status = readImuReg(hi2c, LSM6DS33_ADDR, OUTX_H_G, OUTX_L_G, gyro->x)) != HAL_OK)
	{
		gyro->broke = 1;
		return status;
	}

	//read the Y registers and write the raw values to the struct's y data point
	if ((status = readImuReg(hi2c, LSM6DS33_ADDR, OUTY_H_G, OUTY_L_G, gyro->y)) != HAL_OK)
	{
		gyro->broke = 1;
		return status;
	}

	//read the Z registers and write the raw values to the struct's z data point
	if ((status = readImuReg(hi2c, LSM6DS33_ADDR, OUTZ_H_G, OUTZ_L_G, gyro->z)) != HAL_OK)
	{
		gyro->broke = 1;
		return status;
	}

	return HAL_OK;
}





HAL_StatusTypeDef accelInit(IMU_T * imu, ACCEL_DATA_RATE_t data_rate,
                            ACCEL_AA_FILTER_t aa_filter, ACCEL_FS_t full_scale)
{
	HAL_StatusTypeDef status;
  ImuSensor * accel = imu->accel;

	if ((status = HAL_I2C_IsDeviceReady(hi2c, LSM6DS33_ADDR, 2, 100)) != HAL_OK)
	{
		accel->broke = 1;
		return status;
	}


	/*Set the conversion rate for the accelerometer readings based on the full-scale selection
	 * Specified in LSB/mG*/

	switch (full_scale)
	{
	  case(ACCEL_2G):
				accel->conversion = 0.061;
				break;
		case(ACCEL_4G):
				accel->conversion = 0.122;
				break;
		case(ACCEL_8G):
				accel->conversion = 0.244;
				break;
		case(ACCEL_16G):
				accel->conversion = 0.488;
				break;
		default:
			accel->conversion = 0.122;
			break;
	}
	accel->conversion = accel->conversion / 1000;



  // init CTRL 1
	uint8_t init = data_rate << 4 | full_scale << 2;
	init |= 0x07;
	if ((status = HAL_I2C_Mem_Write(hi2c, LSM6DS33_ADDR, CTRL1_XL, 1, &init, 1, 10) != HAL_OK))
	{
		accel->broke = 1;
		return status;
	}

	accel->broke = 0;
	return status;
}



/*Read the acceleration data for each axis and put that value into the struct's storage values
 * I2C_HandleTypeDef *hi2c -- pointer to i2c HandleTypeDef
 * retval: HAL_StatusTypeDef*/

HAL_StatusTypeDef readAccel(ImuSensor * accel, I2C_HandleTypeDef *hi2c)
{
	HAL_StatusTypeDef status;

	//read the high register for the x axis, return status if error
	if ((status = readImuReg(hi2c, LSM6DS33_ADDR, OUTX_H_XL, OUTX_L_XL, accel->x)) != HAL_OK)
	{
		accel->broke = 1;
		return status;
	}

	//read the high register for the y axis, return status if error
	if ((status = readImuReg(hi2c, LSM6DS33_ADDR, OUTY_H_XL, OUTY_L_XL, accel->y) != HAL_OK)
	{
		accel->broke = 1;
		return status;
	}

	//read the high register for the z axis, return status if error
	if ((status = readImuReg(hi2c, LSM6DS33_ADDR, OUTZ_H_XL, OUTZ_L_XL, accel->z) != HAL_OK)
	{
		accel->broke = 1;
		return status;
	}

	return HAL_OK;
}



