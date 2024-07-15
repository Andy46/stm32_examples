/*
 * sensors_gy91.c
 *
 *  Created on: Jul 15, 2024
 *      Author: agamb
 */


#include "sensors_gy91.h"


int8_t sensor_gy91_init(void)
{
	int8_t status = SENSOR_OK;

	status = MPU9250_Init();
	if (status != 0)
	{
		return SENSOR_ERROR;
	}

	// Configuration
	MPU9250_SetSampleRateDivider(LP_ACCEL_ODR_250HZ);
	MPU9250_SetDLPFBandwidth(DLPF_BANDWIDTH_20HZ);
	MPU9250_SetGyroRange(GYRO_RANGE_2000DPS);
	MPU9250_SetAccelRange(ACCEL_RANGE_4G);

    return status;
}

int8_t sensor_gy91_read_all(int16_t* accel_data, int16_t* mag_data, int16_t* gyro_data)
{
	if (accel_data == NULL || mag_data == NULL || gyro_data == NULL)
	{
		return SENSOR_ERROR;
	}

	MPU9250_GetData(accel_data, mag_data, gyro_data);
}
