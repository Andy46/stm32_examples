/*
 * sensors_gy91.h
 *
 *  Created on: Jul 15, 2024
 *      Author: agamb
 */

#ifndef SRC_SENSORS_SENSORS_GY91_H_
#define SRC_SENSORS_SENSORS_GY91_H_

#include <stdint.h>

#include <MPU9250.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define SENSOR_OK     0
#define SENSOR_ERROR -1

int8_t sensor_gy91_init(void);

int8_t sensor_gy91_read_all(int16_t* accel_data, int16_t* mag_data, int16_t* gyro_data);

#ifdef __cplusplus
}
#endif

#endif /* SRC_SENSORS_SENSORS_GY91_H_ */
