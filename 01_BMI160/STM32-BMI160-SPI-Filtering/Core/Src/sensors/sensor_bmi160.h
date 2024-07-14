/*
 * sensor_bmi160.h
 *
 *  Created on: Jul 14, 2024
 *      Author: agamb
 */

#ifndef SRC_SENSORS_SENSOR_BMI160_H_
#define SRC_SENSORS_SENSOR_BMI160_H_

#include <stdint.h>

#include <bmi160.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define SENSOR_OK     0
#define SENSOR_ERROR -1

int8_t sensor_bmi160_init(void);
int8_t sensor_bmi160_config(void);

int8_t sensor_bmi160_read_accel(struct bmi160_sensor_data* accel_data);
int8_t sensor_bmi160_read_gyro(struct bmi160_sensor_data* gyro_data);
int8_t sensor_bmi160_read_all(struct bmi160_sensor_data* accel_data, struct bmi160_sensor_data* gyro_data);

#ifdef __cplusplus
}
#endif

#endif /* SRC_SENSORS_SENSOR_BMI160_H_ */
