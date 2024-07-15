/*
 * calibration.c
 *
 *  Created on: Jul 15, 2024
 *      Author: agamb
 */

#include <stdio.h>
#include <math.h>

#include "../../sensors/sensors_gy91.h"

#define SENSITIVITY 50

void calibration_main(void)
{
	int16_t accel_data[3];
	int16_t mag_data[3];
	int16_t gyro_data[3];

    int64_t cal_sum = 0;
    for (int i = 0; i < 1000; i++)
    {
    	sensor_gy91_read_all(accel_data, mag_data, gyro_data);
    	cal_sum += gyro_data[2];
    }

    int16_t calZ = cal_sum / 1000;

    printf("Calibration is: %d\n", calZ);

    uint32_t lastTime;
    uint32_t firstRun = 1;
    float x_deg = 0;
    uint32_t sensortime = 0;
    while (1)
    {
        for (int i = 0; i < SENSITIVITY; i++)
        {
        	sensor_gy91_read_all(accel_data, mag_data, gyro_data);
        	sensortime = HAL_GetTick();
//            printf("GYRO:%-5d;%-5d;%-5d\n", gyro_data.x, gyro_data.y, gyro_data.z-calZ);
//            printf("Time: %lu\n", gyro_data.sensortime);
            if (firstRun == 1)
            {
                lastTime = sensortime;
                firstRun = 0;
            }
//            printf("Delta : %lu", gyro_data.sensortime -lastTime);
            if (abs(gyro_data[2] - calZ) > 20)
            {
                x_deg += (gyro_data[2] - calZ) * 0.061 * ((sensortime - lastTime) * 0.000039f);
            }
            lastTime = sensortime;
        }
        printf("%.2f\n", x_deg);
    }
}
