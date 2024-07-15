/*
 * calibration.c
 *
 *  Created on: Jul 15, 2024
 *      Author: agamb
 */

#include <stdio.h>

#include "../../sensors/sensor_bmi160.h"

#define SENSITIVITY 50

void calibration_main(void)
{

    struct bmi160_sensor_data gyro_data;

    int64_t cal_sum = 0;
    for (int i = 0; i < 100; i++)
    {
    	sensor_bmi160_read_gyro(&gyro_data);
    	cal_sum += gyro_data.z;
    }

    int16_t calZ = cal_sum / 100;

    printf("Calibration is: %d\n", calZ);

    uint32_t lastTime;
    uint32_t firstRun = 1;
    float x_deg = 0;
    while (1)
    {
        for (int i = 0; i < SENSITIVITY; i++)
        {
            sensor_bmi160_read_gyro(&gyro_data);
//            printf("GYRO:%-5d;%-5d;%-5d\n", gyro_data.x, gyro_data.y, gyro_data.z-calZ);
//            printf("Time: %lu\n", gyro_data.sensortime);
            if (firstRun == 1)
            {
                lastTime = gyro_data.sensortime;
                firstRun = 0;
            }
//            printf("Delta : %lu", gyro_data.sensortime -lastTime);
            if (abs(gyro_data.z - calZ) > 5)
            {
                x_deg += (gyro_data.z - calZ) * 0.061 * ((gyro_data.sensortime - lastTime) * 0.000039f);
            }
            lastTime = gyro_data.sensortime;
        }
        printf("%.2f\n", x_deg);
    }
}
