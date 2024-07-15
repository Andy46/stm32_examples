/*
 * complementary.c
 *
 *  Created on: Jul 15, 2024
 *      Author: agamb
 */

#include <stdio.h>
#include <math.h>

#include "main.h"

#include "../../sensors/sensor_bmi160.h"

#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

#define SENSITIVITY 50

void complementary_main(void)
{
    struct bmi160_sensor_data accel_data;
    struct bmi160_sensor_data gyro_data;

    int64_t cal_sum = 0;
    for (int i = 0; i < 100; i++) {
        sensor_bmi160_read_gyro(&gyro_data);
        cal_sum += gyro_data.z;
    }

    int16_t calZ = cal_sum / 100;

    printf("Calibration is: %d\n", calZ);

    uint32_t lastTime;
    uint32_t firstRun = 1;
    float pitch = 0.0, roll = 0.0, yaw = 0.0;

    // Complementary filter parameter
    const float alpha = 0.98;

    // Low-pass filter parameters
    float gyroXFiltered = 0, gyroYFiltered = 0, gyroZFiltered = 0;
    const float lpfAlpha = 0.5;  // Adjust this value to tune the low-pass filter

    while (1) {
        for (int i = 0; i < SENSITIVITY; i++) {
            sensor_bmi160_read_all(&accel_data, &gyro_data);

            if (firstRun == 1) {
                lastTime = gyro_data.sensortime;
                firstRun = 0;
            }

            uint32_t deltaTime = gyro_data.sensortime - lastTime;
            lastTime = gyro_data.sensortime;

            if (abs(gyro_data.x) < 5) gyro_data.x = 0;
            if (abs(gyro_data.y) < 5) gyro_data.y = 0;
            if (abs(gyro_data.z) < 5) gyro_data.z = 0;

            // Apply low-pass filter to gyro data
            gyroXFiltered = lpfAlpha * gyro_data.x + (1 - lpfAlpha) * gyroXFiltered;
            gyroYFiltered = lpfAlpha * gyro_data.y + (1 - lpfAlpha) * gyroYFiltered;
            gyroZFiltered = lpfAlpha * gyro_data.z + (1 - lpfAlpha) * gyroZFiltered;

            float gyroRateX = gyroXFiltered * 0.061; // Gyro rate in degrees per second
            float gyroRateY = gyroYFiltered * 0.061;
            float gyroRateZ = gyroZFiltered * 0.061;

            // Calculate the angles from the accelerometer
            float accelPitch = atan2(accel_data.y, accel_data.z) * 180.0 / M_PI;
            float accelRoll = atan2(-accel_data.x, sqrt(accel_data.y * accel_data.y + accel_data.z * accel_data.z)) * 180.0 / M_PI;

            // Apply the complementary filter
            pitch = alpha * (pitch + gyroRateX * deltaTime * 0.000039f) + (1 - alpha) * accelPitch;
            roll = alpha * (roll + gyroRateY * deltaTime * 0.000039f) + (1 - alpha) * accelRoll;

            // Integrate the yaw rate from the gyroscope
            yaw += gyroRateZ * deltaTime * 0.000039f;

            // Optional: Apply some correction or drift compensation to yaw
        }

        printf("Pitch: %.2f, Roll: %.2f, Yaw: %.2f\n", pitch, roll, yaw);
    }
}
