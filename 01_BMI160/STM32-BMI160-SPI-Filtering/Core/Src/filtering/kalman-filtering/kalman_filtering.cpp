/*
 * kalman_filtering.cpp
 *
 *  Created on: Jul 14, 2024
 *      Author: agamb
 */

#include "main.h"
#include "../../sensors/sensor_bmi160.h"
#include <bmi160.h>
#include "gpio.h"

#include <stdio.h>

#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;

struct bmi160_sensor_data bmi160_accel;
struct bmi160_sensor_data bmi160_gyro;

#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105


float convertRawAcceleration(int16_t aRaw) {
  // since we are using 2 g range
  // -2 g maps to a raw value of -32768
  // +2 g maps to a raw value of 32767

  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float convertRawGyro(int16_t gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767

  float g = (gRaw * 250.0) / 32768.0;
  return g;
}

void filtering_main(void)
{

	sensor_bmi160_read_all(&bmi160_accel, &bmi160_gyro);

	double roll  = atan2(bmi160_accel.y, bmi160_accel.z) * RAD_TO_DEG;
	double pitch = atan(-bmi160_accel.x / sqrt(bmi160_accel.y * bmi160_accel.y + bmi160_accel.z * bmi160_accel.z)) * RAD_TO_DEG;
	timer = bmi160_accel.sensortime;

	kalmanX.setAngle(roll); // Set starting angle
	kalmanY.setAngle(pitch);
	gyroXangle = roll;
	gyroYangle = pitch;
	compAngleX = roll;
	compAngleY = pitch;

	while(1)
	{
		sensor_bmi160_read_all(&bmi160_accel, &bmi160_gyro);
	    double dt = (double)(bmi160_accel.sensortime - timer) / 1000000; // Calculate delta time
		timer = bmi160_accel.sensortime;

		double roll  = atan2(bmi160_accel.y, bmi160_accel.z) * RAD_TO_DEG;
		double pitch = atan(-bmi160_accel.x / sqrt(bmi160_accel.y * bmi160_accel.y + bmi160_accel.z * bmi160_accel.z)) * RAD_TO_DEG;

		double gyroXrate = gyroX / 131.0; // Convert to deg/s
		double gyroYrate = gyroY / 131.0; // Convert to deg/s

		  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
		    kalmanX.setAngle(roll);
		    compAngleX = roll;
		    kalAngleX = roll;
		    gyroXangle = roll;
		  } else
		    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

		  if (abs(kalAngleX) > 90)
		    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
		  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);


		  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
		  gyroYangle += gyroYrate * dt;
		  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
		  //gyroYangle += kalmanY.getRate() * dt;

		  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
		  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

		  // Reset the gyro angle when it has drifted too much
		  if (gyroXangle < -180 || gyroXangle > 180)
		    gyroXangle = kalAngleX;
		  if (gyroYangle < -180 || gyroYangle > 180)
		    gyroYangle = kalAngleY;

		  printf("%f\t%f\t%f\t%f\t\t%f\t%f\t%f\t%f\t\n", roll, gyroXangle, compAngleX, kalAngleX, pitch, gyroYangle, compAngleY, kalAngleY);


	}
	// Should not return
}
