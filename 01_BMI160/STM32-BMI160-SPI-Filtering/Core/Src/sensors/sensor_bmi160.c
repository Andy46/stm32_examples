/*
 * sensor_bmi160.c
 *
 *  Created on: Jul 14, 2024
 *      Author: agamb
 */

#include "sensor_bmi160.h"

#include <stdio.h>
#include <stdint.h>

#include <spi.h>
#include <gpio.h>

#ifdef __cplusplus
extern "C"
{
#endif

static struct bmi160_dev bmi160dev;

#define BMI160_CS_PORT GPIOB
#define BMI160_CS_PIN  GPIO_PIN_0

/* SPI callback functions */
int8_t bmi160spi_read_cb (uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t bmi160spi_write_cb (uint8_t dev_addr, uint8_t reg_addr, uint8_t *read_data, uint16_t len);
void bmi160spi_delay_ms_cb (uint32_t period);

//#undef DEBUG

int8_t sensor_bmi160_init(void)
{
	int8_t status = SENSOR_OK;

    // Set SPI chip select line to unselect
    HAL_GPIO_WritePin(BMI160_CS_PORT, BMI160_CS_PIN, GPIO_PIN_SET);

    /* Set interface address spi address */
    bmi160dev.intf = BMI160_SPI_INTF;
//    bmi160dev.id   = BMI160_DEV_ADDR;

    // Set communication callback functions for BMI160 library
    bmi160dev.read     = bmi160spi_read_cb;
    bmi160dev.write    = bmi160spi_write_cb;
    bmi160dev.delay_ms = bmi160spi_delay_ms_cb;

    // Initialize sensor using library
    status = bmi160_init(&bmi160dev);
    if (status == BMI160_OK)
    {
        printf("BMI160 initialized!\n");
        printf("Chip ID 0x%x\n", bmi160dev.chip_id);
    }
    else
    {
    	printf("Error initializing BMI160!\n");
        status = SENSOR_ERROR;
    }

    return sensor_bmi160_config();
}

int8_t sensor_bmi160_config(void)
{
	int8_t status = 0;

    /* Select the Output data rate, range of accelerometer sensor */
    bmi160dev.accel_cfg.odr   = BMI160_ACCEL_ODR_1600HZ;
    bmi160dev.accel_cfg.range = BMI160_ACCEL_RANGE_4G;
    bmi160dev.accel_cfg.bw    = BMI160_ACCEL_BW_NORMAL_AVG4;

    /* Select the power mode of accelerometer sensor */
    bmi160dev.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

    /* Select the Output data rate, range of Gyroscope sensor */
    bmi160dev.gyro_cfg.odr   = BMI160_GYRO_ODR_100HZ;
    bmi160dev.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
    bmi160dev.gyro_cfg.bw    = BMI160_GYRO_BW_OSR4_MODE;

    /* Select the power mode of Gyroscope sensor */
    bmi160dev.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

    /* Set the sensor configuration */
    status = bmi160_set_sens_conf(&bmi160dev);
    if (status == BMI160_OK)
    {
    	printf("BMI160 configured!\n");
    }
    else
    {
    	printf("Error configuring BMI160!");
        status = SENSOR_ERROR;
    }
    return status;
}

int8_t sensor_bmi160_read_accel(struct bmi160_sensor_data* accel_data)
{
	if (accel_data == NULL)
	{
		return SENSOR_ERROR;
	}

    bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_TIME_SEL), accel_data, NULL, &bmi160dev);
    return SENSOR_OK;
}

int8_t sensor_bmi160_read_gyro(struct bmi160_sensor_data* gyro_data)
{
	if (gyro_data == NULL)
	{
		return SENSOR_ERROR;
	}

    bmi160_get_sensor_data((BMI160_GYRO_SEL | BMI160_TIME_SEL), NULL, gyro_data, &bmi160dev);
    return SENSOR_OK;
}

int8_t sensor_bmi160_read_all(struct bmi160_sensor_data* accel_data, struct bmi160_sensor_data* gyro_data)
{
	if (accel_data == NULL)
	{
		return SENSOR_ERROR;
	}

    bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL |BMI160_TIME_SEL), accel_data, gyro_data, &bmi160dev);
    return SENSOR_OK;
}

// I2C Callbacks
int8_t bmi160spi_read_cb (uint8_t dev_addr, uint8_t reg_addr, uint8_t *read_data, uint16_t len)
{
#ifdef DEBUG
    printf("Reading from SPI!\n");
    printf("reg: 0x%x\n", reg_addr);
    printf("Len: %d\n", len);
#endif

    // Activate SPI chip select line
    HAL_GPIO_WritePin(BMI160_CS_PORT, BMI160_CS_PIN, GPIO_PIN_RESET);

    // Send command
    HAL_SPI_Transmit(&hspi1, &reg_addr, sizeof(reg_addr), 1000);

    // Receive data
    HAL_SPI_Receive(&hspi1, read_data, len, 1000);

    // Deactivate SPI chip select line
    HAL_GPIO_WritePin(BMI160_CS_PORT, BMI160_CS_PIN, GPIO_PIN_SET);

    return SENSOR_OK;
}

int8_t bmi160spi_write_cb (uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
#ifdef DEBUG
    printf("Writing to SPI!\n");
#endif

    // Compose transmission buffer (address + data)
    uint8_t buffer[len+1];
    buffer[0] = reg_addr;
    memcpy(&(buffer[1]), data, len);

    // Activate SPI chip select line
    HAL_GPIO_WritePin(BMI160_CS_PORT, BMI160_CS_PIN, GPIO_PIN_RESET);

    // Send buffer
    HAL_SPI_Transmit(&hspi1, buffer, sizeof(buffer), 1000);

    // Deactivate SPI chip select line
    HAL_GPIO_WritePin(BMI160_CS_PORT, BMI160_CS_PIN, GPIO_PIN_SET);

    return 0;
}

void bmi160spi_delay_ms_cb (uint32_t period_ms)
{
    HAL_Delay(period_ms);
}

#ifdef __cplusplus
}
#endif
