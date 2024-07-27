/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_spi.h"
#include "stm32f1xx_ll_spi.h"
#include "spi.h"
#include "usart.h"

#include <stdio.h>

#include "sensors/pmw3360.h"
#include "sensors/srom_3360_0x04.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

static void pmw3360_init(const uint8_t dpi);
static void angle_init(const uint8_t angle);
static inline void spi_write(const unsigned char addr, const unsigned char data);
static inline const unsigned char spi_read(const unsigned char addr);
unsigned char SPI_Send(unsigned char Txdata);
static uint8_t spi_transmit_receive(uint8_t data_in, uint8_t *data_out);
void HAL_Delay_us(__IO uint32_t delay_us);

#define SS_LOW HAL_GPIO_WritePin(PMW_CS_GPIO_Port, PMW_CS_Pin, GPIO_PIN_RESET);
#define SS_HIGH HAL_GPIO_WritePin(PMW_CS_GPIO_Port, PMW_CS_Pin, GPIO_PIN_SET);


#define CPI_Page_Address 0x08007000
#define Polling_Skip_Page_Address 0x08007400
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static unsigned char old_profile;

// use this instead of bitshifts or LSB/MSB macros.
union motion_data
{
	int16_t sum;
	struct { uint8_t low, high; };
};
union motion_data x, y;
int8_t whl;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

//SPI_HandleTypeDef hspi1; // for HAL SPI lib

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void sensor_config(void) // Sensor Power-up configure
{
  // dpi settings CPI/CPI
	uint8_t dpi;

//	old_profile = ((*(__IO uint16_t*)(0x08007000)) < 0xFF) ? *(__IO uint16_t*)(0x08007000) : 0x01;
//	if (old_profile == 0x00)
		dpi = 0x03;// dpi = (1 + i) * 100
//	if (old_profile == 0x01)
//		dpi = 0x07;// dpi = (1 + i) * 100
//	if (old_profile == 0x02)
//		dpi = 0x0f;// dpi = (1 + i) * 100

  // Init 3360
  pmw3360_init(dpi);

  // Angle snapping settings // turn off thx.
  uint8_t angle_index = 0;
  uint8_t angles[] = {0x00, 0x80}; // Off, On
  // Init angle snapping
  angle_init(angles[angle_index]);
}

void burst_read(void)	// read sensor move
{
		union motion_data delta_x, delta_y;
		SS_LOW;// step 1: low NCS/SS/CS Signal
		SPI_Send(0x50); // step 2: send Motion regsiter addr: 0x50.
    HAL_Delay_us(35); //step 4: wait Tsard_motbr.

		// at here send any vule to sensor reg
		(void) SPI_Send(0x00); // motion, not used
		(void) SPI_Send(0x00); // observation, not used
//    int motion = (SPI_Send(0x00); & 0x80) > 0;
//    int surface = (SPI_Send(0x00); & 0x08) > 0;   // 0 if on surface / 1 if off surface

		delta_x.low=SPI_Send(0x00);
		delta_x.high=SPI_Send(0x00);
		delta_y.low=SPI_Send(0x00);
		delta_y.high=SPI_Send(0x00);

//    int squal = SPI_Send(0x00);
		SS_HIGH;
    HAL_Delay_us(1); // pulling high at lest 500ns

		x.sum += delta_x.sum;
		y.sum += delta_y.sum;

		printf("X: %5d.%5d - Y: %5d\n", delta_x.high, delta_x.low, delta_y);
}

uint8_t read_byte(uint8_t reg)
{
	// Enable Chip select
	SS_LOW;

	uint8_t wr_buffer = reg & 0x7f; // Cmd to send
	uint8_t rd_buffer;

	HAL_SPI_Transmit(&hspi1, &wr_buffer, 1, 1000);

	HAL_Delay_us(200);

	HAL_SPI_Receive(&hspi1, &rd_buffer, 1, 1000);

	HAL_Delay_us(120);

	// Disable Chip select
	SS_HIGH;
//	HAL_Delay(200);

	//	printf("Reading reg: 0x%02x - 0x%02x\n", reg, rd_buffer);
	return rd_buffer;
}

void write_byte(uint8_t reg, uint8_t data)
{
//	printf("Writing reg: 0x%02x - 0x%02x\n", reg, data);

	// Enable Chip select
	SS_LOW;

	uint8_t wr_buffer = reg | 0x80; // Cmd to send
	uint8_t data_buffer = data;

	HAL_SPI_Transmit(&hspi1, &wr_buffer, 1, 1000);
	HAL_SPI_Transmit(&hspi1, &data_buffer, 1, 1000);

	HAL_Delay_us(35);

	// Disable Chip select
	SS_HIGH;
//	HAL_Delay(200);
}

uint8_t test_spi_comms()
{
	uint8_t connected = 0;


	// Read IC ID
	uint8_t id = read_byte(REG_Product_ID);
	uint8_t rid = read_byte(REG_Revision_ID);

	printf("ID: 0x%02x\n", id);
	printf("rID: 0x%02x\n", rid);
	return id == 0x42;
}

void read_data()
{
	// Read IC ID
//	uint8_t id = read_byte(REG_Product_ID);
//	uint8_t rid = read_byte(REG_Revision_ID);
//	printf("ID: 0x%02x\n", id);
//	printf("rID: 0x%02x\n", rid);

	uint8_t motion = read_byte(REG_Motion);
//	printf("Motion: 0x%02x\n", motion);
//	uint8_t squal = read_byte(REG_SQUAL);
//	printf("SQUAL: 0x%02x\n", squal);

	if (motion & 0x80) // Motion detected
	{
		uint8_t delta_x_l = read_byte(REG_Delta_X_H);
		uint8_t delta_x_h = read_byte(REG_Delta_X_L);
		uint8_t delta_y_l = read_byte(REG_Delta_Y_H);
		uint8_t delta_y_h = read_byte(REG_Delta_Y_L);
		x.high = delta_x_h;
		x.low  = delta_x_l;
		y.high = delta_y_h;
		y.low  = delta_y_l;
//		printf("Motion detected: 0x%02x.%02x - 0x%02x.%02x\n", delta_x_h, delta_x_l, delta_y_h, delta_y_l);
//		printf("Motion detected: %d - %d\n", x.sum, y.sum);
	}
	else
	{
		x.sum = 0;
		y.sum = 0;
	}
}

void calc_position()
{
	static int32_t X_POS = 0, Y_POS = 0;

	X_POS += x.sum;
	Y_POS += y.sum;

	printf("Current POS: %5d - %5d\n", X_POS, Y_POS);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_DMA_Init();
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	printf("Testing download Mouse code!\n");

	uint8_t connection = test_spi_comms();
	printf("Has connection? %d\n", connection);
//	while(1);

	// PMW3360 Initial profile
	sensor_config();

	connection = test_spi_comms();
	printf("Has connection? %d\n", connection);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  write_byte(REG_Motion, 0x00);
//	  HAL_Delay(10);
	  read_data();

	  calc_position();
//	  HAL_Delay(10);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */

/*************** PMW3360-DM Sensor Initialization ***************/

// dpi argument is what's written to register 0x0f
// actual dpi value = (dpi + 1) * 100
static void pmw3360_init(const uint8_t dpi)
{
  SS_HIGH;
  HAL_Delay(3);

  SS_LOW;
  spi_write(0x3b, 0xb6);
	SS_HIGH;
  HAL_Delay(300);  // when power up shut down 300ms

  // drop and raise ncs to reset spi port
  SS_LOW;
  HAL_Delay_us(40);
	SS_HIGH;
  HAL_Delay_us(40);

	// power up reset
  SS_LOW;
  spi_write(0x3a, 0x5a);
	SS_HIGH;
  HAL_Delay(50);

  // read from 0x02 to 0x06
  SS_LOW;
  spi_read(0x02); //Motion
  spi_read(0x03); //X_lo
  spi_read(0x04); //X_Hi
  spi_read(0x05); //Y_Lo
  spi_read(0x06); //Y_Hi

  // srom download
  spi_write(0x10, 0x00);
  spi_write(0x13, 0x1d);
  SS_HIGH;
  HAL_Delay(10);
  SS_LOW;
  spi_write(0x13, 0x18);

  SPI_Send(0x62 | 0x80);
  for (uint16_t i = 0; i < SROM_LENGTH; i++)
  {
    HAL_Delay_us(16);
    SPI_Send(srom[i]);
  }
  HAL_Delay_us(18);
  SS_HIGH;
  HAL_Delay_us(200);

  // configuration/settings
  SS_LOW;
  spi_write(0x10, 0x00); // Rest mode & independant X/Y CPI disabled
  spi_write(0x0d, 0x00); // Camera angle
  spi_write(0x11, 0x00); // Camera angle fine tuning
  spi_write(0x0f, dpi); // DPI
  // LOD Stuff
  spi_write(0x63, 0x03); // LOD: 0x00 disable lift detection, 0x02 = 2mm, 0x03 = 3mm
  spi_write(0x2b, 0x80); // Minimum SQUAL for zero motion data (default: 0x10)��max is 0x80
  spi_write(0x2c, 0x0a); // Minimum Valid features (reduce SQUAL score) (default: 0x0a)
  SS_HIGH;
  HAL_Delay_us(200);
}

// angle snapping
static void angle_init(const uint8_t angle)
{
  SS_LOW;
  spi_write(0x42, angle); // Angle snapping (Angle snap disabled. This is the default value.): 0x00 = off, 0x80 = on
  SS_HIGH;
}


void HAL_Delay_us(__IO uint32_t delay_us)
{
  uint32_t first_value = 0;
  uint32_t current_value = 0;
  uint32_t reload = SysTick ->LOAD;

  uint32_t nus_number = delay_us * ((reload + 1) / 1000);
  uint32_t change_number = 0;

  first_value = SysTick ->VAL;
  while (1)
  {
    current_value = SysTick ->VAL;
    if (current_value != first_value)
    {

      if (current_value < first_value)
      {
        change_number += first_value - current_value;
        //change_number = first_value - current_value + change_number;
      }

      else
      {
        change_number += reload - current_value + first_value;
      }
      first_value = current_value;
      if (change_number >= nus_number)
      {
        break;
      }
    }
  }
}

//SPI Read sensor.
static inline void spi_write(const unsigned char addr, const unsigned char data)
{
  SPI_Send(addr | 0x80);
  SPI_Send(data);
  HAL_Delay_us(180); // maximum of t_SWW, t_SWR
}

static inline  const unsigned char spi_read(const unsigned char addr)
{
  SPI_Send(addr);
	HAL_Delay_us(160); // t_SRAD
  uint8_t data = SPI_Send(0x00);
  HAL_Delay_us(20);
  return data;
}


// SPI latency depends on your circuit layout and code.
unsigned char SPI_Send(unsigned char Txdata)
{
	/*
	1. Data into Buffer.
	1. Transmit data to PMW3360.
	2. Return Receive data from Inbox.
	*/

	uint8_t Rxdata;
	spi_transmit_receive(Txdata,&Rxdata);
//	Rxdata = SPI1_ReadWriteByte(Txdata);
	return Rxdata;
}
// data_in:data for transfer
// data_out: receive data
static uint8_t spi_transmit_receive(uint8_t data_in, uint8_t *data_out)
{
	uint32_t timeout_cnt;

	// Wait until TXE flag is set to send data
	timeout_cnt = 0;
	static const uint32_t timeout_cnt_num_send = 100;
	// Tx buffer not empty set 0;

	while(!LL_SPI_IsActiveFlag_TXE(SPI1))	//Tx�ǿգ�������
	{
		timeout_cnt ++;
		// Tx buffer empty set 1;
		if((timeout_cnt > timeout_cnt_num_send)||(LL_SPI_IsActiveFlag_TXE(SPI1)))	//Tx�գ�������
		{
			break;
		}
	}

	// Transmit data in 8 Bit mode
	LL_SPI_TransmitData8(SPI1, data_in);

	// Check BSY flag
	timeout_cnt = 0;
	static const uint32_t timeout_cnt_num_busy = 50;

	// SPI_Busy set 1
	while(LL_SPI_IsActiveFlag_BSY(SPI1))	// SPI_Busy
	{
		timeout_cnt ++;
        // SPI_Not_Busy set 0
		if((timeout_cnt > timeout_cnt_num_busy)||(!LL_SPI_IsActiveFlag_BSY(SPI1))) // SPI_Not_Busy
		{
			break;
		}
	}

	// Check RXNE flag
	timeout_cnt = 0;
	static const uint32_t timeout_cnt_num_recv = 200;

    // Rx buffer empty, set 0;
	while(!LL_SPI_IsActiveFlag_RXNE(SPI1))
	{
		timeout_cnt ++;
		// Rx buffer not empty set 1;
		if((timeout_cnt > timeout_cnt_num_recv)||(LL_SPI_IsActiveFlag_RXNE(SPI1)))//Rx�ǿ�
		{
			break;
		}
	}

	// Read 8-Bits in the data register
	*data_out = LL_SPI_ReceiveData8(SPI1);

	return *data_out;
}
uint8_t SPI1_ReadWriteByte(uint8_t TxData)  //
{
	uint8_t retry = 0;

	/* Check if Tx buffer is empty */
	while (!LL_SPI_IsActiveFlag_TXE(SPI1))
	{
		retry++;
		if(retry > 200)
			continue;
	}

	/* Write character in Data register.
	TXE flag is cleared by reading data in DR register */
	LL_SPI_TransmitData8(SPI1, TxData);
	retry = 0;

	/* Check if Rx buffer is not empty */
	while (!LL_SPI_IsActiveFlag_RXNE(SPI1))
	{
		retry++;
		if(retry > 200) continue;
	}

	/* received byte from SPI lines. */
	return LL_SPI_ReceiveData8(SPI1);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
