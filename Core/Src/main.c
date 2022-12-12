/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "complementary_filter.h"
#include "kalman_filter.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t mpuAddr = 0x68 << 1;
float dt = 0.01;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void calibration(float* cal) {
	uint8_t rawData[6];
	for(int i = 0; i < 3000; i++) {
		uint8_t accStart = 0x3B;
		HAL_I2C_Mem_Read(&hi2c1, mpuAddr, accStart, I2C_MEMADD_SIZE_8BIT, rawData, 6, HAL_MAX_DELAY);
		
		// ACC
		cal[0] += ((int16_t)(rawData[0] << 8 | rawData[1])) / 16384.0; // x
		cal[1] += ((int16_t)(rawData[2] << 8 | rawData[3])) / 16384.0; // y
		cal[2] += ((int16_t)(rawData[4] << 8 | rawData[5])) / 16384.0; // z
		
		uint8_t gyroStart = 0x43;
		HAL_I2C_Mem_Read(&hi2c1, mpuAddr, gyroStart, I2C_MEMADD_SIZE_8BIT, rawData, 6, HAL_MAX_DELAY);
		
		//GYRO
		cal[3] += ((int16_t)(rawData[0] << 8 | rawData[1])) / 131.0; // x
		cal[4] += ((int16_t)(rawData[2] << 8 | rawData[3])) / 131.0; // y
		cal[5] += ((int16_t)(rawData[4] << 8 | rawData[5])) / 131.0; // z
		
		HAL_Delay(3);
	}
	
	cal[0] /= 3000.0;
	cal[1] /= 3000.0;
	cal[2] /= 3000.0;
	cal[3] /= 3000.0;
	cal[4] /= 3000.0;
	cal[5] /= 3000.0;
}

void readRaw(float *data, float* cal) {
	uint8_t rawData[6];
	uint8_t accStart = 0x3B;
	HAL_I2C_Mem_Read(&hi2c1, mpuAddr, accStart, I2C_MEMADD_SIZE_8BIT, rawData, 6, HAL_MAX_DELAY);
	
	// ACC
	data[0] = ((int16_t)(rawData[0] << 8 | rawData[1])) / 16384.0 - cal[0]; // x
	data[1] = ((int16_t)(rawData[2] << 8 | rawData[3])) / 16384.0 - cal[1]; // y
	data[2] = ((int16_t)(rawData[4] << 8 | rawData[5])) / 16384.0 - cal[2] + 1; // z
	uint8_t gyroStart = 0x43;
	HAL_I2C_Mem_Read(&hi2c1, mpuAddr, gyroStart, I2C_MEMADD_SIZE_8BIT, rawData, 6, HAL_MAX_DELAY);
	
	//GYRO
	data[3] = ((int16_t)(rawData[0] << 8 | rawData[1])) / 131.0 - cal[3]; // x
	data[4] = ((int16_t)(rawData[2] << 8 | rawData[3])) / 131.0 - cal[4]; // y
	data[5] = ((int16_t)(rawData[4] << 8 | rawData[5])) / 131.0 - cal[5]; // z
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
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	float calibrate[6];
	calibration(calibrate);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	float compFiltered[3];
	float kalFiltered[3];
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		float raw[6];
		readRaw(raw, calibrate);
		complementary(raw, compFiltered, dt);
		kalman(raw, kalFiltered, dt);
		
		char result[70] = { '\0' };
		//sprintf(result, "%.2f %.2f %.2f %.2f %.2f %.2f", raw[0], raw[1], raw[2], raw[3], raw[4], raw[5]);
		sprintf(result, "%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f", raw[0], raw[1], raw[2], raw[3], raw[4], compFiltered[0], compFiltered[1], kalFiltered[0], kalFiltered[1]);
		HAL_UART_Transmit(&huart1, (uint8_t*)result, sizeof(result), HAL_MAX_DELAY);
		
		uint8_t msg[] = "\r\n";
		HAL_UART_Transmit(&huart1, msg, sizeof(msg), HAL_MAX_DELAY);
		
		HAL_Delay(dt*1000); // 1/1khz
  }
  /* USER CODE END 3 */
}

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */
	uint8_t PWR_MGMT_1 = 0x00; // power
	HAL_I2C_Mem_Write(&hi2c1, mpuAddr, 0x6B, I2C_MEMADD_SIZE_8BIT, &PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, HAL_MAX_DELAY);
	uint8_t GyroSensitivity = 0x00; // gyro sensitivity ±250°/s 131 LSB/°/s
	HAL_I2C_Mem_Write(&hi2c1, mpuAddr, 0x1B, I2C_MEMADD_SIZE_8BIT, &GyroSensitivity, I2C_MEMADD_SIZE_8BIT, HAL_MAX_DELAY);
	uint8_t AccSensitivity = 0x00; // acceleromter sensitivity ±2g 16384 LSB/g
	HAL_I2C_Mem_Write(&hi2c1, mpuAddr, 0x1C, I2C_MEMADD_SIZE_8BIT, &AccSensitivity, I2C_MEMADD_SIZE_8BIT, HAL_MAX_DELAY);
	uint8_t SMPLRT_DIV = 0x07; // 8khz/(7+1) = 1khz sample rate
	HAL_I2C_Mem_Write(&hi2c1, mpuAddr, 0x19, I2C_MEMADD_SIZE_8BIT, &SMPLRT_DIV, I2C_MEMADD_SIZE_8BIT, HAL_MAX_DELAY);
  /* USER CODE END I2C1_Init 2 */
	
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

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