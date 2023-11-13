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
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <servocontroller.h>
#include <filters.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define CUR_MED_ORDER 35 // Order of median filter
#define A1 0.00000125208
#define B1 0.00010988452

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t readyFlag; // When true, all the algorithms and systems are initialized and ready to operate

// Servo structure instances
servocontrol_t servo1;
servocontrol_t servo2;

volatile uint16_t adc[2]; // DMA buffer to store ADC values from current sensor

uint8_t convCpltFlag; // DMA interrupt flag. True means that data in buffer is up to date.
float adc1_med, adc1_med_ema; // Variables to store filtered ADC data (median filter -> exponential moving average)
extern float current;
MedianFilter *median_filter1; // Median filter instance
EMAFilter *ema_filter1; // Exponential moving average filter instance

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);

// Some functions to shorten the code in main()
// Look inside to see how to initialize a servo
void initServo1Func();
void initServo2Func();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	HAL_ADCEx_Calibration_Start(&hadc1);

	// Initialization of 1st motor controller
	initServo1Func();

	// Initialization of 2nd motor controller
	initServo2Func();

	// Clear CNT and start timers in encoder mode
	__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

	// Start TIM3 in PWM mode and also turn on interrupts
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim3);

	// Allow powering motors by setting Enable pins of a driver HIGH
	HAL_GPIO_WritePin(ENA_GPIO_Port, ENA_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(ENA2_GPIO_Port, ENA2_Pin, GPIO_PIN_SET);

	// Starting DMA for capturing current measures from ADC
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &adc, 2);

	// Initialization of filters
	ema_filter1 = initEMAFilter(0.09, 0);
	median_filter1 = initMedianFilter(CUR_MED_ORDER);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		// TODO write comments about this code, make functions for multiple filtering
		if (convCpltFlag) {
			adc1_med = updateAndGetMedian(median_filter1, adc[0]);
			adc1_med_ema = updateEMA(ema_filter1, adc1_med);
			current = (adc1_med_ema * adc1_med_ema * A1 + adc1_med_ema * B1)
					* servo_getCurrentDirection(&servo1);
			convCpltFlag = 0;
		}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc->Instance == ADC1) {
		convCpltFlag = 1;
	}
}

void initServo1Func() {
	servo_baseInit(&servo1, Triple, 1200, 21.3, 0);
	servo_encoderInit(&servo1, &htim1, 44);
	servo_driverInit(&servo1, &htim3, 1, INA_GPIO_Port, INA_Pin, INB_GPIO_Port,
	INB_Pin, 0, 998);
	servo_positionInit(&servo1, 50, 0, 0, 0.00033333, 0);
	servo_velocityInit(&servo1, 20, 10, 0, 0.01, 1);
	servo_currentInit(&servo1, 0.5, 0.01, 0, 0, 0.000055555, 0);
	servo_setPositionTolerance(&servo1, 0.026);
}

void initServo2Func() {
	servo_baseInit(&servo2, Triple, 1200, 21.3, 1);
	servo_encoderInit(&servo2, &htim2, 44);
	servo_driverInit(&servo2, &htim3, 2, INA2_GPIO_Port, INA2_Pin,
	INB2_GPIO_Port, INB2_Pin, 0, 998);
	servo_positionInit(&servo2, 50, 0, 0, 0.00033333, 0);
	servo_velocityInit(&servo2, 20, 10, 0, 0.01, 1);
	servo_currentInit(&servo2, 0.5, 0.01, 0, 0, 0.000055555, 0);
	servo_setPositionTolerance(&servo2, 0.026);
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
	while (1) {
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
