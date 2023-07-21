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
#include <main.h>
#include <tim.h>
#include <gpio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <servocontroller.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//#define STMNO 1
#define STMNO 2

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
servocontrol_t servo1;
servocontrol_t servo2;

volatile uint32_t ModeCounter;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */

	servo_baseInit(&servo1, Double, 1200, 21.3, 0);
	//TODO: check PPR of encoder
	servo_encoderInit(&servo1, &htim1, 44);
	servo_driverInit(&servo1, &htim3, 1, INA_GPIO_Port, INA_Pin, INB_GPIO_Port, INB_Pin, 0, 998);
	servo_positionInit(&servo1, 50, 0, 0, 0.00033333, 0);
	servo_velocityInit(&servo1, 20, 10, 0, 0.01, 1);
	servo_setPositionTolerance(&servo1, 0.026);

	servo_baseInit(&servo2, Double, 1200, 21.3, 1);
	servo_encoderInit(&servo2, &htim2, 44);
	servo_driverInit(&servo2, &htim3, 2, INA2_GPIO_Port, INA2_Pin, INB2_GPIO_Port, INB2_Pin, 0, 998);
	servo_positionInit(&servo2, 50, 0, 0, 0.00033333, 0);
	servo_velocityInit(&servo2, 20, 10, 0, 0.01, 1);
	servo_setPositionTolerance(&servo2, 0.026);

	__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);

	__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim3);

	HAL_GPIO_WritePin(ENA_GPIO_Port, ENA_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(ENA2_GPIO_Port, ENA2_Pin, GPIO_PIN_SET);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		if (ModeCounter <= 1000) {
			servo_controlPosition(&servo1, 6);
			servo_controlPosition(&servo2, 6);
		} else {
			if ((ModeCounter > 1000) && (ModeCounter <= 2000)) {
				if (STMNO == 1) {
					servo_controlPosition(&servo1, -1.667);
					servo_controlPosition(&servo2, 15);
				} else {
					servo_controlPosition(&servo1, 15);
					servo_controlPosition(&servo2, -1.667);
				}

			} else {
				if ((ModeCounter > 2000) && (ModeCounter <= 3000)) {
					if (STMNO == 1) {
						servo_controlPosition(&servo1, -1.667);
						servo_controlPosition(&servo2, 1.667);
					} else {
						servo_controlPosition(&servo1, 1.667);
						servo_controlPosition(&servo2, -1.667);
					}
				} else {
					if ((ModeCounter > 3000) && (ModeCounter <= 5500)) {
						if (STMNO == 1) {
							servo_controlPosition(&servo1, 36.333);
							servo_controlPosition(&servo2, -36.333);
						} else {
							servo_controlPosition(&servo1, 39.666);
							servo_controlPosition(&servo2, -39.666);
						}
					} else{
						if(ModeCounter > 5500){
							NVIC_SystemReset();
						}
					}
				}
			}
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
void SystemClock_Config(void) {
RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
	Error_Handler();
}

/** Initializes the CPU, AHB and APB buses clocks
 */
RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1
		| RCC_CLOCKTYPE_PCLK2;
RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
	Error_Handler();
}
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
