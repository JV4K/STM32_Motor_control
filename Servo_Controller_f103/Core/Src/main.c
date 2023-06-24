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
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "reg.h"
#include "encoder_assert.h"

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

volatile PIDREG ang_reg1;
volatile PIDREG vel_reg1;

volatile PIDREG ang_reg2;
volatile PIDREG vel_reg2;

float deltt;
float threshold;

volatile ENCODER enc1;
volatile ENCODER enc2;
volatile float FilteredVel1;
volatile float FilteredVel2;

volatile uint16_t ModeCounter;
volatile uint16_t Mode;
uint16_t Diag = 0;
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
	deltt = 0.00033333;

	RegParamsUpd(&ang_reg1, 0.6, 0, 0.5, deltt, 800, -800, 3, 0, 0);
	RegParamsUpd(&vel_reg1, 0.25, 1.3, 0, 0.01, 998, -998, 0, 0, 0);
	EncoderInit(&enc1, &htim1, 44, 0.01);

	RegParamsUpd(&ang_reg2, 0.6, 0, 0.5, deltt, 800, -800, 3, 0, 0);
	RegParamsUpd(&vel_reg2, 0.25, 1.3, 0, 0.01, 998, -998, 0, 0, 0);
	EncoderInit(&enc2, &htim2, 44, 0.01);

	__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);

	__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim3);

	HAL_GPIO_WritePin(ENA_GPIO_Port, ENA_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(ENA2_GPIO_Port, ENA2_Pin, GPIO_PIN_SET);
	ModeCounter = 0;

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		if (ModeCounter <= 3000) {
			ang_reg1.Ref = 710;
			ang_reg2.Ref = -710;
		} else {
			if ((ModeCounter > 3000) && (ModeCounter <= 6000)) {
				if (STMNO == 1) {
					ang_reg1.Ref = 0;
					ang_reg2.Ref = -1420;
				} else {
					ang_reg1.Ref = 1420;
					ang_reg2.Ref = 0;
				}

			} else {
				if ((ModeCounter > 6000) && (ModeCounter <= 9000)) {
					if (STMNO == 1) {
						ang_reg1.Ref = 401.5;
						ang_reg2.Ref = -1018.5;
					} else {
						ang_reg1.Ref = 1821.5;
						ang_reg2.Ref = 401.5;
					}
				} else {
					if ((ModeCounter > 9000) && (ModeCounter <= 13000)) {
						Diag = 1;
						if (STMNO == 1) {
							ang_reg2.Ref = -2438.5;
						} else {

							ang_reg1.Ref = 3241.5;
						}
					} else {
						if ((ModeCounter > 13000) && (ModeCounter <= 16000)) {
							Diag = 0;
							if (STMNO == 1) {
								ang_reg1.Ref = 0;
								ang_reg2.Ref = -2840;
							} else {
								ang_reg1.Ref = 2840;
								ang_reg2.Ref = 0;
							}
						} else {
							if (ModeCounter > 16000) {
								EncoderReset(&enc1);
								EncoderReset(&enc2);
								ang_reg1.Ref = 0;
								ang_reg2.Ref = 0;
								ModeCounter = 0;
							}
						}
					}
				}
			}
		}

// Direction and actuation
		if (Diag && (STMNO == 1)) {
			HAL_GPIO_WritePin(INA_GPIO_Port, INA_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(INB_GPIO_Port, INB_Pin, GPIO_PIN_SET);
		} else {
			if (vel_reg1.Out == 0) {
				HAL_GPIO_WritePin(INA_GPIO_Port, INA_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(INB_GPIO_Port, INB_Pin, GPIO_PIN_SET);
			} else {
				if (vel_reg1.Out > 0) {
					HAL_GPIO_WritePin(INA_GPIO_Port, INA_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(INB_GPIO_Port, INB_Pin, GPIO_PIN_RESET);
					TIM3->CCR1 = vel_reg1.Out;
				} else {
					HAL_GPIO_WritePin(INA_GPIO_Port, INA_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(INB_GPIO_Port, INB_Pin, GPIO_PIN_SET);
					TIM3->CCR1 = -(vel_reg1.Out);
				}
			}
		}

		if (Diag && (STMNO == 2)) {
			HAL_GPIO_WritePin(INA2_GPIO_Port, INA2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(INB2_GPIO_Port, INB2_Pin, GPIO_PIN_SET);
		} else {
			if (vel_reg2.Out == 0) {
				HAL_GPIO_WritePin(INA2_GPIO_Port, INA2_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(INB2_GPIO_Port, INB2_Pin, GPIO_PIN_SET);
				//			HAL_GPIO_WritePin(ENA2_GPIO_Port, ENA2_Pin, GPIO_PIN_RESET);
			} else {
				//			HAL_GPIO_WritePin(ENA2_GPIO_Port, ENA2_Pin, GPIO_PIN_SET);
				if (vel_reg2.Out > 0) {
					HAL_GPIO_WritePin(INA2_GPIO_Port, INA2_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(INB2_GPIO_Port, INB2_Pin, GPIO_PIN_RESET);
					TIM3->CCR2 = vel_reg2.Out;
				} else {
					HAL_GPIO_WritePin(INA2_GPIO_Port, INA2_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(INB2_GPIO_Port, INB2_Pin, GPIO_PIN_SET);
					TIM3->CCR2 = -(vel_reg2.Out);
				}
			}
		}

		// Update feedback and reference
		ang_reg1.Fdb = enc1.Angle;
		vel_reg1.Ref = ang_reg1.Out;
		vel_reg1.Fdb = FilteredVel1;

		ang_reg2.Fdb = enc2.Angle;
		vel_reg2.Ref = ang_reg2.Out;
		vel_reg2.Fdb = FilteredVel2;

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
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
