/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32f1xx_it.c
 * @brief   Interrupt Service Routines.
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
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <pid.h>
#include <encoder.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define NUM_READ 10

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint16_t freq3khz;
uint16_t freq100hz;
float setAngle = 7;
//extern volatile uint16_t ModeCounter;

volatile float FilteredVel1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
float Median_velocity_1(float);
float SMA_velocity_1(float);

float Median_velocity_2(float);
float SMA_velocity_2(float);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim3;
/* USER CODE BEGIN EV */
pid_t angle_controller;
pid_t velocity_controller;
encoder_t encoder;

float MedianVel;
float FilteredVel;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void) {
	/* USER CODE BEGIN NonMaskableInt_IRQn 0 */

	/* USER CODE END NonMaskableInt_IRQn 0 */
	/* USER CODE BEGIN NonMaskableInt_IRQn 1 */
	while (1) {
	}
	/* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void) {
	/* USER CODE BEGIN HardFault_IRQn 0 */

	/* USER CODE END HardFault_IRQn 0 */
	while (1) {
		/* USER CODE BEGIN W1_HardFault_IRQn 0 */
		/* USER CODE END W1_HardFault_IRQn 0 */
	}
}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void) {
	/* USER CODE BEGIN MemoryManagement_IRQn 0 */

	/* USER CODE END MemoryManagement_IRQn 0 */
	while (1) {
		/* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
		/* USER CODE END W1_MemoryManagement_IRQn 0 */
	}
}

/**
 * @brief This function handles Prefetch fault, memory access fault.
 */
void BusFault_Handler(void) {
	/* USER CODE BEGIN BusFault_IRQn 0 */

	/* USER CODE END BusFault_IRQn 0 */
	while (1) {
		/* USER CODE BEGIN W1_BusFault_IRQn 0 */
		/* USER CODE END W1_BusFault_IRQn 0 */
	}
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void) {
	/* USER CODE BEGIN UsageFault_IRQn 0 */

	/* USER CODE END UsageFault_IRQn 0 */
	while (1) {
		/* USER CODE BEGIN W1_UsageFault_IRQn 0 */
		/* USER CODE END W1_UsageFault_IRQn 0 */
	}
}

/**
 * @brief This function handles System service call via SWI instruction.
 */
void SVC_Handler(void) {
	/* USER CODE BEGIN SVCall_IRQn 0 */

	/* USER CODE END SVCall_IRQn 0 */
	/* USER CODE BEGIN SVCall_IRQn 1 */

	/* USER CODE END SVCall_IRQn 1 */
}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void) {
	/* USER CODE BEGIN DebugMonitor_IRQn 0 */

	/* USER CODE END DebugMonitor_IRQn 0 */
	/* USER CODE BEGIN DebugMonitor_IRQn 1 */

	/* USER CODE END DebugMonitor_IRQn 1 */
}

/**
 * @brief This function handles Pendable request for system service.
 */
void PendSV_Handler(void) {
	/* USER CODE BEGIN PendSV_IRQn 0 */

	/* USER CODE END PendSV_IRQn 0 */
	/* USER CODE BEGIN PendSV_IRQn 1 */

	/* USER CODE END PendSV_IRQn 1 */
}

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void) {
	/* USER CODE BEGIN SysTick_IRQn 0 */

	/* USER CODE END SysTick_IRQn 0 */
	HAL_IncTick();
	/* USER CODE BEGIN SysTick_IRQn 1 */

	/* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles TIM3 global interrupt.
 */
void TIM3_IRQHandler(void) {
	/* USER CODE BEGIN TIM3_IRQn 0 */

	freq3khz++;
	freq100hz++;

	if (freq3khz >= 6) {
		encoder_updatePosition(&encoder);
		pid_calculate(&angle_controller, setAngle, encoder_getAngle(&encoder));

		freq3khz = 0;
	}

	if (freq100hz >= 180) {
		encoder_updateVelocity(&encoder);
		MedianVel = Median_velocity_1(encoder_getVelocity(&encoder));
		FilteredVel = SMA_velocity_1(MedianVel);
		pid_calculate(&velocity_controller, pid_getOutput(&angle_controller), encoder_getVelocity(&encoder));

		freq100hz = 0;

		if (pid_getOutput(&velocity_controller) == 0) {
			HAL_GPIO_WritePin(INA_GPIO_Port, INA_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(INB_GPIO_Port, INB_Pin, GPIO_PIN_SET);
		} else {
			if (pid_getOutput(&velocity_controller) > 0) {
				HAL_GPIO_WritePin(INA_GPIO_Port, INA_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(INB_GPIO_Port, INB_Pin, GPIO_PIN_RESET);
				TIM3->CCR1 = pid_getOutput(&velocity_controller);
			} else {
				HAL_GPIO_WritePin(INA_GPIO_Port, INA_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(INB_GPIO_Port, INB_Pin, GPIO_PIN_SET);
				TIM3->CCR1 = -(pid_getOutput(&velocity_controller));
			}
		}
	}
	/* USER CODE END TIM3_IRQn 0 */
	HAL_TIM_IRQHandler(&htim3);
	/* USER CODE BEGIN TIM3_IRQn 1 */

	/* USER CODE END TIM3_IRQn 1 */
}

/* USER CODE BEGIN 1 */
float Median_velocity_1(float newVal) {
	static float buffer[NUM_READ];
	static uint32_t count = 0;
	buffer[count] = newVal;
	if ((count < NUM_READ - 1) && (buffer[count] > buffer[count + 1])) {
		for (int i = count; i < NUM_READ - 1; i++) {
			if (buffer[i] > buffer[i + 1]) {
				float buff = buffer[i];
				buffer[i] = buffer[i + 1];
				buffer[i + 1] = buff;
			}
		}
	} else {
		if ((count > 0) && (buffer[count - 1] > buffer[count])) {
			for (int i = count; i > 0; i--) {
				if (buffer[i] < buffer[i - 1]) {
					float buff = buffer[i];
					buffer[i] = buffer[i - 1];
					buffer[i - 1] = buff;
				}
			}
		}
	}
	if (++count >= NUM_READ)
		count = 0;
	return buffer[(int) NUM_READ / 2];
}

float SMA_velocity_1(float newVal) {
	static int t = 0;
	static float vals[NUM_READ];
	static float average = 0;
	if (++t >= NUM_READ)
		t = 0; // перемотка t
	average -= vals[t];         // вычитаем старое
	average += newVal;          // прибавляем новое
	vals[t] = newVal;           // запоминаем в массив
	return ((float) average / NUM_READ);
}

float Median_velocity_2(float newVal) {
	static float buffer[NUM_READ];
	static uint32_t count = 0;
	buffer[count] = newVal;
	if ((count < NUM_READ - 1) && (buffer[count] > buffer[count + 1])) {
		for (int i = count; i < NUM_READ - 1; i++) {
			if (buffer[i] > buffer[i + 1]) {
				float buff = buffer[i];
				buffer[i] = buffer[i + 1];
				buffer[i + 1] = buff;
			}
		}
	} else {
		if ((count > 0) && (buffer[count - 1] > buffer[count])) {
			for (int i = count; i > 0; i--) {
				if (buffer[i] < buffer[i - 1]) {
					float buff = buffer[i];
					buffer[i] = buffer[i - 1];
					buffer[i - 1] = buff;
				}
			}
		}
	}
	if (++count >= NUM_READ)
		count = 0;
	return buffer[(int) NUM_READ / 2];
}

float SMA_velocity_2(float newVal) {
	static int t = 0;
	static float vals[NUM_READ];
	static float average = 0;
	if (++t >= NUM_READ)
		t = 0; // перемотка t
	average -= vals[t];         // вычитаем старое
	average += newVal;          // прибавляем новое
	vals[t] = newVal;           // запоминаем в массив
	return ((float) average / NUM_READ);
}
/* USER CODE END 1 */
