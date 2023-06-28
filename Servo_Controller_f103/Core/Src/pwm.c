/*
 * pwm.c
 *
 *  Created on: 28/06//2023
 *      Author: JV4K
 */

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

void pwm_initDriver(pwmControl_t *driver, TIM_HandleTypeDef *htim, uint8_t pwmChannel,
		GPIO_TypeDef *dir1_Port, uint32_t dir1_Pin, GPIO_TypeDef *dir2_Port, uint32_t dir2_Pin) {
	driver->dir1_Port = dir1_Port;
	driver->dir1_Pin = dir1_Pin;

	driver->dir2_Port = dir2_Port;
	driver->dir2_Pin = dir2_Pin;

	driver->htim = htim;
	driver->timerChannel1 = pwmChannel;
}

void pwm_dutyLimits(pwmControl_t *driver, uint16_t minDuty, uint16_t maxDuty) {
	driver->minDuty = minDuty;
	driver->maxDuty = maxDuty;
}

void pwm_setSpeed(pwmControl_t *driver, int16_t, duty) {
	if (!_duty) {
		driver->htim->Instance->CCR1 = 0;
		HAL_GPIO_WritePin(driver->dir1_Port, driver->dir1_Pin, 0);
		HAL_GPIO_WritePin(driver->dir1_Port, driver->dir1_Pin, 0);
	} else {
		driver->_duty = constrain(duty, driver->minDuty, driver->maxDuty);
		driver->htim->Instance->CCR1 = abs(_duty);
		if (_duty > 0) {
			HAL_GPIO_WritePin(driver->dir1_Port, driver->dir1_Pin, 1);
			HAL_GPIO_WritePin(driver->dir1_Port, driver->dir1_Pin, 0);
		} else {
			HAL_GPIO_WritePin(driver->dir1_Port, driver->dir1_Pin, 0);
			HAL_GPIO_WritePin(driver->dir1_Port, driver->dir1_Pin, 1);
		}
	}
}

void pwm_break(pwmControl_t *driver) {
	driver->_duty = 0;
	driver->htim->Instance->CCR1 = 0;
	HAL_GPIO_WritePin(driver->dir1_Port, driver->dir1_Pin, 1);
	HAL_GPIO_WritePin(driver->dir1_Port, driver->dir1_Pin, 1);
}