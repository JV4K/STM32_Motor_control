/*
 * encoder.h
 *
 *  Created on: 25 июн. 2023 г.
 *      Author: JV4K
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#endif /* INC_ENCODER_H_ */

#include <main.h>

#define PI 3.14159265

typedef struct {
	// Handler of timer in encoder mode
	TIM_HandleTypeDef *htim;

	// Value by which counter register increments each revolution of shaft
	int16_t countsPerRevolution;

	uint16_t registerValue;

	// Currently stored ticks within a full revolution of shaft
	int16_t currentTicks;

	int32_t fullRevolutions;

	// Current angle in rads
	float angle;

	// Current angle in rads
	float velocity; // Angular velocity in rad/s

	// Update period
	float dt;

	float previousAngle;
//	float deltaAngle;

} encoder_t;
