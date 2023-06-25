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

#define M_PI 3.14159265358979323846

typedef struct {
	// Handler of timer in encoder mode
	TIM_HandleTypeDef *htim;

	// Value by which counter register increments each revolution of shaft
	int16_t countsPerRevolution;

	// Current angle in rads
	float angle;

	// Current angle in rads per second
	float velocity;

	uint16_t registerValue;
	int16_t currentTicks;
	int32_t fullRevolutions;
	float dt;
	float gearRatio;
	float previousAngle;
} encoder_t;
