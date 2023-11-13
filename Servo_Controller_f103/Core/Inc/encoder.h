/*
 * encoder.h
 *
 *  Created on: 25/06/2023
 *      Author: JV4K
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#endif /* INC_ENCODER_H_ */
#include <main.h>
#include <filters.h>

#define M_PI 3.14159265358979323846
#define MED_ORDER 5

typedef struct {
	// Handler of timer in encoder mode
	TIM_HandleTypeDef *htim;

	// Value by which counter register increments each revolution of shaft
	uint16_t countsPerRevolution;

	// Current angle in rads
	float angle;

	// Current angle in rads per second
	float angularVelocity;
	int16_t currentTicks;
	int32_t fullRevolutions;
	float dt;
	float gearRatio;
	float previousAngle;

	// Velocity filter
	MedianFilter* filter;
} encoder_t;

// Initialization with tim handler (e.g. &htim1), CPR, velocity update period and gear ratio (default = 1)
void encoder_init(encoder_t *encoder, TIM_HandleTypeDef *timerHandle, uint16_t CPR, float dt, float gearRatio);
void encoder_updatePosition(encoder_t *encoder);
void encoder_updateVelocity(encoder_t *encoder);
void encoder_reset(encoder_t *encoder);

float encoder_getAngle(encoder_t *encoder);
float encoder_getVelocity(encoder_t *encoder);

