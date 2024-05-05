/*
 * encoder_iq18.h
 *
 *  Created on: Dec 18, 2023
 *      Author: zhmis
 */

#ifndef INC_ENCODER_IQ18_H_
#define INC_ENCODER_IQ18_H_

#include <main.h>
#include "IQmathLib.h"
#include "ema_iq18.h"

#define PI_IQ18 823549 // IQ18 representation of PI

typedef struct {
	// Handler of timer in encoder mode
	TIM_HandleTypeDef *htim;

	// Value by which counter register increments each revolution of shaft
	uint16_t countsPerRevolution;

	// Current angle in rads
	_iq18 angle;

	// Current angle in rads per second
	_iq18 angularVelocity;
	int16_t currentTicks;
	_iq18 fullRevolutions;
	_iq18 cur_angle_after_gearbox;
	_iq18 dt;
	_iq18 gearRatio;
	_iq18 previousAngle;

	int8_t reverse_flag;

	// Velocity filter
	EMA_iq18 *filter;
} encoder_iq18_t;

void encoder_init_iq18(encoder_iq18_t *encoder, TIM_HandleTypeDef *timerHandle,
		uint16_t CPR, float dt, float gearRatio, float vel_filter_k,
		uint8_t is_reversed);
void encoder_updatePosition_iq18(encoder_iq18_t *encoder);
void encoder_updateVelocity_qep_iq18(encoder_iq18_t *encoder);
void encoder_reset_iq18(encoder_iq18_t *encoder);

_iq18 encoder_getAngle_iq18(encoder_iq18_t *encoder);
_iq18 encoder_getVelocity_iq18(encoder_iq18_t *encoder);

#endif /* INC_ENCODER_IQ18_H_ */
