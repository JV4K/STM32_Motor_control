/*
 * encoder_iq18.c
 *
 *  Created on: Dec 18, 2023
 *      Author: JV4K
 */

#include "encoder_iq18.h"

void encoder_init_iq18(encoder_iq18_t *encoder, TIM_HandleTypeDef *timerHandle,
		uint16_t CPR, float dt, float gearRatio, float vel_filter_k,
		uint8_t is_reversed) {
	encoder->htim = timerHandle;
	encoder->countsPerRevolution = CPR;
	encoder->dt = _IQ18(dt);
	encoder->filter = initEMA_iq18(vel_filter_k);
	if (gearRatio) {
		encoder->gearRatio = _IQ18(gearRatio);
	} else {
		encoder->gearRatio = _IQ18(1);
	}

	if (!is_reversed) {
		encoder->reverse_flag = 1;
	} else {
		encoder->reverse_flag = -1;
	}
}
void encoder_updatePosition_iq18(encoder_iq18_t *encoder) {
	encoder->currentTicks = (int16_t) encoder->htim->Instance->CNT;

	encoder->fullRevolutions += _IQ18div(
			_IQ18(encoder->currentTicks / encoder->countsPerRevolution),
			encoder->gearRatio);

	encoder->currentTicks = encoder->currentTicks
			% encoder->countsPerRevolution;
	encoder->htim->Instance->CNT = (uint16_t) encoder->currentTicks;

	encoder->angle = _IQ18mpy(
			(encoder->fullRevolutions
					+ _IQ18div(
							_IQ18div(_IQ18(encoder->currentTicks),
									_IQ18(encoder->countsPerRevolution)),
							encoder->gearRatio)) << 1,
			PI_IQ18);
}
void encoder_updateVelocity_qep_iq18(encoder_iq18_t *encoder) {
	_iq18 velocity = _IQ18div((encoder->angle - encoder->previousAngle),
			encoder->dt);
//	encoder->angularVelocity = velocity;
	encoder->angularVelocity = updateEMA_iq18(encoder->filter, velocity);
	encoder->previousAngle = encoder->angle;
}
void encoder_reset_iq18(encoder_iq18_t *encoder) {
	encoder->htim->Instance->CNT = 0;
	encoder->previousAngle = 0;
	encoder->angle = 0;
	encoder->fullRevolutions = 0;
	encoder->currentTicks = 0;
}

_iq18 encoder_getAngle_iq18(encoder_iq18_t *encoder) {
	return encoder->angle * encoder->reverse_flag;
}
_iq18 encoder_getVelocity_iq18(encoder_iq18_t *encoder) {
	return encoder->angularVelocity * encoder->reverse_flag;
}
