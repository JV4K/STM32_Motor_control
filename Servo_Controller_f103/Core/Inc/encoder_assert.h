/*
 * encoder_assert.h
 *
 *  Created on: 01/04/2023
 *      Author: JV4K
 */

#ifndef INC_ECODER_ASSERT_H_
#define INC_ECODER_ASSERT_H_

#endif /* INC_ECODER_ASSERT_H_ */

#include "main.h"

#define PI 3.14159265

typedef struct{
	TIM_HandleTypeDef* htim;
	int16_t CPR; // Counts per rotation
	uint16_t CntValUint; // Data from CNT stored here
	int16_t CntValInt; // Signed ticks
	int32_t CurRots; // Full rotations
	int32_t CurTicks;
	float Angle; // Current angle in rads
	float AngVel; // Angular velocity in rad/s
	float SamplingPeriod; // Sampling period in seconds;
	float PrevTicks; // Previous angle (needed to calculate velocity)
	float DeltTicks; // Delta angle

}ENCODER;

void EncoderReset(volatile ENCODER*);
void EncoderPosition(volatile ENCODER*);
void EncoderVelocity(volatile ENCODER*);
void EncoderInit(volatile ENCODER*, TIM_HandleTypeDef*, int16_t, float);
