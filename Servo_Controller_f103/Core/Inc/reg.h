/*
 * reg.h
 *
 *  Created on: 31/03/2023
 *      Author: JV4K
 */

#ifndef INC_REG_H_
#define INC_REG_H_

#endif /* INC_REG_H_ */

#include "main.h"

typedef struct {
	float Ref;	// Reference AKA Task
//	float PrevRef;
	float Fdb;	// Feedback
	float Err;	// Error
	float Kp;	// Proportional gain
	float Ki;	// Integral gain
	float Kd;	// Differential gain
	int8_t ShrinkFlag; // Decides whether error needs to be cut if in a threshold
	float ErrThreshold;	// Threshold of angle accuracy (if error = 0 +-threshold, error = 0)

	float Up;	// Proportional component
	float Ui;	// Integral component
	float Ud;	// Differential component
	float DeltaT;	// Sampling period
	int32_t OutPreSat;	// Output signal before saturation
	int32_t OutMax;	// Maximum limit for output signal
	int32_t OutMin;	// Minimum limit for output signal
	int32_t Out;	// Final Output signal

	//float KIntegralAntiWindUp; // Gain which is used to make integral component anti-windup jacket
	//int32_t SatErr;	// Part of output signal which was removed after saturation

	int32_t PrevErr;	// Storing previous error here
	// Flag which states whether previous output signal is in saturation zone or not
	int8_t InSaturationFlag;
} PIDREG;

void pid_reg_reset(volatile PIDREG*);
void pid_reg_calc(volatile PIDREG*);
void RegParamsUpd(volatile PIDREG*, float, float, float, float, int32_t, int32_t, int8_t, float);

