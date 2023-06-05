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
	float Fdb;	// Feedback
	float Err;	// Error
	float Kp;	// Proportional gain
	float Ki;	// Integral gain
	float Kd;	// Differential gain

	// Threshold of angle accuracy (if error = 0 +-threshold, error = 0)
	float ZeroDrift;

	float OutDeadZone; // Minimal output which is not zero

	float Up;	// Proportional component
	float Ui;	// Integral component
	float Ud;	// Differential component
	float DeltaT;	// Sampling period
	float Kt; // Anti-windup gain
	int32_t OutPreSat;	// Output signal before saturation
	int32_t OutMax;	// Maximum limit for output signal
	int32_t OutMin;	// Minimum limit for output signal
	int32_t Out;	// Final Output signal

	int32_t PrevErr;	// Storing previous error here
} PIDREG;

void pid_reg_reset(volatile PIDREG*);
void pid_reg_calc(volatile PIDREG*);
void RegParamsUpd(volatile PIDREG *v, float kp, float ki, float kd, float dt,
		int32_t MaxOut, int32_t MinOut, float ZeroDrift, float DeadZone,
		float Antiwindup);
