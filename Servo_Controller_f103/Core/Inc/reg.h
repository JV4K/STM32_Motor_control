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
	float setpoint;
	float feedback;
	float error;
	float kp;
	float ki;
	float kd;

	// If set, represents a gain of integral component anti-windup algorithm
	float Kt;

	// If set, controller neglects error in range of [-toleranceBand; +toleranceBand]
	float toleranceBand;

	// If set, output can be either can be 0, >deadZone or <-deadZone
	float deadZone;

	// PID components
	float P;
	float I;
	float D;

	// Sampling period in seconds
	float dt;

	// Limits
	float outputUpperLimit;
	float outputLowerLimit;

	float rawOutput; // No saturation
	float output; // Final Output signal

	float previousError;
} PidController_t;

// Initialization with gains and update period
void pid_init(PidController_t *pid, float newKp, float newKi, float newKd,
		float newDt);

// Setters for gains and update period (frequency)
void pid_setGains(PidController_t *pid, float newKp, float newKi, float newKd);
void pid_setPeriod(PidController_t *pid, float newDt); // Sets update period in seconds
void pid_setFrequencyHz(PidController_t *pid, float newFrequency);

// Integral component anti-windup gain
void pid_setAntiWindup(PidController_t *pid, float newKt);

// Setters for output limits
void pid_setLimits(PidController_t *pid, float newUpperLimit, float newLowerLimit);
void pid_setUpperLimit(PidController_t *pid, float);
void pid_setLowerLimit(PidController_t *pid, float);

// Dead zone and tolerance band setters
void pid_setDeadZone(PidController_t *pid, float);
void pid_setToleranceBand(PidController_t *pid, float);

// Resets all the components and previous error
void pid_reset(PidController_t *pid);

// Must be called with specified update period, returns true if still moving to set point
uint8_t pid_calculate(PidController_t *pid);
