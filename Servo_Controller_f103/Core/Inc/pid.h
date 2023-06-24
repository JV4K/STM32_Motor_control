/*
 * pid.h
 *
 *  Created on: 31/03/2023
 *      Author: JV4K
 */

#ifndef INC_REG_H_
#define INC_REG_H_

#endif /* INC_REG_H_ */

#include "main.h"

typedef struct {
	float error;
	float kp;
	float ki;
	float kd;

	// If set, represents a gain of integral component anti-windup algorithm
	float kt;

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

	// Output limits
	float upperLimit;
	float lowerLimit;

	float rawOutput; // No saturation
	float output; // Final Output signal

	float previousError;
} pid_t;

// Initialization with gains and update period
void pid_init(pid_t *pid, float newKp, float newKi, float newKd, float newDt);

// Setters for gains and update period (frequency)
void pid_setGains(pid_t *pid, float newKp, float newKi, float newKd);
void pid_setPeriod(pid_t *pid, float newDt); // Sets update period in seconds
void pid_setFrequencyHz(pid_t *pid, float newFrequency);

// Integral component anti-windup gain
void pid_setAntiWindup(pid_t *pid, float newKt);

// Setters for output limits
void pid_setLimits(pid_t *pid, float newUpperLimit, float newLowerLimit);
void pid_setUpperLimit(pid_t *pid, float newUpperLimit);
void pid_setLowerLimit(pid_t *pid, float newLowerLimit);

// Dead zone and tolerance band setters
void pid_setDeadZone(pid_t *pid, float newDeadZone);
void pid_setToleranceBand(pid_t *pid, float newToleranceBand);

// Resets all the components and previous error
void pid_reset(pid_t *pid);

// Must be called with specified update period, returns true if still moving to set point
void pid_calculate(pid_t *pid, float setpoint, float feedback);
