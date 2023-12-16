/*
 * pid_iq18.h
 *
 *  Created on: Dec 16, 2023
 *      Author: JV4K
 */

#ifndef INC_PID_IQ18_H_
#define INC_PID_IQ18_H_

#include "main.h"
#include "IQmathLib.h"

typedef struct {
	_iq18 error;
	_iq18 kp;
	_iq18 ki;
	_iq18 kd;

	// If set, represents a gain of integral component anti-windup algorithm
	_iq18 kb;

	// If set, controller neglects error in range of [-toleranceBand; +toleranceBand]
	_iq18 toleranceBand;

	// PID components
	_iq18 P;
	_iq18 I;
	_iq18 D;

	// Sampling period in seconds
	_iq18 dt;

	// Output limits
	_iq18 upperLimit;
	_iq18 lowerLimit;

	_iq18 rawOutput; // No saturation
	_iq18 output; // Final saturated output signal

	_iq18 previousError;

} pid_iq18_t;

// Must be called with specified update period
void pid_iq18_calculate(pid_iq18_t *pid, _iq18 setpoint, _iq18 feedback);

// Resets all the components and previous error
void pid_iq18_reset(pid_iq18_t *pid);

// Getter for output
_iq18 pid_iq18_getOutput(pid_iq18_t *pid);

// Initialization with gains and function call frequency in herz
void pid_iq18_init(pid_iq18_t *pid, float Kp, float Ki, float Kd, float Frequency);

// Setter for gains
void pid_iq18_setGains(pid_iq18_t *pid, float Kp, float Ki, float Kd);

// Integral component anti-windup gain
void pid_iq18_setAntiWindup(pid_iq18_t *pid, _iq18 Kt);

// Setters for output limits
void pid_iq18_setLimits(pid_iq18_t *pid, float UpperLimit, float LowerLimit);
void pid_iq18_setUpperLimit(pid_iq18_t *pid, float UpperLimit);
void pid_iq18_setLowerLimit(pid_iq18_t *pid, float LowerLimit);

// Tolerance band setter
void pid_iq18_setToleranceBand(pid_iq18_t *pid, float ToleranceBand);


#endif /* INC_PID_IQ18_H_ */
