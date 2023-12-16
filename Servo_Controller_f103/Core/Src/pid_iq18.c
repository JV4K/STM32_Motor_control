/*
 * pid_iq18.c
 *
 *  Created on: Dec 16, 2023
 *      Author: JV4K
 */

#include "pid_iq18.h"

// Must be called with specified update period
void pid_iq18_calculate(pid_iq18_t *pid, _iq18 setpoint, _iq18 feedback) {
	pid->error = setpoint - feedback;

	if ((pid->error > -(pid->toleranceBand))
			&& (pid->error < pid->toleranceBand)) {
		pid->error = 0;
		pid_iq18_reset(pid);
	}

	pid->P = _IQ18mpy(pid->error, pid->kp);
	pid->I += _IQ18mpy(
			(_IQ18mpy(pid->error, pid->ki)
					+ _IQ18mpy(pid->output - pid->rawOutput, pid->kb)),
			pid->dt);
	pid->D = _IQ18div(_IQ18mpy((pid->error - pid->previousError), pid->kd),
			pid->dt);

	pid->rawOutput = pid->P + pid->I + pid->D;
	pid->output = _IQsat(pid->rawOutput, pid->upperLimit, pid->lowerLimit);
	pid->previousError = pid->error;
}

// Resets all the components and previous error
void pid_iq18_reset(pid_iq18_t *pid) {
	pid->P = 0;
	pid->I = 0;
	pid->D = 0;
	pid->previousError = 0;
	pid->output = 0;
}

// Getter for output
_iq18 pid_iq18_getOutput(pid_iq18_t *pid) {
	return pid->output;
}

// Initialization with gains and function call frequency in herz
void pid_iq18_init(pid_iq18_t *pid, float Kp, float Ki, float Kd,
		float Frequency) {
	pid->kp = _IQ18(Kp);
	pid->ki = _IQ18(Ki);
	pid->kd = _IQ18(Kd);
	pid->dt = _IQ18(((float ) 1 / Frequency));
}

// Setter for gains
void pid_iq18_setGains(pid_iq18_t *pid, float Kp, float Ki, float Kd) {
	pid->kp = _IQ18(Kp);
	pid->ki = _IQ18(Ki);
	pid->kd = _IQ18(Kd);
}

// Integral component anti-windup gain
void pid_iq18_setAntiWindup(pid_iq18_t *pid, _iq18 Kb) {
	pid->kb = _IQ18(Kb);
}

// Setters for output limits
void pid_iq18_setLimits(pid_iq18_t *pid, float UpperLimit, float LowerLimit) {
	pid->lowerLimit = _IQ18(LowerLimit);
	pid->upperLimit = _IQ18(UpperLimit);
}

void pid_iq18_setUpperLimit(pid_iq18_t *pid, float UpperLimit) {
	pid->upperLimit = _IQ18(UpperLimit);
}

void pid_iq18_setLowerLimit(pid_iq18_t *pid, float LowerLimit) {
	pid->lowerLimit = _IQ18(LowerLimit);
}

// Tolerance band setter
void pid_iq18_setToleranceBand(pid_iq18_t *pid, float ToleranceBand) {
	pid->toleranceBand = _IQ18(ToleranceBand);
}
