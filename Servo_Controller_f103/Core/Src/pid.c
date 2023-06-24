/*
 * pid.c
 *
 *  Created on: 31/03/2023
 *      Author: JV4K
 */

#include <pid.h>

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

void pid_calculate(pid_t *pid, float setpoint, float feedback) {
	pid->error = setpoint - feedback;

	if ((pid->error > -(pid->toleranceBand)) && (pid->error < pid->toleranceBand)) {
		pid->error = 0;
		pid_reset(pid);
	}

	//TODO proper calculations
	if (pid->error) {
		pid->P = pid->error * pid->kp;
		pid->I += (pid->output - pid->rawOutput) * pid->kt + pid->error * pid->dt;
		pid->D = (pid->error - pid->previousError) / pid->dt;

		pid->rawOutput = pid->P + pid->I + pid->D;
		pid->output = constrain(pid->rawOutput, pid->lowerLimit, pid->upperLimit);

		pid->previousError = pid->error;
	}
}

void pid_reset(pid_t *pid) {
	pid->P = 0;
	pid->I = 0;
	pid->D = 0;
	pid->previousError = 0;
	pid->output = 0;
}

void pid_init(pid_t *pid, float newKp, float newKi, float newKd, float newDt) {
}

void pid_setGains(pid_t *pid, float newKp, float newKi, float newKd) {
}

void pid_setPeriod(pid_t *pid, float newDt) {
}

void pid_setFrequencyHz(pid_t *pid, float newFrequency) {
}

void pid_setAntiWindup(pid_t *pid, float newKt) {
}

void pid_setLimits(pid_t *pid, float newUpperLimit, float newLowerLimit) {
}

void pid_setUpperLimit(pid_t *pid, float newUpperLimit) {
}

void pid_setLowerLimit(pid_t *pid, float newLowerLimit) {
}

void pid_setDeadZone(pid_t *pid, float newDeadZone) {
}

void pid_setToleranceBand(pid_t *pid, float newToleranceBand) {
}
