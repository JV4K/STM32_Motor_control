/*
 * servocontroller.c
 *
 *  Created on: Jun 29, 2023
 *      Author: JV4K
 */

#include <servocontroller.h>

void servo_baseInit(servocontrol_t *servo, enum loops servoLoops, uint16_t motorRpm, float gearRatio) {
	servo->controllerLoops = servoLoops;
	servo->encoder.gearRatio = gearRatio;
	servo->pid_velocity.lowerLimit = -(float) motorRpm / gearRatio;
	servo->pid_velocity.upperLimit = (float) motorRpm / gearRatio;
}
void servo_encoderInit(servocontrol_t *servo, TIM_HandleTypeDef *htim, uint16_t CPR) {
	servo->encoder.htim = htim;
	servo->encoder.countsPerRevolution = CPR;
}

void driver_init(servocontrol_t *servo, TIM_HandleTypeDef *htim, uint8_t timerChannel,
		GPIO_TypeDef *dir1_Port, uint32_t dir1_Pin, GPIO_TypeDef *dir2_Port, uint32_t dir2_Pin) {
	servo->driver.htim = htim;
	servo->driver.timerChannel = timerChannel;
	servo->driver.dir1_Port = dir1_Port;
	servo->driver.dir1_Pin = dir1_Pin;
	servo->driver.dir2_Port = dir2_Port;
	servo->driver.dir2_Pin = dir2_Pin;
}

void servo_positionInit(servocontrol_t *servo, float kp, float ki, float kd, float dt) {
	servo->pid_position.kp = kp;
	servo->pid_position.ki = ki;
	servo->pid_position.kd = kd;
	servo->pid_position.dt = dt;
}
void servo_velocityInit(servocontrol_t *servo, float kp, float ki, float kd, float dt) {
	servo->pid_velocity.kp = kp;
	servo->pid_velocity.ki = ki;
	servo->pid_velocity.kd = kd;
	servo->pid_velocity.dt = dt;

}
void servo_currentInit(servocontrol_t *servo, float kp, float ki, float kd, float dt) {
	servo->pid_current.kp = kp;
	servo->pid_current.ki = ki;
	servo->pid_current.kd = kd;
	servo->pid_current.dt = dt;
}

void servo_setPositionTolerance(servocontrol_t *servo, float tolerance) {
	servo->pid_position.toleranceBand = tolerance;
}

int servo_getState(servocontrol_t *servo) {
	if (servo->pid_position.error == 0) {
		return 0;
	} else {
		return 1;
	}
}

void servo_positionLoop(servocontrol_t *servo) {
	encoder_updatePosition(&servo->encoder);

	switch (servo->currentMode) {
	case Position: {
		pid_calculate(&servo->pid_position, servo->positionSetpoint, encoder_getAngle(&servo->encoder));
		if (servo->controllerLoops == Single) {
			pwm_setSpeed(&servo->driver, pid_getOutput(&servo->pid_position));
		} else {
			servo->velocitySetpoint = pid_getOutput(&servo->pid_position);
		}
		break;
	}
	default: {
		pid_reset(&servo->pid_position);
		servo->positionSetpoint = 0;
		break;
	}
	}
}

void servo_updateVelocity(servocontrol_t *servo) {
	encoder_updateVelocity(&servo->encoder);
	pid_calculate(&servo->pid_velocity, servo->velocitySetpoint, encoder_getVelocity(&servo->encoder));
	switch (servo->controllerLoops) {
	case Single:
		pid_reset(&servo->pid_velocity);
		servo->velocitySetpoint = 0;
		break;
	case Double:
		pwm_setSpeed(&servo->driver, pid_getOutput(&servo->pid_velocity));
		break;
	case Triple:
		servo->currentSetpoint = pid_getOutput(&servo->pid_velocity);
		break;
	}
}
void servo_updateCurrent(servocontrol_t *servo, float currentFeedback) {
	if (servo->controllerLoops == Triple) {
		pid_calculate(&servo->pid_current, servo->currentSetpoint, currentFeedback);
		pwm_setSpeed(&servo->driver, pid_getOutput(&servo->pid_current));
	} else {
		pid_reset(&servo->pid_current);
		servo->currentSetpoint = 0;
	}
}

void servo_controlPosition(servocontrol_t *servo, float setpoint) {
	servo->currentMode = Position;
	servo->positionSetpoint = setpoint;
}

void servo_controlVelocity(servocontrol_t *servo, float setpoint) {
	servo->currentMode = Velocity;
	servo->velocitySetpoint = setpoint;
}
