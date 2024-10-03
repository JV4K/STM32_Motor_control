/*
 * servo_iq18.c
 *
 *  Created on: Dec 18, 2023
 *      Author: zhmis
 */
#include "servo_iq18.h"

void servo_iq18_base_init(servo_iq18_t *servo, enum loops servoLoops,
		float motorSpeed, float gearRatio, uint8_t reverse) {
	servo->controllerLoops = servoLoops;
	servo->encoder.gearRatio = _IQ18(gearRatio);
	if (reverse) {
		servo->reverseFlag = -1;
	} else {
		servo->reverseFlag = 1;
	}
	servo->maxShaftSpeed = _IQ18(motorSpeed / gearRatio);

	if (servoLoops != Single) {
		servo->pid_position.lowerLimit = -servo->maxShaftSpeed;
		servo->pid_position.upperLimit = servo->maxShaftSpeed;
	}
}

void servo_iq18_encoder_init(servo_iq18_t *servo, TIM_HandleTypeDef *htim,
		uint16_t CPR, uint8_t encoder_is_reversed) {
	servo->encoder.htim = htim;
	servo->encoder.countsPerRevolution = CPR;
	servo->encoder.filter = initEMA_iq18(VEL_EMA, 0);
	if(!encoder_is_reversed){
		servo->encoder.reverse_flag = 1;
	}else{
		servo->encoder.reverse_flag = -1;
	}
}

void servo_iq18_driver_init(servo_iq18_t *servo, TIM_HandleTypeDef *htim,
		uint8_t timerChannel, GPIO_TypeDef *dir1_Port, uint32_t dir1_Pin,
		GPIO_TypeDef *dir2_Port, uint32_t dir2_Pin, uint16_t minDuty,
		uint16_t maxDuty) {
	servo->driver.htim = htim;
	servo->driver.timerChannel = timerChannel;
	servo->driver.dir1_Port = dir1_Port;
	servo->driver.dir1_Pin = dir1_Pin;
	servo->driver.dir2_Port = dir2_Port;
	servo->driver.dir2_Pin = dir2_Pin;

	servo->driver.minDuty = minDuty;
	servo->driver.maxDuty = maxDuty;

	switch (servo->controllerLoops) {
	case Single:
		servo->pid_position.lowerLimit = _IQ18(-1);
		servo->pid_position.upperLimit = _IQ18(1);
		break;
	case Double:
		servo->pid_velocity.lowerLimit = _IQ18(-1);
		servo->pid_velocity.upperLimit = _IQ18(1);
		break;
	case Triple:
		servo->pid_current.lowerLimit = _IQ18(-1);
		servo->pid_current.upperLimit = _IQ18(1);
		break;
	default:
		break;
	}
}

void servo_iq18_position_init(servo_iq18_t *servo, float kp, float ki, float kd,
		float dt, float kb) {
	servo->pid_position.kp = _IQ18(kp);
	servo->pid_position.ki = _IQ18(ki);
	servo->pid_position.kd = _IQ18(kd);
	servo->pid_position.dt = _IQ18(dt);
	servo->pid_position.kb = _IQ18(kb);
}

void servo_iq18_velocity_init(servo_iq18_t *servo, float kp, float ki, float kd,
		float dt, float kb) {
	servo->pid_velocity.kp = _IQ18(kp);
	servo->pid_velocity.ki = _IQ18(ki);
	servo->pid_velocity.kd = _IQ18(kd);
	servo->pid_velocity.dt = _IQ18(dt);
	servo->pid_velocity.kb = _IQ18(kb);
	servo->encoder.dt = _IQ18(dt);
}

void servo_iq18_current_init(servo_iq18_t *servo, float ratedCurrent, float kp,
		float ki, float kd, float dt, float kb) {
	servo->pid_current.kp = _IQ18(kp);
	servo->pid_current.ki = _IQ18(ki);
	servo->pid_current.kd = _IQ18(kd);
	servo->pid_current.dt = _IQ18(dt);
	servo->pid_current.kb = _IQ18(kb);
	servo->maxCurrent = _IQ18(ratedCurrent);
	if (servo->controllerLoops == Triple) {
		servo->pid_velocity.lowerLimit = _IQ18(-ratedCurrent);
		servo->pid_velocity.upperLimit = _IQ18(ratedCurrent);
	}
}

void servo_iq18_setPositionTolerance(servo_iq18_t *servo, float tolerance) {
	servo->pid_position.toleranceBand = _IQ18(tolerance);
}

int servo_iq18_getState(servo_iq18_t *servo) {
	if (servo->pid_position.error == 0) {
		return 0;
	}
	return 1;
}
int servo_iq18_getDirection(servo_iq18_t *servo) {
	if (servo->encoder.angularVelocity > 0) {
		return 1 * servo->reverseFlag;
	}
	if (servo->encoder.angularVelocity < 0) {
		return -1 * servo->reverseFlag;
	}
	return 0;
}

int servo_iq18_getCurrentDirection(servo_iq18_t *servo) {
	return servo->driver.dir;
}

void servo_iq18_positionLoop(servo_iq18_t *servo) {
//	encoder_updatePosition_iq18(&servo->encoder);

	switch (servo->currentMode) {
	case Position: {
		pid_iq18_calculate(&servo->pid_position, servo->positionSetpoint,
				encoder_getAngle_iq18(&servo->encoder));
		if (servo->controllerLoops == Single) {
			int32_t duty = (int32_t) (((int64_t) pid_iq18_getOutput(
					&servo->pid_position) * servo->driver.maxDuty) >> 18);
			pwm_setSpeed(&servo->driver, duty);
		} else {
			servo->velocitySetpoint = pid_iq18_getOutput(&servo->pid_position);
		}
		break;
	}
	default: {
		pid_iq18_reset(&servo->pid_position);
		servo->positionSetpoint = 0;
		break;
	}
	}
}

void servo_iq18_velocityLoop(servo_iq18_t *servo) {
	encoder_updatePosition_iq18(&servo->encoder);
	encoder_updateVelocity_qep_iq18(&servo->encoder);
	if (servo->currentMode != Current) {
		pid_iq18_calculate(&servo->pid_velocity, servo->velocitySetpoint,
				encoder_getVelocity_iq18(&servo->encoder));
	}

	switch (servo->controllerLoops) {
	case Single:
		pid_iq18_reset(&servo->pid_velocity);
		servo->velocitySetpoint = 0;
		break;
	case Double:
		int32_t duty = (int32_t) (((int64_t) pid_iq18_getOutput(
				&servo->pid_velocity) * servo->driver.maxDuty) >> 18);
		pwm_setSpeed(&servo->driver, duty);
		break;
	case Triple:
		if (servo->currentMode != Current) {
			servo->currentSetpoint = pid_iq18_getOutput(&servo->pid_velocity);
		} else {
			pid_iq18_reset(&servo->pid_velocity);
			servo->velocitySetpoint = 0;
		}
		break;
	}
}

void servo_iq18_currentLoop(servo_iq18_t *servo, _iq18 currentFeedback) {
	if (servo->controllerLoops == Triple) {
		pid_iq18_calculate(&servo->pid_current, servo->currentSetpoint,
				currentFeedback);
		int32_t duty = (int32_t) (((int64_t) pid_iq18_getOutput(
				&servo->pid_current) * servo->driver.maxDuty) >> 18);
		pwm_setSpeed(&servo->driver, duty);
	} else {
		pid_iq18_reset(&servo->pid_current);
		servo->currentSetpoint = 0;
	}
}

void servo_iq18_controlPosition(servo_iq18_t *servo, float setpoint) {
	servo->currentMode = Position;
	servo->positionSetpoint = _IQ18(setpoint) * servo->reverseFlag;
}

void servo_iq18_controlVelocity(servo_iq18_t *servo, float setpoint) {
	if (servo->controllerLoops != Single) {
		servo->currentMode = Velocity;
		servo->velocitySetpoint = _IQsat(_IQ18(setpoint) * servo->reverseFlag,
				servo->maxShaftSpeed, -servo->maxShaftSpeed);
	}
}

void servo_iq18_controlCurrent(servo_iq18_t *servo, float setpoint) {
	if (servo->controllerLoops == Triple) {
		servo->currentMode = Current;
		servo->currentSetpoint = _IQsat(_IQ18(setpoint * servo->reverseFlag),
				servo->maxCurrent, -servo->maxCurrent);
	}
}

void servo_iq18_reset(servo_iq18_t *servo){
	encoder_reset_iq18(&servo->encoder);
	pid_iq18_reset(&servo->pid_current);
	pid_iq18_reset(&servo->pid_velocity);
	pid_iq18_reset(&servo->pid_position);
}