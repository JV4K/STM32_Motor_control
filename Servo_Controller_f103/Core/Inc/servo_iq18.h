/*
 * servo_iq18.h
 *
 *  Created on: Dec 18, 2023
 *      Author: zhmis
 */

#ifndef INC_SERVO_IQ18_H_
#define INC_SERVO_IQ18_H_

#include <main.h>
#include "IQmathLib.h"
#include "encoder_iq18.h"
#include "pid_iq18.h"
#include "pwm.h"

#define VEL_EMA 0.4 // Smoothing factor of velocity filter

enum loops {
	Single, Double, Triple
};

enum servoMode {
	Idle, Position, Velocity, Current
};

/*
 Servo controller structure
 */
typedef struct {
	encoder_iq18_t encoder;
	pid_iq18_t pid_position;
	pid_iq18_t pid_velocity;
	pid_iq18_t pid_current;
	pwmControl_t driver;
	int8_t reverseFlag;

	int8_t encoder_reverse;
	int8_t current_reverse;

	_iq18 positionSetpoint;
	_iq18 velocitySetpoint;
	_iq18 currentSetpoint;

	int8_t positionState;
	enum loops controllerLoops;
	enum servoMode currentMode;
	_iq18 maxShaftSpeed;
	_iq18 maxCurrent;
} servo_iq18_t;

void servo_iq18_base_init(servo_iq18_t *servo, enum loops servoLoops,
		float motorSpeed, float gearRatio, uint8_t reverse);
void servo_iq18_encoder_init(servo_iq18_t *servo, TIM_HandleTypeDef *htim,
		uint16_t CPR, uint8_t encoder_is_reversed);

void servo_iq18_driver_init(servo_iq18_t *servo, TIM_HandleTypeDef *htim,
		uint8_t timerChannel, GPIO_TypeDef *dir1_Port, uint32_t dir1_Pin,
		GPIO_TypeDef *dir2_Port, uint32_t dir2_Pin, uint16_t minDuty,
		uint16_t maxDuty);

void servo_iq18_position_init(servo_iq18_t *servo, float kp, float ki, float kd,
		float dt, float kb);
void servo_iq18_velocity_init(servo_iq18_t *servo, float kp, float ki, float kd,
		float dt, float kb);
void servo_iq18_current_init(servo_iq18_t *servo, float ratedCurrent, float kp,
		float ki, float kd, float dt, float kb);

void servo_iq18_setPositionTolerance(servo_iq18_t *servo, float tolerance);
int servo_iq18_getState(servo_iq18_t *servo);
int servo_iq18_getDirection(servo_iq18_t *servo);
int servo_iq18_getCurrentDirection(servo_iq18_t *servo);

void servo_iq18_positionLoop(servo_iq18_t *servo);
void servo_iq18_velocityLoop(servo_iq18_t *servo);
void servo_iq18_currentLoop(servo_iq18_t *servo, _iq18 currentFeedback);

void servo_iq18_controlPosition(servo_iq18_t *servo, float setpoint);
void servo_iq18_controlVelocity(servo_iq18_t *servo, int16_t setpoint_q9);
void servo_iq18_controlCurrent(servo_iq18_t *servo, float setpoint);

void servo_iq18_reset(servo_iq18_t *servo);


#endif /* INC_SERVO_IQ18_H_ */
