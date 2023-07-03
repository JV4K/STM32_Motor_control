/*
 * servocontroller.h
 *
 *  Created on: Jun 29, 2023
 *      Author: JV4K
 */

#ifndef INC_SERVOCONTROLLER_H_
#define INC_SERVOCONTROLLER_H_

#endif /* INC_SERVOCONTROLLER_H_ */

#include <main.h>
#include <encoder.h>
#include <pid.h>
#include <pwm.h>

enum loops {
	Single, Double, Triple
};

enum servoMode {
	Idle, Position, Velocity
};

typedef struct {
	encoder_t encoder;
	pid_t pid_position;
	pid_t pid_velocity;
	pid_t pid_current;
	pwmControl_t driver;
	float positionSetpoint;
	float velocitySetpoint;
	float currentSetpoint;

	int8_t positionState;
	enum loops controllerLoops;
	enum servoMode currentMode;
} servocontrol_t;

void servo_baseInit(servocontrol_t *servo, enum loops servoLoops, uint16_t motorRpm, float gearRatio);
void servo_encoderInit(servocontrol_t *servo, TIM_HandleTypeDef *htim, uint16_t CPR);

void driver_init(servocontrol_t *servo, TIM_HandleTypeDef *htim, uint8_t timerChannel,
		GPIO_TypeDef *dir1_Port, uint32_t dir1_Pin, GPIO_TypeDef *dir2_Port, uint32_t dir2_Pin);

void servo_positionInit(servocontrol_t *servo, float kp, float ki, float kd, float dt);
void servo_velocityInit(servocontrol_t *servo, float kp, float ki, float kd, float dt);
void servo_currentInit(servocontrol_t *servo, float kp, float ki, float kd, float dt);

void servo_setPositionTolerance(servocontrol_t *servo, float tolerance);
int servo_getState(servocontrol_t *servo);

void servo_updatePosition(servocontrol_t *servo);
void servo_updateVelocity(servocontrol_t *servo);
void servo_updateCurrent(servocontrol_t *servo, float currentFeedback);

void servo_controlPosition(servocontrol_t *servo, float setpoint);
void servo_controlVelocity(servocontrol_t *servo, float setpoint);
