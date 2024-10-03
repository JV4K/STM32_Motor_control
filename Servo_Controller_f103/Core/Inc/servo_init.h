#ifndef INC_SERVO_INIT_H_
#define INC_SERVO_INIT_H_

// #include "main.h"
#include "tim.h"
#include "adc.h"
#include "servo_iq18.h"
#include "settings.h"

extern servo_iq18_t servo1_g, servo2_g;

void servo_periph_init();
void servo_init();


#endif /* INC_SERVO_INIT_H_ */