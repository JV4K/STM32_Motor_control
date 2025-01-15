#ifndef INC_ADC_CURRENT_H_
#define INC_ADC_CURRENT_H_

#include "tim.h"
#include "adc.h"
#include "dma.h"
#include "servo_iq18.h"
#include "settings.h"
#include "IQmathLib.h"
#include "ema_iq18.h"
#include "global_flags.h"

extern volatile uint16_t adc[2];
extern _iq18 current[2];
extern EMA_iq18 *filter1, *filter2;

void adc_cur_init();
void adc_callback_handler();

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);

#endif /* INC_ADC_CURRENT_H_ */