#ifndef INC_LOOPS_HANDLER_H_
#define INC_LOOPS_HANDLER_H_

#include "tim.h"
#include "IQmathLib.h"
#include "main_init.h"
#include "spi_slave_handler.h"

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#endif /* INC_LOOPS_HANDLER_H_ */