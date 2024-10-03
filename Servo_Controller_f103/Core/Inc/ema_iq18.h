/*
 * ema_iq18.h
 *
 *  Created on: Dec 18, 2023
 *      Author: zhmis
 */

#ifndef INC_EMA_IQ18_H_
#define INC_EMA_IQ18_H_
#include "IQmathLib.h"
typedef struct {
	_iq18 alpha;    // Weight factor (0 < alpha < 1)
	_iq18 previous; // Previous EMA value
} EMA_iq18;

// Initialize the EMA filter
EMA_iq18* initEMA_iq18(float alpha, _iq18 initial_value);

// Update the EMA filter with a new value and return the filtered result
_iq18 updateEMA_iq18(EMA_iq18 *filter, _iq18 newValue);

#endif /* INC_EMA_IQ18_H_ */
