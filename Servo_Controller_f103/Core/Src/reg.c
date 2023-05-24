/*
 * reg.c
 *
 *  Created on: 31/03/2023
 *      Author: JV4K
 */

#include "reg.h"

/*
 * Use this when setting a new reference in order to
 * ensure that zero conditions are set.
 */
void pid_reg_reset(volatile PIDREG *v) {
//	v->Ref = 0;
//	v->Fdb = 0;
//	v->Err = 0;
//	v->Up = 0;
//	v->Out = 0;
	v->Ui = 0;
//	v->PrevErr = 0;
////	v->SatErr = 0;
//	v->OutPreSat = 0;
}
/*
 * This function calculates output signal.
 */
void pid_reg_calc(volatile PIDREG *v) {
//	 Compute the error
//	if (v->Ref != v->PrevRef){
//		pid_reg_reset(v);
//	}
//	v->PrevRef = v->Ref;
	v->Err = v->Ref - v->Fdb;

	if (v->ShrinkFlag) { // If user enabled this feature
		if ((v->Err <= v->ErrThreshold) && (v->Err >= -(v->ErrThreshold))) {
			v->Err = 0;
		}
	}

	// Compute the proportional component
	if (v->Kp) {
		v->Up = v->Kp * v->Err;
	}

	// Compute the integral component
	if (v->Ki) {
		if (!(v->InSaturationFlag)) {
			v->Ui = v->Ui + v->Err * v->DeltaT;
			//v->Ui = v->Ui + v->Err * v->DeltaT - v->KIntegralAntiWindUp*v->SatErr;
		}
	}

	// Compute the differential component
	if (v->Kd) {
		v->Ud = (v->Err - v->PrevErr) / v->DeltaT;
	}

	// Compute the pre-saturated output
	v->OutPreSat = v->Up + v->Ui * v->Ki + v->Ud * v->Kd;

	// Saturation
	if (v->OutPreSat >= v->OutMax) {
		//v->SatErr = v->OutPreSat - v->OutMax;
		v->Out = v->OutMax;
		v->InSaturationFlag = 1;
	} else {
		if (v->OutPreSat <= v->OutMin) {
			//v->SatErr = v->OutPreSat - v->OutMin;
			v->Out = v->OutMin;
			v->InSaturationFlag = 1;
		} else {
			//v->SatErr = 0;
			v->Out = v->OutPreSat;
			v->InSaturationFlag = 0;
		}
	}

	// Set currently used error value as previous
	v->PrevErr = v->Err;
}

void RegParamsUpd(volatile PIDREG *v, float kp, float ki, float kd, float dt,
		int32_t MaxOut, int32_t MinOut, int8_t flag, float Threshold) {
	v->Kp = kp;
	v->Ki = ki;
	v->Kd = kd;
	v->DeltaT = dt;
	v->OutMax = MaxOut;
	v->OutMin = MinOut;
	v->ErrThreshold = Threshold;
	v->ShrinkFlag = flag;
}
