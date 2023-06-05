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

	if (v->ZeroDrift) { // If user enabled this feature
		if ((v->Err <= v->ZeroDrift) && (v->Err >= -(v->ZeroDrift))) {
			v->Err = 0;
		}
	}

	// Compute the proportional component
	if (v->Kp) {
		v->Up = v->Kp * v->Err;
	}

	// Compute the integral component
	if (v->Ki) {
		v->Ui =
				v->Ui
						+ ((float) (v->Out - v->OutPreSat) * v->Kt
								+ v->Err * v->DeltaT);
	}

	// Compute the differential component
	if (v->Kd) {
		v->Ud = (v->Err - v->PrevErr) / v->DeltaT;
	}

	// Compute the pre-saturated output
	v->OutPreSat = v->Up + v->Ui * v->Ki + v->Ud * v->Kd;

	// Saturation
	if (v->OutPreSat >= v->OutMax) {
		v->Out = v->OutMax;
	} else {
		if (v->OutPreSat <= v->OutMin) {
			v->Out = v->OutMin;
		} else {
			v->Out = v->OutPreSat;
		}
	}

	if (v->OutDeadZone) {
		if ((v->Out > 0) && (v->Out < v->OutDeadZone)) {
			v->Out = v->OutDeadZone;
		} else {
			if ((v->Out < 0) && (v->Out > -v->OutDeadZone)) {
				v->Out = -v->OutDeadZone;
			}
		}
	}

// Set currently used error value as previous
	v->PrevErr = v->Err;
}

void RegParamsUpd(volatile PIDREG *v, float kp, float ki, float kd, float dt,
		int32_t MaxOut, int32_t MinOut, float ZeroDrift, float DeadZone,
		float Antiwindup) {
	v->Kp = kp;
	v->Ki = ki;
	v->Kd = kd;
	v->DeltaT = dt;
	v->OutMax = MaxOut;
	v->OutMin = MinOut;
	v->ZeroDrift = ZeroDrift;
	v->OutDeadZone = DeadZone;
	v->Kt = Antiwindup;
}
