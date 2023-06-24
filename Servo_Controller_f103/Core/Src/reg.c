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
void pid_reg_reset(volatile PidController_t *v) {
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
void pid_reg_calc(volatile PidController_t *v) {
//	 Compute the error
//	if (v->Ref != v->PrevRef){
//		pid_reg_reset(v);
//	}
//	v->PrevRef = v->Ref;
	v->error = v->setpoint - v->feedback;

	if (v->ZeroDrift) { // If user enabled this feature
		if ((v->error <= v->ZeroDrift) && (v->error >= -(v->ZeroDrift))) {
			v->error = 0;
		}
	}

	// Compute the proportional component
	if (v->Kpt) {
		v->Up = v->Kpt * v->error;
	}

	// Compute the integral component
	if (v->ki) {
		v->Ui =
				v->Ui
						+ ((float) (v->Out - v->OutPreSat) * v->Kt
								+ v->error * v->DeltaT);
	}

	// Compute the differential component
	if (v->kd) {
		v->Ud = (v->error - v->PrevErr) / v->DeltaT;
	}

	// Compute the pre-saturated output
	v->OutPreSat = v->Up + v->Ui * v->ki + v->Ud * v->kd;

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
	v->PrevErr = v->error;
}

void RegParamsUpd(volatile PidController_t *v, float kp, float ki, float kd, float dt,
		int32_t MaxOut, int32_t MinOut, float ZeroDrift, float DeadZone,
		float Antiwindup) {
	v->Kpt = kp;
	v->ki = ki;
	v->kd = kd;
	v->DeltaT = dt;
	v->OutMax = MaxOut;
	v->OutMin = MinOut;
	v->ZeroDrift = ZeroDrift;
	v->OutDeadZone = DeadZone;
	v->Kt = Antiwindup;
}
