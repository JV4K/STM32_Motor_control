/*
 * encoder_assert.c
 *
 *  Created on: 01/04/2023
 *      Author: JV4K
 */

#include <encoder_assert.h>

void EncoderReset(volatile ENCODER *enc) {
	enc->CurRots = 0;
	enc->PrevTicks = 0;
	enc->Angle = 0;
	enc->htim->Instance->CNT = 0;
}

void EncoderInterrupt(volatile ENCODER *enc, int8_t Dir) {
	if (Dir)
		enc->CurRots--;
	else
		enc->CurRots++;
}

void EncoderPosition(volatile ENCODER *enc) {
	enc->CntValUint = enc->htim->Instance->CNT;
	enc->CntValInt = (int16_t) enc->CntValUint;
	if (enc->CntValInt < -enc->CPR) {
		enc->htim->Instance->CNT = 0;
		enc->CurRots--;
	} else {
		if (enc->CntValInt > enc->CPR) {
			enc->htim->Instance->CNT = 0;
			enc->CurRots++;
		}
	}

	// Get how many ticks happened from a starting point
	enc->CurTicks = (enc->CurRots * enc->CPR) + (enc->CntValInt);

	// Transform ticks to angle
	enc->Angle = ((float) (enc->CurTicks) / enc->CPR) * 360;
}

void EncoderVelocity(volatile ENCODER *enc) {
	// Get ticks delta
	enc->DeltTicks = enc->CurTicks - enc->PrevTicks;
	enc->PrevTicks = enc->CurTicks;

	// Compute angular velocity in RPM
	enc->AngVel = (((float) (enc->DeltTicks)) / (float) (enc->CPR))
			* (enc->SamplingFreq) * 60;
}

void EncoderSettings(volatile ENCODER *enc, TIM_HandleTypeDef *htim_new,
		int16_t CPR_new, float Freq_new) {
	enc->htim = htim_new;
	enc->CPR = CPR_new;
	enc->SamplingFreq = Freq_new;
}
