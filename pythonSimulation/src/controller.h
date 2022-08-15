/*
 * sogi_pllFXP.h
 *
 *  Created on: 23.07.2022
 *      Author: philip
 */

#ifndef INC_CONTROLLER_H_
#define INC_CONTROLLER_H_

#include <stdint.h>

#define ADC_BITS 12
int32_t get_ADC_BITS();

//#define VIN_ADCR (1.04 * 3.3 * 190/10)  // adc range in volts [0,62.7V]  30V real shows as 29.6V -> 1,0135; 25,2÷24,8=1,016
#define VIN_ADCR (1.055 * 3.3 * 190/10)  // adc range in volts ~[0,62.7V]
int32_t get_VIN_ADCR();

#define VIN_MIN 24.5  //25
#define VIN_MIN_RAW ((1<<ADC_BITS)*VIN_MIN/VIN_ADCR)
#define VIN_START 26.0  //32
#define VIN_START_RAW ((1<<ADC_BITS)*VIN_START/VIN_ADCR)
#define VDC_REF 29.0  // todo implement MPP Tracker
#define VDC_REF_RAW ((1<<ADC_BITS)*VDC_REF/VIN_ADCR)
#define VIN_CROWBAR_MIN 30.0
#define VIN_CROWBAR_MIN_RAW ((1<<ADC_BITS)*VIN_CROWBAR_MIN/VIN_ADCR)
#define VIN_CROWBAR_MAX 31.5
#define VIN_CROWBAR_MAX_RAW ((1<<ADC_BITS)*VIN_CROWBAR_MAX/VIN_ADCR)
#define VIN_MAX 32.5
#define VIN_MAX_RAW ((1<<ADC_BITS)*VIN_MAX/VIN_ADCR)

#define VGRID_AMP 325
#define VGRID_TRATIO 14  // transformer winding ratio
int32_t get_VGRID_TRATIO();
#define C (6*1100e-6)
float get_C();

//#define VGRID_ADCR (1.04 * 3.3 * 325/0.67)  // signed; additional transformer winding scales 325V down  // same 50Hz
#define VGRID_ADCR (1.056 * 3.3 * 325/0.67)  // signed; additional transformer winding scales 325V down  // same RMS
int32_t get_VGRID_ADCR();
#define VD_MIN_RAW ((1<<ADC_BITS)* 0.9 * VGRID_AMP/VGRID_ADCR)
#define VD_MAX_RAW ((1<<ADC_BITS)* 1.15 * VGRID_AMP/VGRID_ADCR)


#define SCALE_F2WRAW  ((1<<15) -1)
#define F_MIN 49
#define W_MIN_RAW F_MIN*SCALE_F2WRAW
#define F_MAX 51
#define W_MAX_RAW F_MAX*SCALE_F2WRAW

#define IGRID_ADCR 30  // [-15,+15A]; no grid current sensor is used at the moment
int32_t get_IGRID_ADCR();
#define IAC_AMP_MAX 8.5  //(6 * sqrt(2))  // ToDo 6,2Aamp sind in echt nur 4,2Aamp, bei 7 hat Automat ausgelöst -> Phase shift, dann okay, aber nur 50Wout -> 8A, okay -> 8.5A
#define IAC_AMP_MAX_RAW ((1<<ADC_BITS) * IAC_AMP_MAX/IGRID_ADCR)
#define I_REF_AMP_MIN 1.4  // todo fix when current scale fixed  1.2refamp -> 0.3W Einspeisung
#define I_REF_AMP_MIN_RAW ((1<<ADC_BITS) * I_REF_AMP_MIN/IGRID_ADCR)


typedef struct {
	int32_t y;
	int32_t y_min;
	int32_t y_max;
	int32_t x_prev;
	int32_t c1;  // (Ki*T/2 + Kp)
	int32_t c2;  // (Ki*T/2 - Kp)
} piController;


extern piController piCtrl;
void pi_step(int32_t x, piController *ctrl);

int16_t calc_IacAmp2VacSecAmpDCscale(int32_t i_amp);
int16_t get_IacPhase();
int32_t step_predict_i(int32_t i_ref, int32_t i);
int16_t step_pi_Vdc2IacAmp(int32_t vdc_ref, int32_t vdc);
int16_t step_pi_Vdc2IacAmp_volt_comp(int32_t vdc_ref, int32_t vdc, int16_t phase);
int16_t step_pi_Vdc2IacAmp_charge_comp(int32_t vdc_ref, int32_t vdc, int32_t vac_sec_VDCscale, int32_t i_ref_VDCscale);

int16_t sin1(int16_t angle);
int16_t cos1(int16_t angle);

#endif /* INC_CONTROLLER_H_ */
