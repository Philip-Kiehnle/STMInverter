#include <stdint.h>
#include <math.h>
#include "controller.h"

#define FGRID 50
#define L (2*139e-6 + 4.7e-6 + 7.65e-6)  // 2 x large choke + small choke + transformer
#define ZL (2*M_PI*FGRID*L)
#define R (2*0.1 + 0.077)  // 2 x large choke + transformer
#define T (1/10e3)

#define EXTEND 30

const int L_EXT = L * (1<<EXTEND);
const int R_EXT = R * (1<<EXTEND);

int32_t get_VGRID_TRATIO()
{
	return VGRID_TRATIO;
}

float get_C()
{
	return C;
}

int32_t get_ADC_BITS()
{
	return ADC_BITS;
}

int32_t get_VIN_ADCR()
{
	return VIN_ADCR;
}

int32_t get_VGRID_ADCR()
{
	return VGRID_ADCR;
}

int32_t get_IGRID_ADCR()
{
	return IGRID_ADCR;
}

// generic PI controller:
// tustin approximation: s = 2/T * (z-1)/(z+1)
// y(k) = y(k-1) + Kp * (x(k)-x(k-1)) + Ki * T/2 * (x(k)+x(k-1))
// y(k) = y(k-1) + x(k) * (Ki*T/2 + Kp) + x(k-1) * (Ki*T/2 - Kp)
void pi_step(int32_t x, piController *ctrl)
{
	ctrl->y += x * ctrl->c1 + ctrl->x_prev * ctrl->c2;
	if (ctrl->y > ctrl->y_max) {
		ctrl->y = ctrl->y_max;
	} else if (ctrl->y < ctrl->y_min) {
		ctrl->y = ctrl->y_min;
	}
	ctrl->x_prev = x;
}


// PI voltage controller for DC-link capacitor
//
// model predictive current controller as PT1 element
//#define Tsigma (3*T)
#define Tsigma (3*T + 0.005)  // instead of 100Hz filter, increase Tsigma
#define EXTEND_PI_VDC 12

// Go(s) = 1/C * 1/s * (Kp+Ki/s) * 1/(1+Tsigma*s)
// symmetric optimum:
#define a 2.0  // a>1; typical value is 2; 
#define VR (C / (Tsigma*a))
const int Kp_Vdc = VR * (1<<EXTEND_PI_VDC);
#define Tn (a*a * Tsigma)
const int Ki_Vdc = VR/Tn * (1<<EXTEND_PI_VDC);

volatile int32_t debug_vdc_comp = 0;

int16_t get_IacPhase()
{
	return (int16_t)( (1<<15) * atan(ZL/R) / (2*M_PI));  // calculated at compile time
}

int16_t calc_IacAmp2VacSecAmpDCscale(int32_t i_amp)  // returns amplitude at secondary side
{
	#define Ztot sqrt(R*R + ZL*ZL)
	//const int32_t coeff = Ztot*(1<<14)*IGRID_ADCR/(VGRID_ADCR/VGRID_TRATIO);  // AC scale
	const int32_t coeff = Ztot*(1<<14)*IGRID_ADCR/VIN_ADCR;  // DC scale
	return ( (coeff*i_amp)>>14 );
}

// scale i_dc to i_ac_rms to i_ac_amp
piController piCtrl = { .y=0, .y_min=0, .y_max=IAC_AMP_MAX_RAW*(1<<EXTEND_PI_VDC), .x_prev=0,
								.c1=VIN_ADCR/IGRID_ADCR*sqrt(2)*(Ki_Vdc*T/2 + Kp_Vdc),
								.c2=VIN_ADCR/IGRID_ADCR*sqrt(2)*(Ki_Vdc*T/2 - Kp_Vdc) };


int16_t step_pi_Vdc2IacAmp(int32_t vdc_ref, int32_t vdc)
{
	pi_step((vdc-vdc_ref), &piCtrl);  // no ripple comp

	return (piCtrl.y>>EXTEND_PI_VDC);
}

int16_t step_pi_Vdc2IacAmp_volt_comp(int32_t vdc_ref, int32_t vdc, int16_t phase)
{
	// V2: simple 100 Hz capacitor ripple compensation in steady state
	// scale everything to Vdc
#define EXTEND_FXP_V2 5
#define VAC_AMP_SEC ( (uint16_t)(sqrt(2) * 230/VGRID_TRATIO * (1<<ADC_BITS)/VIN_ADCR) )
#define COEF ( (int32_t)( VAC_AMP_SEC * (1<<EXTEND_FXP_V2)/(C*VDC_REF_RAW*4*2*M_PI*FGRID)) )
	int32_t i_ref_amp = (piCtrl.y*(uint32_t)((IGRID_ADCR*(1<<5))/VIN_ADCR)) >> (EXTEND_PI_VDC+5);

	int32_t vc_ac = COEF * i_ref_amp * sin1((int16_t)(2*phase));

	int32_t vdc_comp = vdc + (vc_ac>>(EXTEND_FXP_V2+15+1)); //+1 for testing, ToDo: remove and check sign
	//debug_vdc_comp = vdc_comp;

	pi_step((vdc_comp-vdc_ref), &piCtrl);  // 100Hz ripple comp
	//pi_step((vdc-vdc_ref), &piCtrl);  // no ripple comp

	return (piCtrl.y>>EXTEND_PI_VDC);
}

int16_t step_pi_Vdc2IacAmp_charge_comp(int32_t vdc_ref, int32_t vdc, int32_t vac_sec_VDCscale, int32_t i_ref_VDCscale)
{
	// V3: dynamic 100 Hz capacitor ripple compensation without trigonometric functions
	// scale everything to Vdc
#define EXTEND_FXP 16
#define VAC_AMP_SEC ( (uint16_t)(sqrt(2) * 230/VGRID_TRATIO * (1<<ADC_BITS)/VIN_ADCR) )
#define C_FIXP ( (uint32_t)(C * (1<<EXTEND_FXP)) )
#define T_FIXP ( (uint32_t)(T * (1<<EXTEND_FXP)) )
	int32_t i_ref_amp = (piCtrl.y*(uint32_t)((IGRID_ADCR*(1<<5))/VIN_ADCR)) >> (EXTEND_PI_VDC+5);
	int32_t p_pv = i_ref_amp*VAC_AMP_SEC/2;
	int32_t p_grid = vac_sec_VDCscale * i_ref_VDCscale;
	static int64_t q = 0;
	q = 63*q/64 + ( p_pv - p_grid) / vdc;
	int32_t vdc_comp = vdc - T_FIXP * (q/C_FIXP);
	debug_vdc_comp = vdc_comp;

	pi_step((vdc_comp-vdc_ref), &piCtrl);  // 100Hz ripple comp
	//pi_step((vdc-vdc_ref), &piCtrl);  // no ripple comp

	return (piCtrl.y>>EXTEND_PI_VDC);
}

int32_t get_vdc_comp()
{
	return debug_vdc_comp;
}


// predictive current controller for RL-load
//          _____        _____
//     ----|_____|------|_____|----
//    |       R            L       |
//    V                            |
//    |                            |
//    –                            –
//

// int32_t step_predict_i(int32_t i_ref, int32_t i)
// {
// 	int64_t v = (L_EXT/T)*((int64_t)i_ref-(int64_t)i) + R_EXT*(int64_t)i;
// 	return (int32_t)(v>>EXTEND);
// }


// *****************
// sine lookup table
// *****************

/*
 * The mask: all bit belonging to the table
 * are 1, the all above 0.
 */
//#define TABLE_BITS  (5)  //"5 bit" large table = 32+1 values. 
#define TABLE_BITS  (7)  //"7 bit" large table = 128+1 values. 
#define TABLE_SIZE  (1<<TABLE_BITS)
#define TABLE_MASK  (TABLE_SIZE-1)

/**
 * "5 bit" lookup table for the offsets. These are the sines for exactly
 * at 0deg, 11.25deg, 22.5deg etc. The values are from -1 to 1 in Q15.
 */
// static int16_t SIN90[TABLE_SIZE+1] = {
//   0x0000,0x0647,0x0c8b,0x12c7,0x18f8,0x1f19,0x2527,0x2b1e,
//   0x30fb,0x36b9,0x3c56,0x41cd,0x471c,0x4c3f,0x5133,0x55f4,
//   0x5a81,0x5ed6,0x62f1,0x66ce,0x6a6c,0x6dc9,0x70e1,0x73b5,
//   0x7640,0x7883,0x7a7c,0x7c29,0x7d89,0x7e9c,0x7f61,0x7fd7,
//   0x7fff
// };

// --> quarter wave sine lookup table -> 512 phase increments -> 9bit total phase
static int16_t SIN90[TABLE_SIZE+1] = {
0, 402, 804, 1206, 1607, 2009, 2410, 2811, 3211, 3611, 
4011, 4409, 4807, 5205, 5601, 5997, 6392, 6786, 7179, 7571, 
7961, 8351, 8739, 9126, 9511, 9895, 10278, 10659, 11038, 11416, 
11792, 12166, 12539, 12909, 13278, 13645, 14009, 14372, 14732, 15090, 
15446, 15799, 16150, 16499, 16845, 17189, 17530, 17868, 18204, 18537, 
18867, 19194, 19519, 19840, 20159, 20474, 20787, 21096, 21402, 21705, 
22004, 22301, 22594, 22883, 23169, 23452, 23731, 24006, 24278, 24546, 
24811, 25072, 25329, 25582, 25831, 26077, 26318, 26556, 26789, 27019, 
27244, 27466, 27683, 27896, 28105, 28309, 28510, 28706, 28897, 29085, 
29268, 29446, 29621, 29790, 29955, 30116, 30272, 30424, 30571, 30713, 
30851, 30984, 31113, 31236, 31356, 31470, 31580, 31684, 31785, 31880, 
31970, 32056, 32137, 32213, 32284, 32350, 32412, 32468, 32520, 32567, 
32609, 32646, 32678, 32705, 32727, 32744, 32757, 32764, 32767
};

/*
 * The lookup table is to 90DEG, the input can be -360 to 360 DEG, where negative
 * values are transformed to positive before further processing. We need two
 * additional bits (*4) to represent 360 DEG:
 */
#define LOOKUP_BITS (TABLE_BITS+2)
#define LOOKUP_MASK ((1<<LOOKUP_BITS)-1)
#define FLIP_BIT    (1<<TABLE_BITS)
#define NEGATE_BIT  (1<<(TABLE_BITS+1))
#define INTERP_BITS (16-1-LOOKUP_BITS)
#define INTERP_MASK ((1<<INTERP_BITS)-1)



/**
 * Sine calculation using interpolated table lookup.
 * Instead of radiants or degrees we use "turns" here. Means this
 * sine does NOT return one phase for 0 to 2*PI, but for 0 to 1.
 * Input: -1 to 1 as int16 Q15  == -32768 to 32767.
 * Output: -1 to 1 as int16 Q15 == -32768 to 32767.
 *
 * See the full description at www.AtWillys.de for the detailed
 * explanation.
 *
 * @param int16_t angle Q15
 * @return int16_t Q15
 */
int16_t sin1(int16_t angle)
{
  int16_t v0, v1;
  if (angle < 0) {
	angle += INT16_MAX;
	angle += 1;
  }
  v0 = (angle >> INTERP_BITS);
  if (v0 & FLIP_BIT) {
	v0 = ~v0;
	v1 = ~angle;
  } else {
	v1 = angle;
  }
  v0 &= TABLE_MASK;
  v1 = SIN90[v0] + (int16_t) (((int32_t) (SIN90[v0+1]-SIN90[v0]) * (v1 & INTERP_MASK)) >> INTERP_BITS);
  if((angle >> INTERP_BITS) & NEGATE_BIT) v1 = -v1;
  return v1;
}
 
/**
 * Cosine calculation using interpolated table lookup.
 * Instead of radiants or degrees we use "turns" here. Means this
 * cosine does NOT return one phase for 0 to 2*PI, but for 0 to 1.
 * Input: -1 to 1 as int16 Q15  == -32768 to 32767.
 * Output: -1 to 1 as int16 Q15 == -32768 to 32767.
 *
 * @param int16_t angle Q15
 * @return int16_t Q15
 */
int16_t cos1(int16_t angle)
{
  if (angle < 0) {
	angle += INT16_MAX;
	angle += 1;
  }
  return sin1(angle - (int16_t)(((int32_t)INT16_MAX * 270) / 360));
}
