#include <stdint.h>
#include <math.h>
#include "sogi_pllFXP.h"
#include "controller.h"


// internal use
void sogi_step(int32_t x);
void dq_step();

/* constants for PLL */
const int F_SAMPLE = 10e3;
const float T = 1/(float)F_SAMPLE;
// scale 2*PI to signed 16bit Phase
#define PHASE_MAX  INT16_MAX
#define SCALE_PHASE2RAW  ( PHASE_MAX / (2*M_PI) )


#define V_MAX 1000
#define V_GRID_AMP 325
// scaling PLL loop gain; 2*PI equals 15bit    vs 12bit signed voltage
//#define SCALE  (V_MAX/V_GRID_AMP * 2)
//#define SCALE  (1<<1)  // 14bit voltage 15bit phase
#define SCALE  (1<<3)  // 12bit voltage 15bit phase

const float KpFLT = 44.4288;  // 5Hz disturbance transfer function bandwidth; 10Hz transfer function bandwidth
const float KiFLT = 986.9604;  // 5Hz disturbance transfer function bandwidth; 10Hz transfer function bandwidth
#define LD_PI_EXTEND 8
const int32_t Kp = (1<<LD_PI_EXTEND) * SCALE * KpFLT;
const int32_t Ki = (1<<LD_PI_EXTEND) * SCALE * KiFLT;
const float wn = 50 * 2*M_PI;


/* Global Variables for PLL */
int32_t w = 0;
int16_t phi16 = 0;
int16_t phase16_offset = 0;


/* Global Variables for SOGI */
int32_t va = 0;
int32_t vb = 0;
#define LD_W_EXTEND 20
const int32_t wscale = (1<<LD_W_EXTEND) * T * wn;

/* Global Variables for DQ trafo */
int32_t vd = 0;
int32_t vq = 0;

/* Global Variables for PI controller */





// Three phase
//                 _______                     ___
//         w      |       | v1_a              |   |
//        ------> |       |-----------------> | + |
//                | SOGI1 |                   |   |          ________
//         v_a    |       | v1_b              |   |  va     |        |
//        ------> |       |----->     v2_b -> | - | ------> |        |
// v_ab  |        |_______|                   |___|         |        |
// ------|                                                  | DQ_PLL | -->
//       |         _______                     ___          |        |
//       | v_b    |       | v2_a              |   |  vb     |        |
//        ------> |       |-----------------> | + | ------> |        |
//                | SOGI2 |                   |   |         |________|
//         w      |       | v2_b              |   |
//        ------> |       |----->     v1_b -> | + |
//                |_______|                   |___|

//int32_t pll_threephase_step(int32_t v) {

// Single phase
//                 _______                 ________        _________
//         w      |       | va            |        |      |         |
//        ------> |       |-----> ------> |        |      |         |
//                | SOGI1 |               |        |  vq  | PI-ctrl | --> w_pictrl  ...---> phi
//         v      |       | vb            | DQ_PLL | ---> |         |
//        ------> |       |-----> ------> |        |      |_________|
//                |_______|               |________|             

int16_t pll_singlephase_step(int32_t v)
{

	const int32_t PI_Y_INIT = wn * SCALE_PHASE2RAW * (1<<LD_PI_EXTEND);
	piController piCtrl = { .y=PI_Y_INIT, .y_min=0.5*PI_Y_INIT, .y_max=1.5*PI_Y_INIT, .x_prev=0, .c1=(Ki*T/2 + Kp), .c2=(Ki*T/2 - Kp) };

	sogi_step(v);
	dq_step();
	pi_step(vq>>15, &piCtrl);

	int32_t w_pictrl = piCtrl.y;
	w = w_pictrl>>LD_PI_EXTEND;
#define LD_PHI_EXTEND 2
	static int32_t phi32 = 0;
	phi32 += (w_pictrl<<LD_PHI_EXTEND)/F_SAMPLE;
	phi16 = phi32>>(LD_PHI_EXTEND+LD_PI_EXTEND); 

	return (phi16+phase16_offset);
}

void pll_set_phaseOffset(int16_t phaseOffset)
{
	phase16_offset = phaseOffset;
}

int32_t pll_get_w()
{
	return w;
}

int32_t pll_get_va()
{
	return va;
}

int32_t pll_get_vb()
{
	return vb;
}

int32_t pll_get_vd()
{
	return vd>>15;
}

int32_t pll_get_vq()
{
	return vq>>15;
}


int32_t integrate(thirdOrderIntegr *integrState, int32_t x)
{
	// source: https://imperix.com/doc/implementation/sogi-pll

	// V2: works better with one delay step removed
	integrState->x3 = integrState->x2;
	integrState->x2 = integrState->x1;
	integrState->x1 += x / 12;  //  mul by T outside of this function

	int32_t y = (int32_t)( ((int64_t)( 23*integrState->x1 - 16*integrState->x2 + 5*integrState->x3 )) >> LD_W_EXTEND);

	// V1: original implementation according to source
	// integrState->x3 = integrState->x2;
	// integrState->x2 = integrState->x1;
	// integrState->x1 += x / 12;  //  mul by T outside of this function

	return y;
}


// SOGI

//          <-------------------------------------------------------------------------------
//       __|__         _______        _____             _____                               |
//      |     |       |       |      |     |  wscale-> |     |          _____       _____   |
//      |  -  |  x2   |       |----> |  +  |           |     |         |     |v_int|     |  |
//  x   |     |-----> | X 1.5 |      |     |---------> |  X  |-------> | 1/s |---->|z^-1 |-----> va
// ---> |  +  |       |       |      |  -  |           |     |         |_____|     |_____|  |
//      |_____|       |_______|      |_____|           |_____|                              |
//                                      ^                                                   |
//                                      |                             _____                 |
//                                      |              _____         |     |                |
//                                      |   vb_int    |     |        |     | <--------------
//                                      |-------------| 1/s | <----- |  X  |
//                                      |             |_____|        |     | <- wscale
//                                      |                            |_____|
//                                      |
//                                      |
//                                       ---------------------------------------> vb
//
//             using backward euler integration; backward euler approximation: s = (z-1)/(z*T)

void sogi_step(int32_t x)
{

	int32_t x2 = x - va;

	// simple integrator
	// static int64_t v_int = 0;
	// static int64_t vb_int = 0;
	// v_int += wscale * (1.5*x2 - vb);
	// vb_int += wscale * va;
	// va = v_int >> LD_W_EXTEND;
	// vb = vb_int >> LD_W_EXTEND;


	// third order integrator
	static thirdOrderIntegr integrStateVA = {}, integrStateVB = {};
	int32_t v_int = integrate( &integrStateVA, wscale * (1.5*x2 - vb) );
	vb = integrate( &integrStateVB, wscale * va );
	va = v_int;

}


void dq_step()
{
	vd = cos1(phi16)*va + sin1(phi16)*vb;
	vq = -sin1(phi16)*va + cos1(phi16)*vb;
}
