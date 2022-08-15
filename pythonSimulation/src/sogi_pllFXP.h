/*
 * sogi_pllFXP.h
 *
 *  Created on: 23.07.2022
 *      Author: philip
 */

#ifndef INC_SOGI_PLLFXP_H_
#define INC_SOGI_PLLFXP_H_

#include <stdint.h>

typedef struct {
	int64_t x1;
	int64_t x2;
	int64_t x3;
} thirdOrderIntegr;


int16_t pll_singlephase_step(int32_t v);

void pll_set_phaseOffset(int16_t phaseOffset);

int32_t pll_get_w();
int32_t pll_get_va();
int32_t pll_get_vb();
int32_t pll_get_vd();
int32_t pll_get_vq();

#endif /* INC_SOGI_PLLFXP_H_ */
