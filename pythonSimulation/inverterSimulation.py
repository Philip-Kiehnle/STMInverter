#!/usr/bin/python3
# simulation environment for photovoltaic grid inverter
import argparse
import os
import sys
import glob
import numpy as np
import matplotlib.pyplot as plt

from ctypes import *

import src_load

T = 1/10e3
fgrid = 50


def align_yaxis(ax1, ax2):
    ax1_ylims = ax1.axes.get_ylim()           # Find y-axis limits set by the plotter
    ax1_yratio = ax1_ylims[0] / ax1_ylims[1]  # Calculate ratio of lowest limit to highest limit

    ax2_ylims = ax2.axes.get_ylim()           # Find y-axis limits set by the plotter
    ax2_yratio = ax2_ylims[0] / ax2_ylims[1]  # Calculate ratio of lowest limit to highest limit


    # If the plot limits ratio of plot 1 is smaller than plot 2, the first data set has
    # a wider range range than the second data set. Calculate a new low limit for the
    # second data set to obtain a similar ratio to the first data set.
    # Else, do it the other way around

    if ax1_yratio < ax2_yratio: 
        ax2.set_ylim(bottom = ax2_ylims[1]*ax1_yratio)
    else:
        ax1.set_ylim(bottom = ax1_ylims[1]*ax2_yratio)

###########################################
## Proportional Resonant controller test ##
## -> Todo: try SWIG Cpp wrapper         ##
###########################################
# so_file = "./grid_control_pr.so"
# lib = CDLL(so_file)
# print(type(lib))
# grid_control = lib.init(T)


##########################
## SOGI-PLL common test ##
##########################
t = np.arange(0, 50/fgrid, T)
arg = 2*np.pi*t
arg_offset = 0*1*np.pi 
v_grid = 325 * np.sin(fgrid*arg+arg_offset) + 0 * np.sin(3*fgrid*arg+arg_offset) + 0 * np.sin(5*fgrid*arg+arg_offset)

N = len(v_grid)

v_grid[int(0.5*N):int(0.7*N)] *= 0.3  # voltage sag
v_grid_meas = v_grid #+ 10  # 10V offset in measurement

phi_pll = np.empty(N)
freq_pll = np.empty(N)
v_grid_estim = np.empty(N)
vab_pll = np.empty([2,N])
vdq_pll = np.empty([2,N])


#########################
## SOGI-PLL float test ##
#########################
# so_file = "./sogi_pllFLT.so"
# lib = CDLL(so_file)

# lib.pll_singlephase_step.restype = c_float
# lib.pll_get_w.restype = c_float
# lib.pll_get_va.restype = c_float
# lib.pll_get_vb.restype = c_float
# lib.pll_get_vd.restype = c_float
# lib.pll_get_vq.restype = c_float

# for i, v in enumerate(v_grid):
#     phi_pll[i] = lib.pll_singlephase_step(c_float(v))
#     freq_pll[i] = lib.pll_get_w()/(2*np.pi)
#     vab_pll[0][i] = lib.pll_get_va()
#     vab_pll[1][i] = lib.pll_get_vb()
#     vdq_pll[0][i] = lib.pll_get_vd()
#     vdq_pll[1][i] = lib.pll_get_vq()


############################
## SOGI-PLL fixpoint test ##
############################
#so_file = "./sogi_pllFXP.so"
so_file = "./controller.so"
ctrl = CDLL(so_file)
ctrl.get_C.restype = c_float
ADC_BITS = ctrl.get_ADC_BITS()
SCALE_VGRID2ADCRAW = (2**ADC_BITS -1) / ctrl.get_VGRID_ADCR()
SCALE_VIN2ADCRAW = (2**ADC_BITS -1) / ctrl.get_VIN_ADCR()
SCALE_I2ADCRAW = (2**ADC_BITS -1) / ctrl.get_IGRID_ADCR()
SCALE_PHASE2RAW  = ( ((2**15) -1) / (2*np.pi) )
SCALE_F2WRAW  = ((2**15) -1)
ctrl.pll_set_phaseOffset(-310)

for i, v in enumerate(v_grid_meas):
    phi_pll[i] = ctrl.pll_singlephase_step( int(SCALE_VGRID2ADCRAW*v) )/SCALE_PHASE2RAW
    freq_pll[i] = ctrl.pll_get_w()/SCALE_F2WRAW

    vab_pll[0][i] = ctrl.pll_get_va()/SCALE_VGRID2ADCRAW
    vab_pll[1][i] = ctrl.pll_get_vb()/SCALE_VGRID2ADCRAW
    vdq_pll[0][i] = ctrl.pll_get_vd()/SCALE_VGRID2ADCRAW
    vdq_pll[1][i] = ctrl.pll_get_vq()/SCALE_VGRID2ADCRAW

    v_grid_estim[i] = vdq_pll[0][i] * np.cos(phi_pll[i])

    # va_pll[i] = ctrl.sin1(int(i/N*2**15))/SCALE_VGRID2ADCRAW12BIT
    # vb_pll[i] = ctrl.sin1(int(i/N*2**15 + 2**14))/SCALE_VGRID2ADCRAW12BIT

##################################
## voltage & current controller ##
##################################
Vdc0 = 30
Vdc_ref = 29.0
Ipv = 1.9  # constant PV current; ToDo: implement PV LUT
sourceDC = src_load.sourceDC(T=T, V0=Vdc0)
v_dc = Vdc0 * np.ones(N)
v_dc_comp = np.zeros(N)
v_dc_comp_python = np.zeros(N)

i_dc = np.zeros(N)

loadRL = src_load.loadRL(L=(2*139e-6 + 4.7e-6 + 7.65e-6), R=(2*0.1 + 0.077), T=T)
i_ref = 3 * np.sin(fgrid*arg+arg_offset)
v_pred = np.zeros(N)
i_grid = np.zeros(N)


pll_locked = False
cnt_locked = 0
transformer_ratio = ctrl.get_VGRID_TRATIO()
Vac_rms_sec = 230/transformer_ratio
C = ctrl.get_C()

q = 0
vac_sec = 0
GRID = True

for i in range(len(v_grid)-1):
    if pll_locked:
        if 1:  # voltage and current control
#            i_ref_amp = ctrl.step_pi_Vdc2IacAmp( int( SCALE_VIN2ADCRAW*Vdc_ref), int( SCALE_VIN2ADCRAW*v_dc[i])) / SCALE_I2ADCRAW
            i_ref_amp = ctrl.step_pi_Vdc2IacAmp_volt_comp( int( SCALE_VIN2ADCRAW*Vdc_ref), int( SCALE_VIN2ADCRAW*v_dc[i]), int(phi_pll[i]*SCALE_PHASE2RAW) ) / SCALE_I2ADCRAW
#            i_ref_amp = ctrl.step_pi_Vdc2IacAmp_charge_comp( int( SCALE_VIN2ADCRAW*Vdc_ref), int( SCALE_VIN2ADCRAW*v_dc[i]),
#                                                             int( SCALE_VIN2ADCRAW*vac_sec ), int( SCALE_VIN2ADCRAW*i_ref[i] ) ) / SCALE_I2ADCRAW
            i_ref[i+1] = i_ref_amp * np.cos(phi_pll[i+1])

            if 0:  # with current sensor
                v_pred[i] = ctrl.step_predict_i(int( SCALE_I2ADCRAW*i_ref[i+1] ), int( SCALE_I2ADCRAW*i_grid[i] ) ) / SCALE_I2ADCRAW

            else:  # sensorless
                phase_shiftRL = ctrl.get_IacPhase()/SCALE_PHASE2RAW
                v_pred[i] = ctrl.calc_IacAmp2VacSecAmpDCscale(int( SCALE_I2ADCRAW*i_ref_amp)) / SCALE_VIN2ADCRAW * np.cos(phi_pll[i+1] +phase_shiftRL)

            v_dc_comp[i] = ctrl.get_vdc_comp()/ SCALE_VIN2ADCRAW

            # V1: 100 Hz capacitor ripple compensation in steady state
#            w = 2*np.pi*fgrid
#            t_ = phi_pll[i]/w
#            q = i_ref_amp/(2**.5)*Vac_rms_sec/Vdc_ref * t_ - Vac_rms_sec*2**.5*i_ref_amp/Vdc_ref * (-np.sin(w*t_) * np.cos(w*t_)/(2*w) + t_/2)
#            v_dc_comp_python[i+1] = v_dc[i] + q/C

            # V2: simple 100 Hz capacitor ripple compensation in steady state
            w = 2*np.pi*fgrid
            vc_ac = np.sin(2*phi_pll[i]) * Vac_rms_sec*2**.5 * i_ref_amp / (C*Vdc_ref * 4*w)
            v_dc_comp_python[i+1] = v_dc[i] + vc_ac

            # V3: dynamic 100 Hz capacitor ripple compensation without trigonometric functions
#            p_pv = i_ref_amp/(2**.5)*Vac_rms_sec
#            vac_sec = v_pred[i]*ctrl.get_VIN_ADCR()/(ctrl.get_VGRID_ADCR()/transformer_ratio) + v_grid[i]/transformer_ratio
#            p_grid = vac_sec * i_ref[i]
#            q = 63*q/64 + T * ( p_pv - p_grid) / v_dc[i]
#            v_dc_comp_python[i+1] = v_dc[i] - q/C

            i_dc[i] = v_grid[i]/transformer_ratio * i_grid[i] / v_dc[i]
            v_dc[i+1] = sourceDC.step(Ipv-i_dc[i])
            i_grid[i+1] = loadRL.step(v_pred[i] + GRID*(v_grid_estim[i] -v_grid[i])/transformer_ratio )
        else: # current control
            v_pred[i] = ctrl.step_predict_i(int( SCALE_I2ADCRAW*i_ref[i+1] ), int( SCALE_I2ADCRAW*i_grid[i] ) ) / SCALE_I2ADCRAW
            #print(FLT2FIXP*i_ref[i] , int( FLT2FIXP*i_grid[i-1] ))
            #print('v_pred', v_pred[i])
            i_grid[i+1] = loadRL.step(v_pred[i] + GRID*(v_grid_estim[i] -v_grid[i])/transformer_ratio )
    elif vdq_pll[0][i] > 300 and abs(vdq_pll[1][i]) < 15 :
        if cnt_locked == int(10e-3/T) :
            pll_locked = True
        else:
            cnt_locked += 1
    else:
        cnt_locked = 0

##################
## plot results ##
##################
fig, ax1 = plt.subplots()

ax1.plot(t, v_grid, label='v_grid')
ax1.plot(t, v_grid_meas, label='v_grid_meas')
ax1.plot(t, v_grid_estim, label='v_grid_estim')
# ax1.plot(t, vab_pll[0], label='va_pll')
# ax1.plot(t, vab_pll[1], label='vb_pll')
# ax1.plot(t, vdq_pll[0], label='vd_pll')
# ax1.plot(t, vdq_pll[1], label='vq_pll')
ax1.plot(t, v_pred, label='v_pred')
ax1.plot(t, v_dc, label='v_dc')
ax1.plot(t, v_dc_comp, label='v_dc_comp')
ax1.plot(t, v_dc_comp_python, label='v_dc_comp_python')


ax2 = ax1.twinx() 
ax2._get_lines.prop_cycler = ax1._get_lines.prop_cycler

ax2.plot(t, phi_pll, label='phi_pll')
ax2.plot(t, freq_pll, label='freq_pll')

ax2.plot(t, i_ref, label='i_ref')
ax2.plot(t, i_grid, label='i_grid')
ax2.plot(t, i_dc, label='i_dc')

ax1.legend(loc='upper left')
ax2.legend(loc='upper right')

align_yaxis(ax1, ax2)

### FFT
#from scipy.fft import fft, fftfreq
#SAMPLE_RATE = 1/T
#yf = fft(i_grid)
#yf = 20*np.log10(yf)
#xf = fftfreq(len(i_grid), 1 / SAMPLE_RATE)

#plt.figure(2)
#plt.plot(xf, yf-np.max((yf)), label='i_grid')
#plt.legend()

plt.show()
