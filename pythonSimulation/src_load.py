#!/usr/bin/python3


class sourceDC:
    # simple capacitor model

    def __init__(self, C=6*1100e-6, T=1/10e3, V0=30):
        self.C = C
        self.T = T
        self.v = V0

    def step(self, i):
        self.v += 1/self.C * i*self.T
        return self.v


class loadRL:

    def __init__(self, L=200e-6, R=0.1, T=1/10e3, I0=0):
        self.L = L
        self.R = R
        self.T = T
        self.i = I0

    def step(self, v):
        self.i += (v-self.i*self.R)*self.T/self.L
        return self.i


