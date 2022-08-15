#!/usr/bin/python3
# quarter sine lookup table generator
import numpy as np


AMP = 2**15 -1
N = 128

for i in range(N+1):
    if i%10 == 0:
        print()
    # print( "0x{:04x}, ".format(int( AMP * np.sin( i/N * 2*np.pi/4 ) )), end='')
    print( "{:d}, ".format(int( AMP * np.sin( i/N * 2*np.pi/4 ) )), end='')

   