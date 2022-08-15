# Python framework for testing C algorithms

Python is used to simulate real world data in order to test inverter control algorithms and the Phase-Locked-Loop(PLL), which are written in C language.

# How to build

1. Compile C-Files to shared objects
``` 
cc -fPIC -shared -o controller.so src/controller.c src/sogi_pllFXP.c
```

2. Run python file
``` 
./inverterSimulation.py
``` 

