#!/usr/bin/env python3

import numpy as np
import math
import itertools
import os

X_RANGE = [0.0, 30.0]
Y_RANGE = [0.0, 30.0]
TH_RANGE = [-math.pi, math.pi]  # (radians)

DX = 0.5  # (m)
DY = 0.5  # (m)
DTH = math.radians(1.0)   # (rdians)

SAVE_FILE_NAME = '/home/hayashi/catkin_ws/src/soma_pkg/soma_tools/data/Q-set3.txt'
HEADER = 'dx={},dy={},dth={}\nx,y,th'.format(DX, DY, math.degrees(DTH))

Q = []
# uniformal sampling
nx = int((X_RANGE[1]-X_RANGE[0])/DX)
ny = int((Y_RANGE[1]-Y_RANGE[0])/DY)
nth = int((TH_RANGE[1]-TH_RANGE[0])/DTH)
print('nx:', nx, 'ny:', ny, 'nth:', nth)
x_set = np.linspace(X_RANGE[0], X_RANGE[1], num=nx+1)
y_set = np.linspace(Y_RANGE[0], Y_RANGE[1], num=ny+1)
th_set = np.linspace(TH_RANGE[0], TH_RANGE[1], num=nth+1)
# make set
Q = list(itertools.product(x_set, y_set, th_set))
Q = np.array(Q)
print(os.path.dirname(__file__))
np.savetxt(SAVE_FILE_NAME,
           Q,
           fmt='%.3f',
           delimiter='\t',
           comments='#',
           header=HEADER)
