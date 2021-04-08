#!/usr/bin/python3

"""
atv_kinematic_model.py

Description
-----------
Analyze python module for ATV kinematic model
\phi: steering angle [rad],
L: thread length [m]

q = (x, y, \theta)
q' = {
    x' = v*cos(\theta)
    y' = v*sin(\theta)
    \theta' = v*tan(\phi)/L
}

"""

import numpy as np
import math
import itertools
from scipy.integrate import odeint  # simulation library
import matplotlib.pyplot as plt
import os

# Phisycal arguments
L = 1.04  # tread length (sec)

# Simulation arguments
T = 5.0  # time limit (sec)
DT = 0.1  # time slice (sec)

V = 1.0  # constant velocity [m]
PHI = math.radians(-30.0)  # constant steering angle [rad]

V_MIN = 1.0
V_MAX = 1.0
V_STEP = 1.0

PHI_MIN = -30.0
PHI_MAX = 30.0
PHI_STEP = 5.0

V_SET = np.linspace(V_MIN, V_MAX, num=int((V_MAX-V_MIN)/V_STEP)+1)
PHI_SET = np.linspace(math.radians(PHI_MIN),
                      math.radians(PHI_MAX),
                      num=int((PHI_MAX-PHI_MIN)/PHI_STEP)+1)

#
SAVE_DIR = os.path.dirname(__file__)+'/../data/kinematic_model/'


def kinematic_model(var, t, l, v, phi):
  dxdt = v*math.cos(var[2])
  dydt = v*math.sin(var[2])
  dthdt = v*math.tan(phi)/l
  return [dxdt, dydt, dthdt]


if __name__ == '__main__':
  print('ATV Kinematic Model Simulator')
  print(os.path.dirname(__file__))

  t_list = np.linspace(0.0, T, num=int(T/DT))

  print('Simulation')

  for (v, phi) in list(itertools.product(V_SET, PHI_SET)):
    print('V={}, PHI={:.2f}'.format(v, math.degrees(phi)))

    Q = odeint(kinematic_model,
               [0.0, 0.0, 0.0],
               t_list,
               args=(L, v, phi))

    np.savetxt(SAVE_DIR+'V{}-PHI{:.1f}.txt'.format(v, math.degrees(phi)),
               Q,
               fmt = '%.3f',
               delimiter = '\t',
               header = 'x,y,th,',
               comments = '#',)
