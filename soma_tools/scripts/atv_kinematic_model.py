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
from scipy.integrate import odeint  # simulation library
import matplotlib.pyplot as plt
import os

# Phisycal arguments
L = 1.04  # tread length (sec)

# Simulation arguments
T = 3.0  # time limit (sec)
DT = 0.1  # time slice (sec)

V = 1.0  # constant velocity [m]
PHI = math.radians(-30.0)  # constant steering angle [rad]

# dPHI = 5.0
# PHI_MIN = -30.0
# PHI_MAX = 30.0
# NUM_PHI = int((PHI_MAX - PHI_MIN)/dPHI) + 1
# PHIS = np.linspace(-30, 30, num=NUM_PHI)  # constant steering angle [deg]
# Q_INIT = [0.0, 0.0, 0.0]  # initial state vector on 2-D [x,y,theta]

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
  print('V={}, PHI={}'.format(V, PHI))

  Q = odeint(kinematic_model,
             [0.0, 0.0, 0.0],
             t_list,
             args=(L, V, PHI))

  np.savetxt(SAVE_DIR+'V{}-PHI{:.2f}'.format(V, math.degrees(PHI)),
             Q,
             fmt='%.3f',
             delimiter='\t',
             header='x,y,th,',
             comments='#',)

#   print(Q)

#   data_set = []

#   for phi in PHIS:
#     print("Steer:", phi, math.radians(phi))
#     Q = odeint(func_kinematics, Q_INIT, t_list, args=(L, V, math.radians(phi)))

#     x_list = np.array(Q[:, 0]).copy()
#     y_list = np.array(Q[:, 1]).copy()
#     th_list = np.array(Q[:, 2]).copy()

#     _data_set = np.array([x_list, y_list, th_list]).transpose()
#     data_set.append(_data_set)
#   print("fin.")

#   data_set = np.array(data_set)
#   print(data_set.shape)

#   for i, d in enumerate(data_set):
#     np.savetxt(SAVE_DIR+str(i)+'_kinematics-model.txt',
#                d,
#                fmt='%.2f',
#                delimiter='\t',
#                header='x,y,th,',
#                comments='#',)
