import numpy as np
import math
from scipy.integrate import odeint
import matplotlib.pyplot as plt

# paremeters
L = 1.04  # tread length [m]
V = 1.0  # constant velocity [m]
dPHI = 5.0
PHI_MIN = -30.0
PHI_MAX = 30.0
NUM_PHI = int((PHI_MAX - PHI_MIN)/dPHI) + 1
PHIS = np.linspace(-30, 30, num=NUM_PHI)  # constant steering angle [deg]
Q_INIT = [0.0, 0.0, 0.0]  # initial state vector on 2-D [x,y,theta]

#
SAVE_DIR = '/home/hayashi/catkin_ws/src/soma_pkg/soma_tools/data/'


def func_kinematics(var, t, l, v, phi):
    dxdt = v*math.cos(var[2])
    dydt = v*math.sin(var[2])
    dthdt = v*math.tan(phi)/l
    return [dxdt, dydt, dthdt]


if __name__ == '__main__':
    print('SOMA Kinematic Model Simulator')

    t_list = np.linspace(0.0, 5.0, num=10000)

    print("compute")
    data_set = []
    for phi in PHIS:
        print("Steer:", phi, math.radians(phi))
        Q = odeint(func_kinematics, Q_INIT, t_list, args=(L, V, math.radians(phi)))

        x_list = np.array(Q[:, 0]).copy()
        y_list = np.array(Q[:, 1]).copy()
        th_list = np.array(Q[:, 2]).copy()

        _data_set = np.array([x_list, y_list, th_list]).transpose()
        data_set.append(_data_set)
    print("fin.")

    data_set = np.array(data_set)
    print(data_set.shape)

    for i, d in enumerate(data_set):
        np.savetxt(SAVE_DIR+str(i)+'_kinematics-model.txt',
                   d,
                   fmt='%.2f',
                   delimiter='\t',
                   header='x,y,th,',
                   comments='#',)
