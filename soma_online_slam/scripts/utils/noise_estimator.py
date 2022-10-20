#!/usr/bin/env python
import os
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from scipy.stats import norm


DIR = os.path.dirname(__file__)
MOTION_FILE_NAME = DIR + '/../../data/motion.txt'


def motion_noise_estimation():
    samples_num = 350

    X1 = []
    X2 = []
    X3 = []
    Y = []
    YA = []

    fo = open(MOTION_FILE_NAME, "r")

    for i in range(samples_num):
        l = fo.readline()
        tmp = l.split(" ")
        # if abs(float(tmp[0])) < 0.1 and abs(float(tmp[1])) < 0.002 and abs(float(tmp[2])) < 0.1:
        X1.append(abs(float(tmp[0])))
        X2.append(abs(float(tmp[1])))
        X3.append(abs(float(tmp[2])))
        Y.append(float(tmp[8]))
        YA.append(abs(float(tmp[8])))

    fo.close()

    samples_num = len(Y)

    X1_, X2_, X3_, Y_ = [], [], [], []
    X1C, X2C, X3C, YC = X1[:], X2[:], X3[:], Y[:]
    division = 10
    thresholds = [(max(X1C)-min(X1C))/division, (max(X2C) -
                                                 min(X2C))/division, (max(X3C)-min(X3C))/division]

    # ax = plt.subplot(2, 2, 1)
    # plt.hist(X1, [(max(X1)-min(X1))/division*i for i in range(division+1)])
    # ax = plt.subplot(2, 2, 2)
    # plt.hist(X1, [(max(X2)-min(X2))/division*i for i in range(division+1)])
    # ax = plt.subplot(2, 2, 3)
    # plt.hist(X1, [(max(X3)-min(X3))/division*i for i in range(division+1)])

    # plt.show()

    # return

    num = samples_num
    while YC != []:
        X1_.append(X1C[0])
        X2_.append(X2C[0])
        X3_.append(X3C[0])
        Y_.append([YC[0]])
        delete_index = [0]
        for i in range(1, num):
            if abs(X1C[i] - X1_[-1]) < thresholds[0] and abs(X2C[i] - X2_[-1]) < thresholds[1] and abs(X3C[i] - X3_[-1]) < thresholds[2]:
                delete_index.append(i)
                Y_[-1].append(YC[i])
        delete_index.sort()
        delete_index.reverse()
        for index in delete_index:
            X1C.pop(index)
            X2C.pop(index)
            X3C.pop(index)
            YC.pop(index)
        num = len(YC)

    X1C_, X2C_, X3C_, YC_ = X1_[:], X2_[:], X3_[:], Y_[:]
    X1_, X2_, X3_, Y_ = [], [], [], []
    min_num = 10
    for i in range(len(YC_)):
        if len(YC_[i]) > min_num:
            X1_.append(X1C_[i])
            X2_.append(X2C_[i])
            X3_.append(X3C_[i])
            Y_.append(norm.fit(YC_[i])[1])

    X = np.transpose([X1_, X2_, X3_])
    X = np.c_[X, np.ones(X.shape[0])]
    linreg, residuals = np.linalg.lstsq(X, Y_, rcond=None)[:2]
    print(linreg)
    print(residuals)

    x1 = np.linspace(min(X1_), max(X1_), 100)
    x2 = np.linspace(min(X2_), max(X2_), 100)
    x3 = np.linspace(min(X3_), max(X3_), 100)

    print("y1")
    y1 = []
    for i in x2:
        for j in x3:
            y1.append(linreg[0]*np.array([x1[0], x1[-1]]) +
                      linreg[1]*i + linreg[2]*j + linreg[3])

    print("y2")
    y2 = []
    for i in x3:
        for j in x1:
            y2.append(
                linreg[0]*j + linreg[1]*np.array([x2[0], x2[-1]]) + linreg[2]*i + linreg[3])

    print("y3")
    y3 = []
    for i in x1:
        for j in x2:
            y3.append(linreg[0]*i + linreg[1]*j + linreg[2]
                      * np.array([x3[0], x3[-1]]) + linreg[3])

    print("fig 1")
    plt.figure(1)
    ax = plt.subplot(2, 2, 1)
    plt.plot(X1, YA, 'k.', alpha=0.1)
    for k in y1:
        plt.plot([x1[0], x1[-1]], k, 'g', alpha=0.1)
    plt.plot(X1_, Y_, 'r.', alpha=0.5)

    ax = plt.subplot(2, 2, 2)
    plt.plot(X2, YA, 'k.', alpha=0.1)
    for k in y2:
        plt.plot([x2[0], x2[-1]], k, 'g', alpha=0.1)
    plt.plot(X2_, Y_, 'r.', alpha=0.5)

    ax = plt.subplot(2, 2, 3)
    plt.plot(X3, YA, 'k.', alpha=0.1)
    for k in y3:
        plt.plot([x3[0], x3[-1]], k, 'g', alpha=0.1)
    plt.plot(X3_, Y_, 'r.', alpha=0.5)

    print("fig 2")
    plt.figure(2)
    ax = plt.axes(projection="3d")
    plt.plot(X1, X2, YA, 'k.', alpha=0.1)
    x11, x22 = np.meshgrid([x1[0], x1[-1]], [x2[0], x2[-1]])
    y12 = []
    for i in x3:
        y12.append(linreg[0]*x11 + linreg[1] *
                   x22 + linreg[2]*i + linreg[3])
    for k in y12:
        ax.plot_surface(x11, x22, k, color='g', alpha=0.02)
    plt.plot(X1_, X2_, Y_, 'r.', alpha=1.0)

    print("fig 3")
    plt.figure(3)
    ax = plt.axes(projection="3d")
    plt.plot(X2, X3, YA, 'k.', alpha=0.1)
    x22, x33 = np.meshgrid([x2[0], x2[-1]], [x3[0], x3[-1]])
    y23 = []
    for i in x1:
        y23.append(linreg[0]*i + linreg[1] *
                   x22 + linreg[2]*x33 + linreg[3])
    for k in y23:
        ax.plot_surface(x22, x33, k, color='g', alpha=0.02)
    plt.plot(X2_, X3_, Y_, 'r.', alpha=1.0)

    print("fig 4")
    plt.figure(4)
    ax = plt.axes(projection="3d")
    plt.plot(X1, X3, YA, 'k.', alpha=0.1)
    x11, x33 = np.meshgrid([x1[0], x1[-1]], [x3[0], x3[-1]])
    y13 = []
    for i in x2:
        y13.append(linreg[0]*x11 + linreg[1] *
                   i + linreg[2]*x33 + linreg[3])
    for k in y13:
        ax.plot_surface(x11, x33, k, color='g', alpha=0.02)
    plt.plot(X1_, X3_, Y_, 'r.', alpha=1.0)

    plt.show()


if __name__ == '__main__':
    motion_noise_estimation()
