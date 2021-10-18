import numpy as np
from math import cos, sin, sqrt, atan2, pi
import matplotlib.pyplot as plt


'''
---------------------- NOTATIONS ----------------------

- Pose: r = (x, y, theta)
- Command: u = (v, omega)
- Observation: z = (z1, ..., zi, ..., zn)
        where zi = (d, phi)
- Belief: b = N(mu, sigma)
- Kalman gain: K

-------------------------------------------------------

'''

# Theoretical motion


def g(r, u):
    x, y, theta = r.transpose()[0]
    v, omega = u.transpose()[0]
    return np.array([[x - float(v)/omega * sin(theta) + float(v)/omega * sin(theta + omega)],
                     [y + float(v)/omega * cos(theta) - float(v) /
                      omega * cos(theta + omega)],
                     [theta + omega]])


# Theoretical motion jacobian / pose

def G(r, u):
    x, y, theta = r.transpose()[0]
    v, omega = u.transpose()[0]
    return np.array([[1, 0, -float(v)/omega * cos(theta) + float(v)/omega * cos(theta + omega)],
                     [0, 1, -float(v)/omega * sin(theta) +
                      float(v)/omega * sin(theta + omega)],
                     [0, 0, 1]])


# Theoretical motion jacobian / command

def V(r, u):
    x, y, theta = mu.transpose()[0]
    v, omega = u.transpose()[0]
    return np.array([[-sin(theta)/omega + sin(theta + omega)/omega, v*sin(theta)/omega**2 - v*sin(theta + omega)/omega**2],
                     [cos(theta)/omega - cos(theta + omega)/omega, -v *
                      cos(theta)/omega**2 + v*cos(theta + omega)/omega**2],
                     [0, 1]])


# Command-related motion noise

def M(u):
    v, omega = u.transpose()[0]
    alphas = 6*[0.1]
    return np.array([[alphas[0]*abs(v) + alphas[1]*abs(omega), 0],
                     [0, alphas[2]*abs(v) + alphas[3]*abs(omega)]])


# Yaw noise induced by motion

def R(u):
    v, omega = u.transpose()[0]
    alphas = 2*[0.1]
    return np.array([[0, 0, 0],
                     [0, 0, 0],
                     [0, 0, alphas[0]*abs(v) + alphas[1]*abs(omega)]])


# Theoretical observation

def h(r):
    x, y, theta = r.transpose()[0]
    xf, yf = 10, 0
    return np.array([[sqrt((xf-x)**2 + (yf-y)**2)],
                     [atan2(yf-y, xf-x) - theta]])


# Theoretical observation jacobian / pose

def H(r):
    x, y, theta = r.transpose()[0]
    xf, yf = 10, 0
    q = (xf-x)**2 + (yf-y)**2
    return np.array([[(x-xf)/sqrt(q), (y-yf)/sqrt(q), 0],
                     [-(y-yf)/q, (x-xf)/q, -1]])


# Observation noise

def Q(z):
    d, phi = z.transpose()[0]
    alphas = 4*[0.1]
    return np.array([[alphas[0]*abs(d) + alphas[1]*abs(phi), 0],
                     [0, alphas[2]*abs(d) + alphas[3]*abs(phi)]])


# TESTS

max_time = 5

# Prior
mu = np.array([[0],
               [0],
               [0]])
sigma = np.array([[5, 0, 0],
                  [0, 5, 0],
                  [0, 0, 0.5]])

# Init display
fig, ax = plt.subplots()

for k in range(max_time):
    # New command & observation
    u = np.array([[1],
                  [0.1]])
    z = np.array([[10 - (mu[0][0]+u[0][0])],
                  [0]])

    # Prediction
    predicted_mu = g(mu, u)
    predicted_sigma = (G(mu, u).dot(sigma)).dot(
        G(mu, u).transpose()) + (V(mu, u).dot(M(u))).dot(V(mu, u).transpose()) + R(u)

    # Correction
    K = (predicted_sigma.dot(H(mu).transpose())).dot(np.linalg.inv(
        (H(mu).dot(predicted_sigma)).dot(H(mu).transpose()) + Q(z)))
    new_mu = predicted_mu + K.dot(z-h(predicted_mu))
    new_sigma = (np.identity(3) - K.dot(H(mu))).dot(predicted_sigma)

    # Log
    print("")
    print("Initial:")
    print(mu)
    print(sigma)
    print("")
    print("Prediction:")
    print(predicted_mu)
    print(predicted_sigma)
    print("")
    print("Correction:")
    print(new_mu)
    print(new_sigma)
    print("")

    # Display

    delta = 0.2
    x = np.arange(-4, 12, delta)
    y = np.arange(-4, 4, delta)
    X, Y = np.meshgrid(x, y)

    Z1 = np.zeros([len(X), len(X[0])])
    for i in range(len(X)):
        for j in range(len(X[0])):
            diff = np.array([[X[i, j]-mu[0][0]], [Y[i, j]-mu[1][0]]])
            diff_transpose = diff.transpose()
            sigma = sigma[:2, :2]
            sigma_inverse = np.linalg.inv(sigma)
            Z1[i, j] = 1/(sqrt(2*pi*np.linalg.det(sigma))) * \
                np.exp(-1/2 * (diff_transpose.dot(sigma_inverse)
                               ).dot(diff))[0][0]

    Z2 = np.zeros([len(X), len(X[0])])
    for i in range(len(X)):
        for j in range(len(X[0])):
            diff = np.array([[X[i, j]-predicted_mu[0][0]],
                            [Y[i, j]-predicted_mu[1][0]]])
            diff_transpose = diff.transpose()
            sigma = predicted_sigma[:2, :2]
            sigma_inverse = np.linalg.inv(sigma)
            Z2[i, j] = 1/(sqrt(2*pi*np.linalg.det(sigma))) * \
                np.exp(-1/2 * (diff_transpose.dot(sigma_inverse)
                               ).dot(diff))[0][0]

    Z3 = np.zeros([len(X), len(X[0])])
    for i in range(len(X)):
        for j in range(len(X[0])):
            diff = np.array([[X[i, j]-new_mu[0][0]], [Y[i, j]-new_mu[1][0]]])
            diff_transpose = diff.transpose()
            sigma = new_sigma[:2, :2]
            sigma_inverse = np.linalg.inv(sigma)
            Z3[i, j] = 1/(sqrt(2*pi*np.linalg.det(sigma))) * \
                np.exp(-1/2 * (diff_transpose.dot(sigma_inverse)
                               ).dot(diff))[0][0]

    if k == 0:
        plt.plot(mu[0][0], mu[1][0], "go")
        CS1 = ax.contour(X, Y, Z1, levels=[0.02], colors=['g'])
        # ax.clabel(CS1, inline=1, fontsize=10)
    plt.plot(predicted_mu[0][0], predicted_mu[1][0], "ro")
    CS2 = ax.contour(X, Y, Z2, levels=[0.02], colors=['r'])
    # ax.clabel(CS2, inline=1, fontsize=10)
    plt.plot(new_mu[0][0], new_mu[1][0], "bo")
    CS3 = ax.contour(X, Y, Z3, levels=[0.02], colors=['b'])
    # ax.clabel(CS3, inline=1, fontsize=10)
    plt.plot(10, 0, "y*", markersize="20")

    mu = new_mu
    sigma = new_sigma

ax.set_title('EKF localization')
plt.show()
