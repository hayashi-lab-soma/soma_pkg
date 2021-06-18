from random import uniform, normalvariate, random
import matplotlib.pyplot as plt
from math import pi, cos, sin, sqrt, erf, atan2, acos, exp, tan
import numpy as np
from scipy.stats import multivariate_normal
import time


# Motion model

def predict(pose, command, noise):
    #  Velocity-based (dead reckoning)
    new_x = pose[0] - command[0]/command[1] * \
        sin(pose[2]) + command[0]/command[1]*sin(pose[2] + command[1])
    new_y = pose[1] + command[0]/command[1] * \
        cos(pose[2]) - command[0]/command[1]*cos(pose[2] + command[1])
    new_theta = pose[2] + command[1]

    x_noise, y_noise, theta_noise = noise
    x_sigma = x_noise[0]*command[0] + x_noise[1]*command[1]
    y_sigma = y_noise[0]*command[0] + y_noise[1]*command[1]
    theta_sigma = theta_noise[0]*command[0] + theta_noise[1]*command[1]
    sigma = [[x_sigma, 0, 0],
             [0, y_sigma, 0],
             [0, 0, theta_sigma]]

    new_x, new_y, new_theta = multivariate_normal.rvs(
        [new_x, new_y, new_theta], sigma)
    if new_theta <= -pi:
        new_theta += 2*pi
    elif new_theta > pi:
        new_theta -= 2*pi

    return new_x, new_y, new_theta


# Observation model

def observation(pose, visibility, noise):
    observation = []
    for i, f in enumerate(features):
        distance = sqrt((f[0] - pose[0])**2 + (f[1] - pose[1])**2)
        angle = atan2(f[1] - pose[1], f[0] - pose[0]) - pose[2]

        d_noise, phi_noise = noise
        if distance < visibility:
            d_sigma = d_noise[0]*distance + d_noise[1]*angle
            phi_sigma = phi_noise[0]*distance + phi_noise[1]*angle
            sigma = [[d_sigma, 0],
                     [0, phi_sigma]]

            noisy_observation = multivariate_normal.rvs(
                [distance, angle], sigma)
            if noisy_observation[1] <= -pi:
                noisy_observation[1] += 2*pi
            elif noisy_observation[1] > pi:
                noisy_observation -= 2*pi
            observation.append(list(noisy_observation))
    return observation


def correspondence(features, pose, observation, noise, threshold):
    corresponding_feature = None
    highest_likelihood = 0
    for i, f in enumerate(features):
        mu = f[0].transpose()[0]
        xf, yf = mu
        d = sqrt((xf - pose[0])**2 + (yf - pose[1])**2)
        phi = atan2(yf - pose[1], xf - pose[0]) - pose[2]

        d_noise, phi_noise = noise
        d_sigma = d_noise[0]*d + d_noise[1]*phi
        phi_sigma = phi_noise[0]*d + phi_noise[1]*phi
        sigma = [[d_sigma, 0],
                 [0, phi_sigma]]

        if observation[1] > 0 and phi < 0:
            phi += 2*pi
        elif observation[1] < 0 and phi > 0:
            phi -= 2*pi

        feature_likelihood = multivariate_normal.pdf(
            observation, [d, phi], sigma)
        if feature_likelihood > highest_likelihood:
            highest_likelihood = feature_likelihood
            corresponding_feature = i

    if highest_likelihood < threshold:
        corresponding_feature = None

    return corresponding_feature, highest_likelihood


# EKF

def h(pose, f):
    x, y, theta = pose.transpose()[0]
    xf, yf = f
    return np.array([[sqrt((xf-x)**2 + (yf-y)**2)],
                     [atan2(yf-y, xf-x) - theta]])


def h_inverse(pose, z):
    x, y, theta = pose.transpose()[0]
    d, phi = z.transpose()[0]
    tmp = phi + theta
    if tmp <= -pi:
        tmp += 2*pi
    elif tmp > pi:
        tmp -= 2*pi
    if tmp >= -pi/2 and tmp <= pi/2:
        xf = x + d/sqrt(1 + tan(phi + theta)**2)
        yf = y + d*tan(phi + theta)/sqrt(1 + tan(phi + theta)**2)
    else:
        xf = x - d/sqrt(1 + tan(phi + theta)**2)
        yf = y - d*tan(phi + theta)/sqrt(1 + tan(phi + theta)**2)
    return np.array([[xf],
                     [yf]])


def H(pose, f):
    x, y, theta = pose.transpose()[0]
    xf, yf = f.transpose()[0]
    q = (xf-x)**2 + (yf-y)**2
    return np.array([[(xf-x)/sqrt(q), (yf-y)/sqrt(q)],
                     [-(yf-y)/q, (xf-x)/q]])


def Q(z):
    d, phi = z.transpose()[0]
    alphas = [0.05, 0, 0.0005, 0]
    return np.array([[alphas[0]*abs(d) + alphas[1]*abs(phi), 0],
                     [0, alphas[2]*abs(d) + alphas[3]*abs(phi)]])


# Display

def display():
    plt.plot([0, 0, map_width, map_width], [
             0, map_height, 0, map_height], "y*")

    for i, f in enumerate(features):
        plt.plot(f[0], f[1], "bo", markersize=feature_radius)
        plt.annotate(xy=f, s=i, xytext=[f[0]-2, f[1]+2])

    plt.plot(robot_pose[0], robot_pose[1], "ro", markersize=robot_radius)
    angle = np.linspace(0, 2*pi, 1000)
    x = robot_pose[0] + visibility*np.cos(angle)
    y = robot_pose[1] + visibility*np.sin(angle)
    plt.plot(x, y, "ro", markersize=0.05)
    plt.arrow(robot_pose[0], robot_pose[1], 10 *
              cos(robot_pose[2]), 10*sin(robot_pose[2]), "ro")

    for p in particles:
        plt.plot(p[0], p[1], "go", markersize=max(
            1, p[3]*min(map_width, map_height)/2))

    highest_weight = 0
    for p in particles:
        if p[3] > highest_weight:
            highest_weight = p[3]
            most_probable_particle = p
    if highest_weight > 0:
        for i, [mu, sigma] in enumerate(most_probable_particle[4]):
            f = mu.transpose()[0]
            plt.plot(f[0], f[1], "go", markersize=feature_radius)
            plt.annotate(xy=f, s=i, xytext=[f[0]-2, f[1]+2])

            delta = 0.5
            window = 50
            x = np.arange(f[0]-window/2, f[0]+window/2, delta)
            y = np.arange(f[1]-window/2, f[1]+window/2, delta)
            X, Y = np.meshgrid(x, y)

            Z = np.zeros([len(X), len(X[0])])
            for k in range(len(X)):
                for j in range(len(X[0])):
                    diff = np.array(
                        [[X[k, j]-mu[0][0]], [Y[k, j]-mu[1][0]]])
                    diff_transpose = diff.transpose()
                    sigma_inverse = np.linalg.inv(sigma)
                    tmp = (diff_transpose.dot(
                        sigma_inverse)).dot(diff)[0][0]
                    tmp = max(tmp, -709)
                    Z[k, j] = 1/(sqrt(2*pi*abs(np.linalg.det(sigma)))) * \
                        np.exp(-1/2 * tmp)

            CS2 = ax.contour(X, Y, Z, levels=[0.002], colors=['g'])

    plt.plot()


# TESTS

# Parameters

max_time = 8

# Command
v = 5
omega = 0.1
u = [v, omega]

# Motion noise
x_noise = [0.05, 0]
y_noise = [0.05, 0]
theta_noise = [0, 0.0005]
motion_noise = [x_noise, y_noise, theta_noise]

# Observation noise
d_noise = [0.05, 0]
phi_noise = [0.0005, 0]
observation_noise = [d_noise, phi_noise]

# Map

map_width = 100
map_height = 100

# Features

features_num = 10
# Only for display (features are collapsed to their center of mass)
feature_radius = 3
features = []
for i in range(features_num):
    new_feature_x = uniform(0, map_width)
    new_feature_y = uniform(0, map_height)
    new_feature = [new_feature_x, new_feature_y]
    features.append(new_feature)

# Initial robot pose

initial_x = map_width/2
initial_y = map_height/2
initial_theta = 0
robot_pose = [initial_x, initial_y, initial_theta]
robot_radius = 10
visibility = 30
print("\nInitial pose: " + str(robot_pose))

# Particles

particles_num = 100
particles = []
particles_sigma = [[100, 0, 0],
                   [0, 100, 0],
                   [0, 0, 1]]

for i in range(particles_num):
    # Right initial pose
    new_particle_x = robot_pose[0]
    new_particle_y = robot_pose[1]
    new_particle_theta = robot_pose[2]

    new_particle_w = 1.0/particles_num
    new_particle_f = []
    new_particle = [new_particle_x, new_particle_y,
                    new_particle_theta, new_particle_w, new_particle_f]
    particles.append(new_particle)

# Display

subplot = 200 + max_time/2*10 + 1
ax = plt.subplot(subplot)
display()

# Simulation

for i in range(max_time-1):
    start = time.time()

    # Real world simulation
    print("\nStep: " + str(i+1))
    print("Command: " + str(u))
    robot_pose[0], robot_pose[1], robot_pose[2] = predict(
        robot_pose, u, motion_noise)
    print("New pose: " + str(robot_pose))
    new_observation = observation(robot_pose, visibility, observation_noise)
    print("Observation: " + str(new_observation))

    for j, p in enumerate(particles):
        print("\nParticle " + str(j) + ":")

        # Prediction
        p[0], p[1], p[2] = predict(p, u, motion_noise)

        # Correction & mapping
        pose = np.array([[p[0]],
                         [p[1]],
                         [p[2]]])
        print("Predicted pose: \n" + str(pose))

        for i in range(len(new_observation)):
            new_z = np.array([[new_observation[i][0]],
                              [new_observation[i][1]]])
            print("Observation " + str(i) + ": \n" + str(new_z))

            print("Number of features: " + str(len(p[4])))
            corresponding_feature, highest_likelihood = correspondence(
                p[4], pose, new_observation[i], observation_noise, 0.001)
            print("Highest likelihood: " + str(highest_likelihood))
            if corresponding_feature != None:
                print("Corresponding feature: " + str(corresponding_feature))
            else:
                print("Previously unseen feature")
            if corresponding_feature == None:
                mu = h_inverse(pose, new_z)
                print("Feature estimated position :\n" + str(mu))
                H1 = H(pose, mu)
                H1_inverse = np.linalg.inv(H1)
                Q1 = Q(new_z)
                sigma = (H1_inverse.dot(Q1)).dot(H1_inverse.transpose())
                p[4].append([mu, sigma])
            else:
                mu, sigma = p[4][corresponding_feature]
                f = mu.transpose()[0]
                predicted_z = h(pose, f)
                H1 = H(pose, mu)
                Q1 = (H1.dot(sigma)).dot(
                    H1.transpose()) + Q(new_z)
                Q1_inverse = np.linalg.inv(Q1)
                K = (sigma.dot(H1.transpose())).dot(Q1_inverse)
                new_mu = mu + K.dot(new_z - predicted_z)
                new_sigma = (np.identity(2) - K.dot(H1)).dot(sigma)
                p[4][corresponding_feature] = [new_mu, new_sigma]
                tmp = (
                    (new_z - predicted_z).transpose().dot(Q1_inverse)).dot(new_z - predicted_z)[0][0]
                tmp = max(tmp, -709)
                weight_update = 1 / \
                    sqrt(2*pi*abs(np.linalg.det(Q1))) * exp(-1/2 * tmp)
                p[3] *= weight_update

    # Normalization
    weights_sum = 0
    for p in particles:
        weights_sum += p[3]
    assert weights_sum > 0, "/!\ Sum of weights equals 0 !"
    for p in particles:
        p[3] /= weights_sum

    max_weight = 0
    for p in particles:
        if p[3] > max_weight:
            max_weight = p[3]
            most_probable_pose = p[:3]
    print("Most probable pose: " + str(most_probable_pose))

    # Resampling criteria (effective particles number)
    effective_particles_num = 0
    for p in particles:
        effective_particles_num += p[3]**2
    effective_particles_num = 1/effective_particles_num

    # Resampling
    if effective_particles_num < particles_num/2:
        print("Resampling: Yes")
        cumulated_weights = [0]
        for p in particles:
            cumulated_weights.append(cumulated_weights[-1] + p[3])

        old_particles = []
        for p in particles:
            old_particles.append(p[:])
        particles = []
        for i in range(particles_num):
            r = random()
            for j in range(len(cumulated_weights)-1):
                if r >= cumulated_weights[j] and r < cumulated_weights[j+1]:
                    resampling_sigma = [[0.01, 0, 0],
                                        [0, 0.01, 0],
                                        [0, 0, 0.0001]]
                    new_particle = list(multivariate_normal.rvs(
                        old_particles[j][:3], resampling_sigma))
                    new_particle.append(1.0/particles_num)
                    new_particle.append(old_particles[j][4])
                    particles.append(new_particle)
    else:
        print("Resampling: No")

    subplot += 1
    ax = plt.subplot(subplot)
    display()

    stop = time.time()
    print("Time: " + str(stop-start))

    plt.show()


print("\n")
plt.show()
