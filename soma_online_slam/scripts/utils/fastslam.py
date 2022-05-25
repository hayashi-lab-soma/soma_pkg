from random import uniform, random, seed
import matplotlib.pyplot as plt
from math import pi, cos, sin, sqrt, atan2, exp, tan, ceil
import numpy as np
from numpy.compat.py3k import _PurePath__fspath__
from scipy.stats import multivariate_normal
from scipy.stats.mvn import mvnun
import time


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


# Convenient representation of particles set history
class History:
    def __init__(self, particles_num):
        self.particles_num = particles_num
        self.particles_sets = [[[i, 0.0, 0.0, 1.0/particles_num]
                                for i in range(particles_num)]]
        self.steps = []
        self.final_most_probable_particle_history = []

        return

    def add_generation(self, new_particles_set, step):
        new_particles_set.sort()
        self.particles_sets.append(new_particles_set[:])
        self.steps.append(step)

        return

    def add_most_probable_particle(self, index):
        self.final_most_probable_particle_history.append(index)

        for t in range(len(self.steps)-1, -1, -1):
            previous = self.final_most_probable_particle_history[-1]
            self.final_most_probable_particle_history.append(
                self.particles_sets[t][previous][0])

        self.final_most_probable_particle_history.reverse()

        return

    def separator(self, lengths):
        tmp = "                   |"
        for l in lengths:
            for i in range(6*l-1):
                tmp += "-"
            tmp += "|"
        tmp += "\n"

        return tmp

    def particles(self, lengths, generation, previous_generation, ps):
        if generation > previous_generation:
            tmp = " GENERATION " + str(generation)
            if generation < 10:
                tmp += " "
            tmp += "     |"
        else:
            tmp = "                   |"

        index = 0
        for l in lengths:
            for i in range((6*l-4)/2):
                tmp += " "
            n = ps[index][0]
            tmp += " " + str(n) + " "
            if n < 10:
                tmp += " "
            for i in range(6*l-5 - (6*l-4)/2):
                tmp += " "
            tmp += "|"
            index += l

        tmp += "                 " + str(len(lengths)) + " particles\n"

        return tmp

    def values(self, type, lengths, previous_lengths, ps, t, best, worst):
        if type == "weight":
            v_index = 3
            tmp = "    WEIGHT         |"
        elif type == "position":
            v_index = 1
            tmp = "POSITION ERROR     |"
        elif type == "map":
            v_index = 2
            tmp = "  MAP ERROR        |"

        average = 0
        index = 0
        for j in range(len(lengths)):
            l = lengths[j]
            for i in range((6*l-5)/2):
                tmp += " "
            if j > 0:
                index += lengths[j-1]
            v = ps[index][v_index]

            tmp += str(round(v, 1)) + " "

            if t == 0 or self.steps[t-1] == "RESAMPLING":
                tmp += "o"
            else:
                old_index = 0
                for pl in previous_lengths:
                    if old_index + pl > index:
                        break
                    else:
                        old_index += pl
                old_v = self.particles_sets[t-1][old_index][v_index]

                if round(v, 1) < round(old_v, 1):
                    tmp += "-"
                elif round(v, 1) == round(old_v, 1):
                    tmp += "="
                if round(v, 1) > round(old_v, 1):
                    tmp += "+"

            if type == "weight":
                average += v * lengths[j]
            else:
                average += v * ps[index][3] * lengths[j]
            for i in range(6*l-6 - (6*l-5)/2):
                tmp += " "
            tmp += "|"

        if type == "weight":
            average /= self.particles_num
        tmp += "     Average: " + str(round(average, 1)) + " / Worst: " + str(
            round(worst, 1)) + " / Best: " + str(round(best, 1)) + "\n"

        return tmp

    def step(self, t):
        tmp = "\n                   "
        l = len(self.steps[t])
        for i in range(3*self.particles_num - l/2):
            tmp += " "
        tmp += self.steps[t]
        for i in range(3*self.particles_num - l + l/2):
            tmp += " "
        tmp += "\n\n"

        return tmp

    def __str__(self):
        tmp = "\n"
        previous_generation = -1
        generation = 0

        for t, ps in enumerate(self.particles_sets):
            position_best = ps[0][1]
            position_worst = position_best
            map_best = ps[0][2]
            map_worst = map_best
            weight_best = ps[0][3]
            weight_worst = weight_best
            if t > 0:
                previous_lengths = lengths[:]
            else:
                previous_lengths = []
            lengths = []
            previous = ps[0][0]
            current_length = 1

            for p in ps[1:]:
                if p[0] != previous:
                    lengths.append(current_length)
                    if p[1] < position_best:
                        position_best = p[1]
                    elif p[1] > position_worst:
                        position_worst = p[1]
                    if p[2] < map_best:
                        map_best = p[2]
                    elif p[2] > map_worst:
                        map_worst = p[2]
                    if p[3] > weight_best:
                        weight_best = p[3]
                    elif p[3] < weight_worst:
                        weight_worst = p[3]
                    current_length = 1
                else:
                    current_length += 1
                previous = p[0]
            lengths.append(current_length)

            tmp += self.separator(lengths)
            tmp += self.particles(lengths, generation, previous_generation, ps)
            tmp += self.separator(lengths)
            tmp += self.values("weight", lengths,
                               previous_lengths, ps, t, weight_best, weight_worst)
            tmp += self.separator(lengths)
            tmp += self.values("position", lengths,
                               previous_lengths, ps, t, position_best, position_worst)
            tmp += self.values("map", lengths,
                               previous_lengths, ps, t, map_best, map_worst)
            tmp += self.separator(lengths)

            if t < len(self.particles_sets) - 1:
                tmp += self.step(t)

            previous_generation = generation
            if t < len(self.particles_sets) - 1 and self.steps[t] == "UPDATE":
                generation += 1

        return tmp

    def display(self):
        highest_weight = 0
        highest_position_error = 0
        highest_map_error = 0

        for t, ps in enumerate(self.particles_sets):
            previous = -1
            for p in ps:
                if p[0] != previous:
                    highest_weight = max(highest_weight, p[3])
                    highest_position_error = max(highest_position_error, p[1])
                    highest_map_error = max(highest_map_error, p[2])
                    previous = p[0]

        particles_weight_evolution = []
        particles_position_error_evolution = []
        particles_map_error_evolution = []

        generation = 0
        previous_generation = 0
        for t, ps in enumerate(self.particles_sets):
            # print(generation)

            new_weights = self.particles_num*[None]
            new_position_errors = self.particles_num*[None]
            new_map_errors = self.particles_num*[None]

            previous = -1
            for p in ps:
                if p[0] != previous:
                    new_weights[p[0]] = p[3]
                    new_position_errors[p[0]] = p[1]
                    new_map_errors[p[0]] = p[2]

                    previous = p[0]

            particles_weight_evolution.append(new_weights)
            particles_position_error_evolution.append(new_position_errors)
            particles_map_error_evolution.append(new_map_errors)

            generation += 1

            if t > 0 and (self.steps[t-1] == "RESAMPLING" or t == len(self.particles_sets)-1):
                if self.steps[t-1] == "RESAMPLING":
                    generation -= 2
                else:
                    generation -= 1

                for f, highest_value in enumerate([highest_weight, highest_position_error, highest_map_error]):
                    plt.figure(f+1)
                    if self.steps[t-1] == "RESAMPLING":
                        for n in range(previous_generation, generation+1):
                            plt.plot([n for i in range(100)], np.linspace(
                                0, highest_value, 100), 'k--', lw=2)
                            plt.plot([generation-0.5 for i in range(100)],
                                     np.linspace(0, highest_value, 100), 'k', lw=3)
                    else:
                        for n in range(previous_generation, generation+1):
                            plt.plot([n for i in range(100)], np.linspace(
                                0, highest_value, 100), 'k--', lw=2)

                for p in range(self.particles_num):
                    X = [i for i in range(previous_generation, generation)]
                    if self.steps[t-1] == "RESAMPLING":
                        X += [generation-0.5, generation]
                    else:
                        X += [generation]

                    plt.figure(1)
                    highest_value = highest_weight
                    Y = [particles_weight_evolution[i][p]
                         for i in range(len(particles_weight_evolution))]
                    plt.plot(X, Y, marker='.', markersize=10)

                    plt.figure(2)
                    highest_value = highest_position_error
                    Y = [particles_position_error_evolution[i][p]
                         for i in range(len(particles_position_error_evolution))]
                    plt.plot(X, Y, marker='.', markersize=10)

                    plt.figure(3)
                    highest_value = highest_map_error
                    Y = [particles_map_error_evolution[i][p]
                         for i in range(len(particles_map_error_evolution))]
                    plt.plot(X, Y, marker='.', markersize=10)

                new_weights = []
                new_position_errors = []
                new_map_errors = []

                previous = -1
                current_weight = 0
                current_position_error = 0
                current_map_error = 0
                for p in ps:
                    if p[0] != previous:
                        current_weight = p[3]
                        current_position_error = p[1]
                        current_map_error = p[2]
                        previous = p[0]
                    new_weights.append(current_weight)
                    new_position_errors.append(current_position_error)
                    new_map_errors.append(current_map_error)

                particles_weight_evolution = [new_weights]
                particles_position_error_evolution = [new_position_errors]
                particles_map_error_evolution = [new_map_errors]

                previous_generation = generation
                generation += 1

        X = [0.0]
        for i in range(len(self.steps)):
            if self.steps[i] == "RESAMPLING" or (i < len(self.steps)-1 and self.steps[i+1] == "RESAMPLING"):
                X.append(X[-1] + 0.5)
            else:
                X.append(X[-1] + 1)

        Y1 = []
        Y2 = []
        Y3 = []
        for t in range(len(self.final_most_probable_particle_history)):
            n = self.final_most_probable_particle_history[t]
            for p in self.particles_sets[t]:
                if p[0] == n:
                    break
            Y1.append(p[3])
            Y2.append(p[1])
            Y3.append(p[2])

        plt.figure(1)
        plt.plot(X, Y1, 'r', lw=3, marker='.', markersize=15)
        plt.title("Weight")
        plt.tick_params(labelright=True)
        plt.figure(2)
        plt.plot(X, Y2, 'r', lw=3, marker='.', markersize=15)
        plt.title("Position error (m)")
        plt.tick_params(labelright=True)
        plt.figure(3)
        plt.plot(X, Y3, 'r', lw=3, marker='.', markersize=15)
        plt.title("Map error (m)")
        plt.tick_params(labelright=True)

        plt.show()

        return


# Motion model (velocity-based/dead reckoning or odometry-based)
def motion(motion_model, pose, command, noise, dt=1):
    x, y, theta = pose

    if motion_model == "velocity":
        v, omega = command
        v_noise, omega_noise, yaw_noise = noise

    elif motion_model == "odometry":
        rot1, trans, rot2 = command
        rot1_noise, trans_noise, rot2_noise = noise

    else:
        assert False, "Invalid motion model: " + motion_model

    # Covariance matrix
    if motion_model == "velocity":
        v_sigma = v_noise[0]*abs(v) + v_noise[1]*abs(omega) + v_noise[2]
        omega_sigma = omega_noise[0] * \
            abs(v) + omega_noise[1]*abs(omega) + omega_noise[2]
        yaw_sigma = yaw_noise[0] * \
            abs(v) + yaw_noise[1]*abs(omega) + yaw_noise[2]
        sigma = [[v_sigma, 0],
                 [0, omega_sigma]]

    elif motion_model == "odometry":
        rot1_sigma = rot1_noise[0] * abs(trans) + \
            rot1_noise[1] * abs(rot1) + rot1_noise[2]
        trans_sigma = trans_noise[0] * \
            abs(trans) + trans_noise[1] * \
            (abs(rot1) + abs(rot2)) + trans_noise[2]
        rot2_sigma = rot2_noise[0] * abs(trans) + \
            rot2_noise[1] * abs(rot2) + rot2_noise[2]
        sigma = [[rot1_sigma, 0, 0],
                 [0, trans_sigma, 0],
                 [0, 0, rot2_sigma]]

    # Noisy commands
    if motion_model == "velocity":
        v, omega = multivariate_normal.rvs(command, sigma)

    elif motion_model == "odometry":
        rot1, trans, rot2 = multivariate_normal.rvs(command, sigma)

    # New pose
    if motion_model == "velocity":
        if omega == 0:
            new_x = x + v*cos(theta)*dt
            new_y = y + v*sin(theta)*dt
            new_theta = theta
        else:
            new_x = x - v/omega*sin(theta)*dt + v/omega*sin(theta + omega)*dt
            new_y = y + v/omega*cos(theta)*dt - v/omega*cos(theta + omega)*dt
            new_theta = multivariate_normal.rvs(theta + omega*dt, yaw_sigma)

    elif motion_model == "odometry":
        new_x = x + trans*cos(theta + rot1)
        new_y = y + trans*sin(theta + rot1)
        new_theta = theta + rot1 + rot2

    # Ensure theta in ]-pi;pi]
    if new_theta <= -pi:
        new_theta += 2*pi
    elif new_theta > pi:
        new_theta -= 2*pi

    new_pose = [new_x, new_y, new_theta]

    return new_pose


# Observation model (lidar)
def observation(pose, min_visibility, max_visibility, noise):
    observation = []
    for i, f in enumerate(features):
        distance = sqrt((f[0] - pose[0])**2 + (f[1] - pose[1])**2)
        angle = atan2(f[1] - pose[1], f[0] - pose[0]) - pose[2]

        d_noise, phi_noise = noise
        if distance > min_visibility and distance < max_visibility:
            d_sigma = d_noise[0]*distance + d_noise[1]*abs(angle) + d_noise[2]
            phi_sigma = phi_noise[0]*distance + \
                phi_noise[1]*abs(angle) + phi_noise[2]
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


# Data association: observation -> map feature
def correspondence(features, pose, observation, min_visibility, max_visibility, noise, threshold):
    if len(features) == 0:
        return [], []

    corresponding_features = len(features)*[0]
    x, y, theta = pose.transpose()[0]

    for i, f in enumerate(features):
        mu = f[0].transpose()[0]
        xf, yf = mu
        d = sqrt((xf - x)**2 + (yf - y)**2)
        phi = atan2(yf - y, xf - x) - theta

        if d > min_visibility and d < max_visibility:
            d_noise, phi_noise = noise
            d_sigma = d_noise[0]*d + d_noise[1]*abs(phi) + d_noise[2]
            phi_sigma = phi_noise[0]*d + phi_noise[1]*abs(phi) + phi_noise[2]
            sigma = [[d_sigma, 0],
                     [0, phi_sigma]]

            if abs(observation[1] - phi) > abs(observation[1] - phi + 2*pi):
                phi -= 2*pi
            elif abs(observation[1] - phi) > abs(observation[1] - phi - 2*pi):
                phi += 2*pi

            d_interval, phi_interval = d_sigma/10, phi_sigma/10
            feature_likelihood = mvnun(np.array([observation[0]-d_interval/2.0, observation[1]-phi_interval/2.0]), np.array(
                [observation[0]+d_interval/2.0, observation[1]+phi_interval/2.0]), np.array([d, phi]), np.array(sigma))[0]

            assert feature_likelihood <= 1, "Probability greater than 1 !"

            if feature_likelihood > threshold:
                corresponding_features[i] = feature_likelihood

    corresponding_likelihoods = list(
        np.sort(np.array(corresponding_features)))
    corresponding_features = list(np.argsort(
        np.array(corresponding_features)))
    corresponding_likelihoods.reverse()
    corresponding_features.reverse()
    for i, e in enumerate(corresponding_likelihoods):
        if e == 0:
            corresponding_features[i] = None

    return corresponding_features, corresponding_likelihoods


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
def display(step):
    plt.plot([0, 0, map_width, map_width], [
             0, map_height, 0, map_height], "y*")

    for i, f in enumerate(features):
        plt.plot(f[0], f[1], "bo", markersize=feature_radius)
        # plt.annotate(xy=f, s=i, xytext=[f[0]-2, f[1]+2])

    plt.plot(robot_pose[0], robot_pose[1], "ro", markersize=robot_radius)
    angle = np.linspace(0, 2*pi, 1000)
    x = robot_pose[0] + min_visibility*np.cos(angle)
    y = robot_pose[1] + min_visibility*np.sin(angle)
    plt.plot(x, y, "ro", markersize=0.05)
    x = robot_pose[0] + max_visibility*np.cos(angle)
    y = robot_pose[1] + max_visibility*np.sin(angle)
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
            # plt.annotate(xy=f, s=i, xytext=[f[0]-2, f[1]+2])

            delta = 1
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

    plt.title("Step " + str(step))
    plt.plot()


# Error computation

def position_error(real, particle):
    res = sqrt((real[0] - particle[0])**2 + (real[1] - particle[1])**2)

    return res


def map_error(features, particle):
    if len(particle[4]) == 0:
        return 0

    res = 0
    for f1 in particle[4]:
        best = 1000
        for f2 in features:
            tmp = sqrt((f1[0].transpose()[0][0] - f2[0])**2 +
                       (f1[0].transpose()[0][1] - f2[1])**2)
            if tmp < best:
                best = tmp
        res += best
    res /= len(particle[4])

    return res


# TESTS

if __name__ == '__main__':
    # Init random seeds
    # seed(0)
    # np.random.seed(0)

    # Parameters

    max_time = 50

    # Command
    v = 0.05
    omega = 0
    u = [v, omega]

    # Motion noise
    v_noise = [0.1, 0.01, 0]
    # omega_noise = [0.01, 0.1, 0]
    omega_noise = [0, 0, 0]
    # yaw_noise = [0.01, 0.1, 0]
    yaw_noise = [0, 0, 0]
    motion_noise = [v_noise, omega_noise, yaw_noise]

    # Observation noise
    d_noise = [0, 0, 0.5]
    phi_noise = [0, 0, 0.01]
    observation_noise = [d_noise, phi_noise]

    # Map
    map_width = 30
    map_height = 30

    # Features

    features_num = 14
    # Only for display (features are collapsed to their center of mass)
    feature_radius = 3
    # positions_type = "random"
    positions_type = "forest"

    if positions_type == "random":
        # Random positions
        features = []
        for i in range(features_num):
            new_feature_x = uniform(0, map_width)
            new_feature_y = uniform(0, map_height)
            new_feature = [new_feature_x, new_feature_y]
            features.append(new_feature)

    elif positions_type == "forest":
        # Forest positions
        features = [[0, 25], [5, 20], [10, 25], [15, 20], [20, 25], [25, 20], [30, 25],
                    [0, 5], [5, 10], [10, 5], [15, 10], [20, 5], [25, 10], [30, 5]]

    correspondence_threshold = 10**(-20)

    # Initial robot pose
    initial_x = 0
    initial_y = map_height/2
    initial_theta = 0
    robot_pose = [initial_x, initial_y, initial_theta]
    robot_radius = 10
    min_visibility = 1
    max_visibility = 10
    # print("\nInitial pose: " + str(robot_pose))

    # Particles
    particles_num = 10
    history = History(particles_num)
    particles = []

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

    # Resampling type
    resampling_type = "random"
    # resampling_type = "deterministic"

    # Init display
    # display_frequency = 20
    # lines_num = ceil(sqrt(max_time*0.5 / display_frequency))
    # columns_num = ceil((max_time / display_frequency + 1) / lines_num)
    # subplot_num = 1
    # ax = plt.subplot(lines_num, columns_num, subplot_num)
    # ax.set_xlim(0, map_width)
    # ax.set_ylim(0, map_height)
    # ax.xaxis.set_visible(False)
    # ax.yaxis.set_visible(False)
    # display(0)

    # Simulation

    for t in range(max_time-1):
        start = time.time()

        # Real world simulation
        print("\nStep: " + str(t+1))
        # print("Command: " + str(u))
        robot_pose[0], robot_pose[1], robot_pose[2] = motion(
            "velocity", robot_pose, u, motion_noise)
        # print("New pose: " + str(robot_pose))
        new_observation = observation(
            robot_pose, min_visibility, max_visibility, observation_noise)
        # print("Observation: " + str(new_observation))

        # Particles update
        for j, p in enumerate(particles):
            # print("\nParticle " + str(j) + ":")

            # Prediction
            p[0], p[1], p[2] = motion("velocity", p[:3], u, motion_noise)

            # Correction & mapping
            pose = np.array([[p[0]],
                            [p[1]],
                            [p[2]]])
            # print("Predicted pose: \n" + str(pose))

            features_correspondences = len(new_observation)*[None]
            features_preferences = []
            features_likelihoods = []

            if len(p[4]) > 0:
                correspondences_orders = len(new_observation)*[0]

                for i in range(len(new_observation)):
                    corresponding_features, corresponding_likelihoods = correspondence(
                        p[4], pose, new_observation[i], min_visibility, max_visibility, observation_noise, correspondence_threshold)

                    features_preferences.append(corresponding_features)
                    features_likelihoods.append(corresponding_likelihoods)

                conflicts = dict()
                for i in range(len(features_preferences)):
                    features_correspondences[i] = features_preferences[i][0]
                for i, c in enumerate(features_correspondences):
                    if c != None and features_correspondences.count(c) > 1:
                        if c not in conflicts:
                            conflicts[c] = [i]
                        else:
                            conflicts[c].append(i)

                while conflicts != {}:
                    conflicts_to_pop = []
                    for c in conflicts:
                        likelihoods = [
                            [features_likelihoods[e][correspondences_orders[e]], e] for e in conflicts[c]]
                        tmp = list(np.sort(likelihoods))
                        tmp.reverse()
                        right_feature = tmp[0][1]

                        for i, e in enumerate(conflicts[c]):
                            if e == right_feature:
                                conflicts[c].pop(i)
                            else:
                                new_order = correspondences_orders[e] + 1
                                if new_order < len(features_preferences[e]):
                                    features_correspondences[e] = features_preferences[e][new_order]
                                    correspondences_orders[e] += 1
                                else:
                                    features_correspondences[e] = None

                        if len(conflicts[c]) < 2:
                            conflicts_to_pop.append(c)

                    for c in conflicts_to_pop:
                        conflicts.pop(c)

                    for i, c in enumerate(features_correspondences):
                        if c != None and features_correspondences.count(c) > 1:
                            if c not in conflicts:
                                conflicts[c] = [i]
                            else:
                                conflicts[c].append(i)

                for i, c in enumerate(features_correspondences):
                    assert c == None or features_correspondences.count(
                        c) == 1, "Draw"

            for i in range(len(new_observation)):
                new_z = np.array([[new_observation[i][0]],
                                  [new_observation[i][1]]])
                # print("Observation " + str(i) + ": \n" + str(new_z))

                # Previously unseen feature
                if features_correspondences[i] == None:
                    mu = h_inverse(pose, new_z)
                    # print("Feature estimated position :\n" + str(mu))
                    H1 = H(pose, mu)
                    H1_inverse = np.linalg.inv(H1)
                    Q1 = Q(new_z)
                    sigma = (H1_inverse.dot(Q1)).dot(H1_inverse.transpose())
                    p[4].append([mu, sigma])

                # Already seen feature
                else:
                    mu, sigma = p[4][features_correspondences[i]]
                    f = mu.transpose()[0]
                    predicted_z = h(pose, f)
                    H1 = H(pose, mu)
                    Q1 = (H1.dot(sigma)).dot(
                        H1.transpose()) + Q(new_z)
                    Q1_inverse = np.linalg.inv(Q1)
                    K = (sigma.dot(H1.transpose())).dot(Q1_inverse)
                    new_mu = mu + K.dot(new_z - predicted_z)
                    new_sigma = (np.identity(2) - K.dot(H1)).dot(sigma)
                    p[4][features_correspondences[i]] = [new_mu, new_sigma]
                    tmp = (
                        (new_z - predicted_z).transpose().dot(Q1_inverse)).dot(new_z - predicted_z)[0][0]
                    tmp = max(tmp, -709)
                    weight_update = 1 / \
                        sqrt(2*pi*abs(np.linalg.det(Q1))) * exp(-1/2 * tmp)

                    # Bias to ensure non-zero weight
                    p[3] = max(p[3]*weight_update, 10**(-323))

            # Delete unused features
            if len(features_preferences) > 0:
                features_to_delete = []
                for i in range(len(features_preferences[0])):
                    delete = True
                    for e in features_preferences:
                        if e[i] != 0:
                            delete = False
                    if delete:
                        features_to_delete.append(i)

                old_features = p[4][:]
                p[4] = []
                for i in range(len(old_features)):
                    if i not in features_to_delete:
                        p[4].append(old_features[i][:])

        # Normalization
        weights_sum = 0
        for p in particles:
            weights_sum += p[3]
        assert weights_sum > 0, "/!\ Sum of weights equals 0 !"
        for p in particles:
            p[3] /= weights_sum

        new_particles_set = []
        for j, p in enumerate(particles):
            new_particles_set.append(
                [j, position_error(robot_pose, p), map_error(features, p), p[3]])
        history.add_generation(new_particles_set, "UPDATE")

        # Find most probable pose and features
        max_weight = 0
        for i, p in enumerate(particles):
            if p[3] > max_weight:
                max_weight = p[3]
                most_probable_particle_index = i
        # print("Most probable particle: " + str(most_probable_particle_index))
        # print("Most probable pose: " +
            #   str(particles[most_probable_particle_index][:3]))
        # print("Most probable features: ")
        # for i, f in enumerate(particles[most_probable_particle_index][4]):
            # print(str(i) + ": " + str(f[0].transpose()[0]))

        # Resampling criteria (effective particles number)
        effective_particles_num = 0
        for p in particles:
            effective_particles_num += p[3]**2
        effective_particles_num = 1/effective_particles_num

        # Resampling
        if effective_particles_num < particles_num/2:
            # if False:
            # print("Resampling: Yes")

            old_particles = []
            for p in particles:
                old_particles.append(p[:])
            particles = []
            resampling_sigma = [[0.01, 0, 0],
                                [0, 0.01, 0],
                                [0, 0, 0.0001]]
            survivors = []
            new_particles_set = []

            # Random resampling
            if resampling_type == "random":
                cumulated_weights = [0]
                for p in old_particles:
                    cumulated_weights.append(cumulated_weights[-1] + p[3])

                for i in range(particles_num):
                    r = random()
                    for j in range(len(cumulated_weights)-1):
                        if r >= cumulated_weights[j] and r < cumulated_weights[j+1]:
                            new_particle = list(multivariate_normal.rvs(
                                old_particles[j][:3], resampling_sigma))
                            new_particle.append(1.0/particles_num)
                            new_particle.append(old_particles[j][4][:])
                            if j not in survivors:
                                survivors.append(j)
                            particles.append(new_particle)
                            new_particles_set.append(
                                [j, position_error(robot_pose, new_particle), map_error(features, new_particle), 1.0/particles_num])

                            break

            # Deterministic resampling
            elif resampling_type == "deterministic":
                order = list(np.argsort([p[3] for p in old_particles]))
                order.reverse()

                p_index = 0
                current_weight = old_particles[order[p_index]][3]
                tmp_w = 0.0
                for i in range(particles_num):
                    if tmp_w > current_weight:
                        p_index += 1
                        current_weight += old_particles[order[p_index]][3]

                    new_particle = list(multivariate_normal.rvs(
                        old_particles[order[p_index]][:3], resampling_sigma))
                    new_particle.append(1.0/particles_num)
                    new_particle.append(old_particles[order[p_index]][4][:])

                    if p_index not in survivors:
                        survivors.append(order[p_index])
                    particles.append(new_particle)
                    new_particles_set.append([order[p_index], position_error(
                        robot_pose, new_particle), map_error(features, new_particle), 1.0/particles_num])

                    tmp_w += 1.0/particles_num

            survivors.sort()
            # print("Survivors: " + str(survivors))
            # print("Survivors: " + str(len(survivors)))
            history.add_generation(new_particles_set, "RESAMPLING")

        # else:
            # print("Resampling: No")

        stop = time.time()

        # if (t+1) % display_frequency == 0 or t == max_time-2:
        # subplot_num += 1
        # ax = plt.subplot(lines_num, columns_num, subplot_num)
        # ax.set_xlim(0, map_width)
        # ax.set_ylim(0, map_height)
        # ax.xaxis.set_visible(False)
        # ax.yaxis.set_visible(False)
        # display(t+1)

        # print("Time: " + str(stop-start))

    # h_outer_space = 0.02
    # v_outer_space = 0.04
    # inner_space = 0.2
    # plt.subplots_adjust(left=h_outer_space,
        # bottom=v_outer_space,
        # right=1-h_outer_space,
        # top=1-v_outer_space,
        # wspace=inner_space,
        # hspace=inner_space)
    # plt.show()

    # print("\nHistory of particles set:")
    history.add_most_probable_particle(most_probable_particle_index)

    # print(history)
    history.display()
