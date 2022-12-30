from math import cos, sin, sqrt, atan2, pi
from itertools import permutations
from random import random, randint
from scipy.stats import multivariate_normal
from time import time
from matplotlib import pyplot as plt
import numpy as np
from scipy.stats.mvn import mvnun


def find_polygons(observation, map, threshold):
    distances = []
    for i in range(len(observation)):
        o1 = observation[i]
        o2 = observation[(i+1) % len(observation)]
        d = o1[0]**2 + o2[0]**2 - 2*o1[0]*o2[0]*cos(o1[1]-o2[1])
        distances.append(d)

    polygons = list(permutations(range(len(map)), len(observation)))

    possible_polygons = []
    for p in polygons:
        error = 0
        for i in range(len(p)):
            t1 = map[p[i]]
            t2 = map[p[(i+1) % len(p)]]
            d = (t1[0] - t2[0])**2 + \
                (t1[1] - t2[1])**2
            error += (sqrt(distances[i]) - sqrt(d))**2
            if error > threshold:
                break
        if error <= threshold:
            possible_polygons.append([p, error])

    return possible_polygons


def find_pose(observation, map, polygon):
    poses = []
    x, y = 0, 0
    for i in range(len(polygon)):
        # Choose 3 vertices of polygon
        x1, y1 = map[polygon[i]]
        d1, phi1 = observation[i]
        x2, y2 = map[polygon[(i+1) % len(polygon)]]
        d2, phi2 = observation[(i+1) % len(polygon)]
        x3, y3 = map[polygon[(i+2) % len(polygon)]]
        d3, phi3 = observation[(i+2) % len(polygon)]

        d = sqrt((x2-x1)**2 + (y2-y1)**2)

        # Find possible position
        if d > d1+d2:
            tmp1, tmp2 = d1, d2
            d1 += (d-tmp1-tmp2)/2
            d2 += (d-tmp1-tmp2)/2

        a = (d1**2 - d2**2 + d**2) / (2*d)
        h = sqrt(abs(d1**2 - a**2))
        x_ = x1 + a/d*(x2-x1)
        y_ = y1 + a/d*(y2-y1)

        x_1 = x_ - h*(y2-y1)/d
        y_1 = y_ + h*(x2-x1)/d
        x_2 = x_ + h*(y2-y1)/d
        y_2 = y_ - h*(x2-x1)/d

        d_1 = sqrt((x_1-x3)**2 + (y_1-y3)**2)
        d_2 = sqrt((x_2-x3)**2 + (y_2-y3)**2)
        if (d_1-d3)**2 <= (d_2-d3)**2:
            x += x_1
            y += y_1
        else:
            x += x_2
            y += y_2

    x /= len(polygon)
    y /= len(polygon)

    # Find possible attitude
    theta, first_theta = 0, 0
    for i, t in enumerate(polygon):
        tmp = atan2(map[t][1]-y, map[t][0]-x) - observation[i][1]
        if tmp <= -pi:
            tmp += 2*pi
        elif tmp > pi:
            tmp -= 2*pi
        if i == 0:
            first_theta = tmp
        else:
            if abs(tmp - first_theta) > abs(tmp - first_theta + 2*pi):
                tmp += 2*pi
            elif abs(tmp - first_theta) > abs(tmp - first_theta - 2*pi):
                tmp -= 2*pi
        theta += tmp
    theta /= len(polygon)

    # Compute fitness
    fitness = 0
    for i, t in enumerate(polygon):
        fitness += ((observation[i][0] - sqrt((map[t][0]-x)
                    ** 2 + (map[t][1]-y)**2))/observation[i][0])**2
        tmp = atan2(map[t][1]-y, map[t][0]-x) + theta
        if abs(tmp - first_theta) > abs(tmp - first_theta + 2*pi):
            tmp += 2*pi
        elif abs(tmp - first_theta) > abs(tmp - first_theta - 2*pi):
            tmp -= 2*pi
        fitness += ((observation[i][1] - tmp)/observation[i][1])**2

    poses.append([[x, y, theta], fitness])

    return poses


def sample_pose(poses, d_sigma, phi_sigma):
    fitness_total = 0
    for p in poses:
        fitness_total += 1/p[1]
    sum = 0
    cum_weights = []
    for p in poses:
        cum_weights.append(sum + 1/p[1]/fitness_total)
        sum = cum_weights[-1]

    sigma = [[2*d_sigma, 0, 0],
             [0, 2*d_sigma, 0],
             [0, 0, phi_sigma]]

    r = random()
    for i, p in enumerate(poses):
        if r <= cum_weights[i]:
            pose = multivariate_normal.rvs(p[0], sigma)

            return pose


def compute_weight(final_pose, odom, particles, rot1_noise, trans_noise, rot2_noise):
    w = 0
    x2, y2, theta2 = final_pose
    real_rot1, real_trans, real_rot2 = odom
    for p in particles:
        initial_pose, weight = p
        x1, y1, theta1 = initial_pose

        rot1 = atan2(y2-y1, x2-x1) - theta1
        trans = sqrt((x2-x1)**2 + (y2-y1)**2)
        if rot1 <= -pi/2:
            trans *= -1
            rot1 += pi
        elif rot1 > pi/2:
            trans *= -1
            rot1 -= pi
        rot2 = theta2 - theta1 - rot1

        rot1_sigma = rot1_noise[0] * abs(trans) + \
            rot1_noise[1] * abs(rot1) + rot1_noise[2]
        trans_sigma = trans_noise[0] * abs(trans) + trans_noise[1] * \
            (abs(rot1) + abs(rot2)) + trans_noise[2]
        rot2_sigma = rot2_noise[0] * abs(trans) + \
            rot2_noise[1] * abs(rot2) + rot2_noise[2]
        sigma = [[rot1_sigma, 0, 0],
                 [0, trans_sigma, 0],
                 [0, 0, rot2_sigma]]

        rot1_interval, trans_interval, rot2_interval = rot1_sigma / \
            10, trans_sigma/10, rot2_sigma/10
        likelihood = mvnun(np.array([real_rot1-rot1_interval/2.0, real_trans-trans_interval/2.0, real_rot2-rot2_interval/2.0]), np.array(
            [real_rot1+rot1_interval/2.0, real_trans+trans_interval/2.0, real_rot2+rot2_interval/2.0]), np.array([rot1, trans, rot2]), np.array(sigma))[0]

        w += likelihood*weight

    return w


if __name__ == '__main__':
    map = [[4, 5], [3, 4], [1, 2], [7, 3], [8, 7],
           [1, 5], [2, 4], [6, 5], [0, 2], [8, 3]]
    old_real = [4, 4, 0]
    old_x, old_y, old_theta = old_real
    num = 20
    old_particles = []
    for i in range(num):
        old_particles.append([old_real, 1.0/num])
    odom = [0, 1, 0]
    rot1, trans, rot2 = odom
    rot1_noise = [0.005, 0.05, 0.0]
    trans_noise = [0.02, 0.005, 0.0]
    rot2_noise = [0.005, 0.05, 0.0]
    rot1_sigma = rot1_noise[0] * abs(trans) + \
        rot1_noise[1] * abs(rot1) + rot1_noise[2]
    trans_sigma = trans_noise[0] * abs(trans) + trans_noise[1] * \
        (abs(rot1) + abs(rot2)) + trans_noise[2]
    rot2_sigma = rot2_noise[0] * abs(trans) + \
        rot2_noise[1] * abs(rot2) + rot2_noise[2]
    sigma = [[rot1_sigma, 0, 0],
             [0, trans_sigma, 0],
             [0, 0, rot2_sigma]]
    noisy_odom = multivariate_normal.rvs([rot1, trans, rot2], sigma)
    real_x = old_x + trans*cos(old_theta + rot1)
    real_y = old_y + trans*sin(old_theta + rot1)
    real_theta = old_theta + rot1 + rot2
    real = [real_x, real_y, real_theta]
    d_noise = [0.05, 0, 0]
    phi_noise = [0.0005, 0, 0]
    seen = []
    vertices_num = 3
    for i in range(vertices_num):
        seen.append(randint(0, len(map)-1))
    print(seen)
    observation = []
    for i in seen:
        t = map[i]
        d = sqrt((t[0]-real[0])**2 + (t[1]-real[1])**2)
        phi = atan2(t[1]-real[1], t[0]-real[0]) - real[2]
        d_sigma = d_noise[0]*d + d_noise[1]*abs(phi) + d_noise[2]
        phi_sigma = phi_noise[0]*d + phi_noise[1]*abs(phi) + phi_noise[2]
        sigma = [[d_sigma, 0],
                 [0, phi_sigma]]
        observation.append(multivariate_normal.rvs([d, phi], sigma))
        if observation[-1][1] <= -pi:
            observation[-1][1] += 2*pi
        elif observation[-1][1] > pi:
            observation[-1][1] -= 2*pi
    # threshold = 0.2*len(seen)
    threshold = 5

    start = int(time() * 1000)
    polygons = find_polygons(observation, map, threshold)
    for p in polygons:
        print(p)
    # print("Number of polygons: " + str(len(polygons)))
    possible_poses = []
    for p in polygons:
        possible_poses += find_pose(observation, map, p[0])
        # if list(p[0]) == seen:
        # print("/!\: " + str(possible_poses[-1]))
    particles = []
    d_sigma, phi_sigma = 0, 0
    for o in observation:
        d, phi = o
        d_sigma += d_noise[0]*d + d_noise[1]*abs(phi) + d_noise[2]
        phi_sigma += phi_noise[0]*d + phi_noise[1]*abs(phi) + phi_noise[2]
    d_sigma /= len(observation)
    phi_sigma /= len(observation)
    weight_sum = 0
    for i in range(num):
        new_pose = sample_pose(possible_poses, d_sigma, phi_sigma)
        new_weight = max(compute_weight(
            new_pose, odom, old_particles, rot1_noise, trans_noise, rot2_noise), 10**(-323))
        weight_sum += new_weight
        particles.append([new_pose[:], new_weight])
    for p in particles:
        p[1] /= weight_sum
    print("Time: " + str(int(time() * 1000) - start) + " ms")
    # print("Particles:")
    # for p in particles:
    # print(str(p[0]), p[1])

    array_length = 0.3
    for i, t in enumerate(map):
        plt.plot(t[0], t[1], 'k.', markersize=15)
        plt.text(t[0]-0.05, t[1]+0.1, i, fontsize=10)
    plt.plot(real[0], real[1], 'g.', markersize=20)
    # plt.plot([real[0], real[0] + array_length*cos(real[2])],
    #  [real[1], real[1] + array_length*sin(real[2])], 'g-')
    for p in polygons:
        for i in range(len(p[0])):
            t1 = map[p[0][i]]
            t2 = map[p[0][(i+1) % len(p[0])]]
            plt.plot([t1[0], t2[0]], [t1[1], t2[1]], 'b-')
    for p in particles:
        plt.plot(p[0][0], p[0][1], 'r.', markersize=max(min(p[1]*50, 20), 5))
    # for p in particles:
        # plt.plot([p[0][0], p[0][0] + array_length*cos(p[0][2])],
        #  [p[0][1], p[0][1] + array_length*sin(p[0][2])], 'r-')
    plt.show()
