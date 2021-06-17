from random import uniform, normalvariate, random
import matplotlib.pyplot as plt
from math import pi, cos, sin, sqrt, erf, atan2, acos
import numpy as np
from scipy.stats import multivariate_normal


# Motion model

def predict(pose, command, noise):

    #  Velocity-based (dead reckoning)

    # linear_velocity_noise = noise[0][0]*command[0] + noise[0][1]*command[1]
    # noisy_linear_velocity = command[0] + \
    #     normalvariate(0, linear_velocity_noise)
    # angular_velocity_noise = noise[1][0]*command[0] + noise[1][1]*command[1]
    # noisy_angular_velocity = command[1] + \
    #     normalvariate(0, angular_velocity_noise)
    # heading_noise = noise[2][0]*command[0] + noise[2][1]*command[1]
    # noisy_heading = normalvariate(0, heading_noise)

    # new_x = pose[0] - noisy_linear_velocity/noisy_angular_velocity * \
    #     sin(pose[2]) + noisy_linear_velocity/noisy_angular_velocity * \
    #     sin(pose[2] + noisy_angular_velocity)
    # new_y = pose[1] + noisy_linear_velocity/noisy_angular_velocity * \
    #     cos(pose[2]) - noisy_linear_velocity/noisy_angular_velocity * \
    #     cos(pose[2] + noisy_angular_velocity)
    # new_h = pose[2] + noisy_angular_velocity + noisy_heading

    new_x = pose[0] - command[0]/command[1] * \
        sin(pose[2]) + command[0]/command[1]*sin(pose[2] + command[1])
    new_y = pose[1] + command[0]/command[1] * \
        cos(pose[2]) - command[0]/command[1]*cos(pose[2] + command[1])
    new_theta = pose[2] + command[1]

    x_sigma = x_noise[0]*command[0] + x_noise[1]*command[1]
    y_sigma = y_noise[0]*command[0] + y_noise[1]*command[1]
    theta_sigma = theta_noise[0]*command[0] + theta_noise[1]*command[1]
    sigma = [[x_sigma, 0, 0],
             [0, y_sigma, 0],
             [0, 0, theta_sigma]]

    new_pose = multivariate_normal.rvs([new_x, new_y, new_theta], sigma)
    if new_pose[2] <= -pi:
        new_pose[2] += 2*pi
    elif new_pose[2] > pi:
        new_pose[2] -= 2*pi

    return new_pose


# Observation model

def observation(pose, visibility, noise):
    observation = []
    for i, f in enumerate(features):
        distance = sqrt((f[0] - pose[0])**2 + (f[1] - pose[1])**2)
        angle = atan2(f[1] - pose[1], f[0] - pose[0]) - pose[2]

        if distance < visibility:
            d_sigma = d_noise[0]*distance + d_noise[1]*angle
            a_sigma = a_noise[0]*distance + a_noise[1]*angle
            sigma = [[d_sigma, 0],
                     [0, a_sigma]]

            noisy_observation = multivariate_normal.rvs(
                [distance, angle], sigma)
            if noisy_observation[1] <= -pi:
                noisy_observation[1] += 2*pi
            elif noisy_observation[1] > pi:
                noisy_observation -= 2*pi
            observation.append(list(noisy_observation))
    return observation


def likelihood(features, pose, visibility, observation, noise):
    likelihood = 1

    visible_features = []
    for i, f in enumerate(features):
        distance = sqrt((f[0] - pose[0])**2 + (f[1] - pose[1])**2)
        angle = atan2(f[1] - pose[1], f[0] - pose[0]) - pose[2]
        if distance < visibility:
            visible_features.append([i, [distance, angle]])

    if visible_features == [] and observation != []:
        return 0

    for o in observation:
        highest_likelihood = 0
        for i, [d, a] in visible_features:
            d_sigma = d_noise[0]*d + d_noise[1]*a
            a_sigma = a_noise[0]*d + a_noise[1]*a
            sigma = [[d_sigma, 0],
                     [0, a_sigma]]

            if o[1] > 0 and a < 0:
                a += 2*pi
            elif o[1] < 0 and a > 0:
                a -= 2*pi
            feature_likelihood = multivariate_normal.pdf(o, [d, a], sigma)
            if feature_likelihood > highest_likelihood:
                highest_likelihood = feature_likelihood
                corresponding_feature = i

        if highest_likelihood == 0:
            return 0
        likelihood *= highest_likelihood
        for i, [j, [d, a]] in enumerate(visible_features):
            if j == corresponding_feature:
                visible_features.pop(i)

    if visible_features != []:
        return 0

    return likelihood


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

    plt.plot()


# TESTS

# Parameters

max_time = 8

# Command
linear_velocity = 5
angular_velocity = 0.1
command = [linear_velocity, angular_velocity]

# Motion noise
x_noise = [0.2, 0.2]
y_noise = [0.2, 0.2]
h_noise = [0.2, 0.2]
motion_noise = [x_noise, y_noise, h_noise]

# Observation noise
d_noise = [0.05, 0]
a_noise = [0.005, 0]
observation_noise = [d_noise, a_noise]

# Map

map_width = 100
map_height = 100

# Features

features_num = 10
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
initial_h = 0
robot_pose = [initial_x, initial_y, initial_h]
robot_radius = 10
visibility = 30
print("\nInitial pose: " + str(robot_pose))

# Particles

# Store lengths of sides of all triangles in map
features_triangles = dict()
for i in range(features_num):
    for j in range(features_num):
        for k in range(features_num):
            if i != j and j != k and k != i:
                f1 = features[i]
                f2 = features[j]
                f3 = features[k]
                first_side = sqrt((f1[0] - f2[0])**2 + (f1[1] - f2[1])**2)
                second_side = sqrt((f2[0] - f3[0])**2 + (f2[1] - f3[1])**2)
                third_side = sqrt((f3[0] - f1[0])**2 + (f3[1] - f1[1])**2)
                first_angle = acos(
                    (first_side**2 - second_side**2 + third_side**2) / (2*first_side*third_side))
                second_angle = acos(
                    (second_side**2 - third_side**2 + first_side**2) / (2*second_side*first_side))
                third_angle = acos(
                    (third_side**2 - first_side**2 + second_side**2) / (2*third_side*second_side))
                features_triangles[k + j*features_num + i *
                                   features_num**2] = [first_side, second_side, third_side, first_angle, second_angle, third_angle]

# Initial observation
initial_observation = observation(robot_pose, visibility, observation_noise)
assert len(initial_observation) >= 3, "/!\ 3 features are necessary to initialize particles ! (only " + \
    str(len(initial_observation)) + " were found)"
print("Initial observation: " + str(initial_observation))

# Extract individul observations and compute triangle's sides
d1, a1 = initial_observation[0]
d2, a2 = initial_observation[1]
d3, a3 = initial_observation[2]
first_side = sqrt(d1**2 + d2**2 - 2*d1*d2*cos(a2-a1))
second_side = sqrt(d2**2 + d3**2 - 2*d2*d3*cos(a3-a2))
third_side = sqrt(d3**2 + d1**2 - 2*d3*d1*cos(a1-a3))
first_angle = acos((first_side**2 - second_side**2 +
                   third_side**2) / (2*first_side*third_side))
second_angle = acos((second_side**2 - third_side**2 +
                    first_side**2) / (2*second_side*first_side))
third_angle = acos((third_side**2 - first_side**2 +
                   second_side**2) / (2*third_side*second_side))

# Find most similar triangle in map
near_triangles = []
distance_threshold = 50
for t in features_triangles:
    triangle_values = features_triangles[t]
    distance = (first_side - triangle_values[0])**2 + (
        second_side - triangle_values[1])**2 + (third_side - triangle_values[2])**2
    first_angle_delta = first_angle - triangle_values[3]
    second_angle_delta = second_angle - triangle_values[4]
    third_angle_delta = third_angle - triangle_values[5]
    for a in [first_angle_delta, second_angle_delta, third_angle_delta]:
        if a <= -pi:
            a += 2*pi
        elif a > pi:
            a -= 2*pi
    distance += first_angle_delta**2 + second_angle_delta**2 + third_angle_delta**2
    if distance < distance_threshold:
        near_triangles.append(t)

# Compute estimated pose from triangle matching and observation
estimated_poses = []
for t in near_triangles:
    first_feature = t / features_num**2
    second_feature = (t - first_feature *
                      features_num**2) / features_num
    third_feature = t - first_feature * \
        features_num**2 - second_feature*features_num
    print("Estimated features: " +
          str([first_feature, second_feature, third_feature]))
    x1, y1 = features[first_feature]
    x2, y2 = features[second_feature]
    x3, y3 = features[third_feature]
    assert (x2-x1) * (y3-y2) != (x3-x2) * \
        (y2-y1), "/!\ Estimated pose can't be computed !"
    estimated_pose_x = ((y2-y1) * (d3**2 - x3**2 - y3**2) + (y3-y2) * (d1**2 - x1**2 - y1**2) +
                        (y1-y3) * (d2**2 - x2**2 - y2**2)) / (2 * ((x2-x1) * (y3-y2) - (x3-x2)*(y2-y1)))
    estimated_pose_y = ((x3-x2) * (d2**2 - d1**2 + x1**2 - x2**2 + y1**2 - y2**2) - (x1-x2) * (
        d2**2 - d3**2 + x3**2 - x2**2 + y3**2 - y2**2)) / (2 * ((x2-x1) * (y3-y2) - (x3-x2)*(y2-y1)))
    estimated_pose_h = (atan2(y1 - estimated_pose_y, x1 - estimated_pose_x) - a1 + atan2(y2 - estimated_pose_y,
                        x2 - estimated_pose_x) - a2 + atan2(y3 - estimated_pose_y, x3 - estimated_pose_x) - a3) / 3
    if estimated_pose_h <= -pi:
        estimated_pose_h += 2*pi
    elif estimated_pose_h > pi:
        estimated_pose_h -= 2*pi
    estimated_poses.append(
        [estimated_pose_x, estimated_pose_y, estimated_pose_h])
    print("Estimated pose: " + str(estimated_poses[-1]))

# Sample particles with gaussian distribution around estimated pose
particles_num = 100
particles = []
particles_sigma = [[10, 0, 0],
                   [0, 10, 0],
                   [0, 0, 0.1]]

# for p in estimated_poses:
#     for i in range(particles_num/len(estimated_poses)):
#         new_particle_x, new_particle_y, new_particle_h = multivariate_normal.rvs(
#             p, particles_sigma)
#         new_particle_w = 1.0/particles_num
#         new_particle = [new_particle_x, new_particle_y,
#                         new_particle_h, new_particle_w]
#         particles.append(new_particle)

for i in range(particles_num):
    # Normal distribution
    new_particle_x, new_particle_y, new_particle_h = multivariate_normal.rvs(
        robot_pose, particles_sigma)

    # Uniform distribution
    # new_particle_x = uniform(0, map_width)
    # new_particle_y = uniform(0, map_height)
    # new_particle_h = uniform(-pi, pi)
    # new_particle_h = robot_pose[2]

    new_particle_w = 1.0/particles_num
    new_particle = [new_particle_x, new_particle_y,
                    new_particle_h, new_particle_w]
    particles.append(new_particle)

# for p in particles:
#     p[3] *= likelihood(features, p[:3], visibility,
#                        initial_observation, observation_noise)

# cumulated_weights = [0]
# for p in particles:
#     cumulated_weights.append(cumulated_weights[-1] + p[3])

# weights_sum = 0
# for p in particles:
#     weights_sum += p[3]
# assert weights_sum > 0, "/!\ Sum of weights equals 0 !"
# for p in particles:
#     p[3] /= weights_sum

# max_weight = 0
# for p in particles:
#     if p[3] > max_weight:
#         max_weight = p[3]
#         most_probable_pose = p[:3]
# print("Most probable pose: " + str(most_probable_pose))

# old_particles = []
# for p in particles:
#     old_particles.append(p[:])
# particles = []
# for i in range(particles_num):
#     r = random()
#     for j in range(len(cumulated_weights)-1):
#         if r >= cumulated_weights[j] and r < cumulated_weights[j+1]:
#             resampling_sigma = [[5, 0, 0],
#                                 [0, 5, 0],
#                                 [0, 0, 0.05]]
#             new_particle = list(multivariate_normal.rvs(
#                 old_particles[j][:3], resampling_sigma))
#             new_particle.append(1.0/particles_num)
#             particles.append(new_particle)

# initial_observation = observation(robot_pose, visibility, observation_sigma)
# print("\nInitial observation: " + str(initial_observation))

# visible_features_list = []
# for p in particles:
#     visible_features = dict()
#     for i, f in enumerate(features):
#         distance = sqrt((f[0] - p[0])**2 + (f[1] - p[1])**2)
#         angle = atan2(f[1] - p[1], f[0] - p[0]) - p[2]
#         if distance < visibility:
#             visible_features[i] = [distance, angle]
#     visible_features_list.append(visible_features)

# features_correspondences = []
# for o in initial_observation:
#     features_likelihoods = dict()
#     for i in range(len(features)):
#         features_likelihoods[i] = 0

#     for e in visible_features_list:
#         for i in e:
#             feature_likelihood = multivariate_normal.pdf(
#                 o, e[i], observation_sigma)
#             features_likelihoods[i] += feature_likelihood

#     print("Observation: " + str(o))
#     print("Features likelihoods: " + str(features_likelihoods))

#     highest_likelihood = 0
#     for i in features_likelihoods:
#         if features_likelihoods[i] > highest_likelihood:
#             highest_likelihood = features_likelihoods[i]
#             corresponding_feature = i
#     features_correspondences.append(corresponding_feature)

# print("Features correspondences: " + str(features_correspondences))

# for p in particles:
#     particle_likelihood = 1

#     for i, o in enumerate(initial_observation):
#         f = features[features_correspondences[i]]
#         distance = sqrt((f[0] - p[0])**2 + (f[1] - p[1])**2)
#         angle = atan2(f[1] - p[1], f[0] - p[0]) - p[2]

#         if distance < visibility:
#             feature_likelihood = multivariate_normal.pdf(
#                 o, [distance, angle], observation_sigma)
#             particle_likelihood *= feature_likelihood
#         else:
#             particle_likelihood = 0
#     p[3] = particle_likelihood

# # Normalization
# weights_sum = 0
# for p in particles:
#     weights_sum += p[3]
# assert weights_sum > 0, "/!\ Sum of weights equals 0 !"
# for p in particles:
#     p[3] /= weights_sum

# # Resampling
# cumulated_weights = [0]
# for p in particles:
#     cumulated_weights.append(cumulated_weights[-1] + p[3])

# old_particles = []
# for p in particles:
#     old_particles.append(p[:])
# particles = []
# for i in range(particles_num):
#     r = random()
#     for j in range(len(cumulated_weights)-1):
#         if r >= cumulated_weights[j] and r < cumulated_weights[j+1]:
#             resampling_sigma = [[500, 0, 0],
#                                 [0, 500, 0],
#                                 [0, 0, 5]]
#             new_particle = list(multivariate_normal.rvs(
#                 old_particles[j][:3], resampling_sigma))
#             new_particle.append(1.0/particles_num)
#             particles.append(new_particle)

# Display

subplot = 200 + max_time/2*10 + 1
plt.subplot(subplot)
display()

# Simulation

for i in range(max_time-1):
    # Real world simulation
    print("\nTemps: " + str(i+1))
    print("Command: " + str(command))
    robot_pose[0], robot_pose[1], robot_pose[2] = predict(
        robot_pose, command, motion_noise)
    print("New pose: " + str(robot_pose))
    new_observation = observation(robot_pose, visibility, observation_noise)
    print("Observation: " + str(new_observation))

    for p in particles:
        # Prediction
        p[0], p[1], p[2] = predict(p, command, motion_noise)

        # Correction
        weight_update = likelihood(features,
                                   p, visibility, new_observation, observation_noise)
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
                    resampling_sigma = [[5, 0, 0],
                                        [0, 5, 0],
                                        [0, 0, 0.05]]
                    new_particle = list(multivariate_normal.rvs(
                        old_particles[j][:3], resampling_sigma))
                    new_particle.append(1.0/particles_num)
                    particles.append(new_particle)
    else:
        print("Resampling: No")

    subplot += 1
    plt.subplot(subplot)
    display()

print("\n")
plt.show()
