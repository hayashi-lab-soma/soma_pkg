from random import uniform, normalvariate, random
import matplotlib.pyplot as plt
from math import pi, cos, sin, sqrt, erf, atan2, acos
import numpy as np
from scipy.stats import multivariate_normal
import time


# Motion model (velocity-based or dead reckoning)
# TODO: Odometry-based motion model

def motion(pose, command, noise):
    x, y, theta = pose
    v, omega = command
    v_noise, omega_noise, yaw_noise = noise

    # Covariance matrix
    v_sigma = v_noise[0]*v + v_noise[1]*omega
    omega_sigma = omega_noise[0]*v + omega_noise[1]*omega
    yaw_sigma = yaw_noise[0]*v + yaw_noise[1]*omega
    sigma = [[v_sigma, 0],
             [0, omega_sigma]]

    # Noisy commands
    v, omega = multivariate_normal.rvs(command, sigma)

    # New pose
    new_x = pose[0] - v/omega * \
        sin(pose[2]) + v/omega*sin(pose[2] + omega)
    new_y = pose[1] + v/omega * \
        cos(pose[2]) - v/omega*cos(pose[2] + omega)
    new_theta = multivariate_normal.rvs(pose[2] + omega, yaw_sigma)

    # Ensure theta in ]-pi;pi]
    if new_theta <= -pi:
        new_theta += 2*pi
    elif new_theta > pi:
        new_theta -= 2*pi

    new_pose = [new_x, new_y, new_theta]

    return new_pose


# Observation model (LIDAR)
# TODO: observation model for other sensors

def observation(pose, visibility, noise):
    x, y, theta = pose
    d_noise, phi_noise = noise

    z = []

    for f in features:
        # Distance and azimut to feature
        xf, yf = f
        d = sqrt((xf - x)**2 + (yf - y)**2)
        phi = atan2(yf - y, xf - x) - theta

        # Feature detected only if within the sensor visibility circle
        if d < visibility:
            # Covariance matrix
            d_sigma = d_noise[0]*d + d_noise[1]*phi
            phi_sigma = theta_noise[0]*d + theta_noise[1]*phi
            sigma = [[d_sigma, 0],
                     [0, phi_sigma]]

            # Sensor data
            d, phi = multivariate_normal.rvs([d, phi], sigma)

            # Ensure phi in ]-pi;pi]
            if phi <= -pi:
                phi += 2*pi
            elif phi > pi:
                phi -= 2*pi

            z.append([d, phi])

    return z


# Find correspondence between individual observation and feature (with distance and angle to features)

def matching(visible_features, zi, noise):
    d_noise, phi_noise = noise

    # If no more visible features to match to individual observation, return None
    if visible_features == []:
        return None

    highest_likelihood = 0

    for i, [d, phi] in visible_features:
        # Covariance matrix
        d_sigma = d_noise[0]*d + d_noise[1]*phi
        phi_sigma = phi_noise[0]*d + phi_noise[1]*phi
        sigma = [[d_sigma, 0],
                 [0, phi_sigma]]

        # Ensure zi and phi are of the same sign (to be able to compare them)
        if zi[1] > 0 and phi < 0:
            phi += 2*pi
        elif zi[1] < 0 and phi > 0:
            phi -= 2*pi

        # Corresponding feature is the one maximizing particle likelihood
        feature_likelihood = multivariate_normal.pdf(zi, [d, phi], sigma)
        if feature_likelihood > highest_likelihood:
            highest_likelihood = feature_likelihood
            corresponding_feature = list(visible_features[i])

    visible_features.pop(i)

    return corresponding_feature


# Find correspondence between truncated observation and triangle of features (with triangle matching)

def triangleMatching(features_triangles, z, noise):
    assert len(
        z) >= 3, "/!\ Triangle matching requires at least 3 individual observations !"

    # Extract first 3 individual observations
    d1, a1 = z[0]
    d2, a2 = z[1]
    d3, a3 = z[2]

    # Compute distances and angles of triangle
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
    # near_triangles = []
    smallest_distance = 1000000000
    # distance_threshold = 50
    for t in features_triangles:
        triangle_values = features_triangles[t]

        # Compute distance between distances and angles
        # TODO: Normalize distances because angular distances much smaller than linear distances !
        distance = (first_side - triangle_values[0])**2 + (
            second_side - triangle_values[1])**2 + (third_side - triangle_values[2])**2

        if first_angle > 0 and triangle_values[3] < 0:
            first_angle -= 2*pi
        elif first_angle < 0 and triangle_values[3] > 0:
            first_angle += 2*pi
        first_angle_delta = first_angle - triangle_values[3]

        if second_angle > 0 and triangle_values[4] < 0:
            second_angle -= 2*pi
        elif second_angle < 0 and triangle_values[4] > 0:
            second_angle += 2*pi
        second_angle_delta = second_angle - triangle_values[4]

        if third_angle > 0 and triangle_values[5] < 0:
            third_angle -= 2*pi
        elif third_angle < 0 and triangle_values[5] > 0:
            third_angle += 2*pi
        third_angle_delta = third_angle - triangle_values[5]

        distance += first_angle_delta**2 + second_angle_delta**2 + third_angle_delta**2

        # if distance < distance_threshold:
        if distance < smallest_distance:
            nearest_triangle = list(t)
            smallest_distance = distance
            # near_triangles.append(t)

    # return near_triangles
    return nearest_triangle


# Likelihood for individual observation

def individual_likelihood(corresponding_feature, pose, zi, noise):
    x, y, theta = pose
    d_noise, phi_noise = noise

    if corresponding_feature == None:
        return 0

    i, [d, phi] = corresponding_feature

    # Covariance matrix
    d_sigma = d_noise[0]*d + d_noise[1]*phi
    phi_sigma = phi_noise[0]*d + phi_noise[1]*phi
    sigma = [[d_sigma, 0],
             [0, phi_sigma]]

    # Ensure zi and phi are of the same sign (to be able to compare them)
    if zi[1] > 0 and phi < 0:
        phi += 2*pi
    elif zi[1] < 0 and phi > 0:
        phi -= 2*pi

    # Compute likelihood
    likelihood = multivariate_normal.pdf(zi, [d, phi], sigma)

    return likelihood


# Overall likelihood

def global_likelihood(features, pose, visibility, z, noise):
    likelihood = 1

    # Consider only features within sensor visibility circle (to be changed ?)
    visible_features = []
    for i, f in enumerate(features):
        # Distance and azimut to feature
        xf, yf = f
        d = sqrt((xf - x)**2 + (yf - y)**2)
        phi = atan2(yf - y, xf - x) - theta

        if d < visibility:
            visible_features.append([i, [d, phi]])

    # For each individual observation (per feature)
    for zi in z:
        # Find correspondence between individual observation and feature
        corresponding_feature = matching(
            visible_features, pose, visibility, zi, noise)

        # Compute individual likelihood
        # Individual likelihood are multiplied to get global likelihood (independence assumption)
        likelihood *= individual_likelihood(
            corresponding_feature, pose, zi, noise)

    # If there are still unmatched visible features, likelihood equals 0
    if visible_features != []:
        likelihood = 0

    return likelihood


# Display

def display():
    # Map
    plt.plot([0, 0, map_width, map_width], [
             0, map_height, 0, map_height], "y*")

    # Features
    for i, f in enumerate(features):
        plt.plot(f[0], f[1], "bo", markersize=feature_radius)
        plt.annotate(xy=f, s=i, xytext=[f[0]-2, f[1]+2])

    # Robot
    plt.plot(robot_pose[0], robot_pose[1], "ro", markersize=robot_radius)
    angle = np.linspace(0, 2*pi, 1000)
    x = robot_pose[0] + visibility*np.cos(angle)
    y = robot_pose[1] + visibility*np.sin(angle)
    plt.plot(x, y, "ro", markersize=0.05)
    plt.arrow(robot_pose[0], robot_pose[1], 10 *
              cos(robot_pose[2]), 10*sin(robot_pose[2]), "ro")

    # Particles
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
v_noise = [0.01, 0]
omega_noise = [0, 0.01]
yaw_noise = [0, 0.01]
motion_noise = [v_noise, omega_noise, yaw_noise]

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
z = observation(robot_pose, visibility, observation_noise)
print("Initial observation: " + str(initial_observation))
near_triangles = [triangleMatching(features_triangles, z, observation_noise)]

# Compute estimated pose from triangle matching and initial observation
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
        (y2-y1), "/!\ Estimated pose can't be computed (division by 0) !"
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

# Init display
subplot = 200 + max_time/2*10 + 1
plt.subplot(subplot)
display()

# Simulation
for i in range(max_time-1):
    start = time.time()

    # Real world simulation
    print("\nStep: " + str(i+1))
    print("Command: " + str(command))
    robot_pose[0], robot_pose[1], robot_pose[2] = motion(
        robot_pose, command, motion_noise)
    print("New pose: " + str(robot_pose))
    new_observation = observation(robot_pose, visibility, observation_noise)
    print("Observation: " + str(new_observation))

    # Particles update

    for p in particles:
        # Prediction
        p[0], p[1], p[2] = motion(p[:3], command, motion_noise)

        # Correction
        weight_update = global_likelihood(features,
                                          p, visibility, new_observation, observation_noise)
        p[3] *= weight_update

    # Normalization
    weights_sum = 0
    for p in particles:
        weights_sum += p[3]
    assert weights_sum > 0, "/!\ Sum of weights equals 0 !"
    for p in particles:
        p[3] /= weights_sum

    # Find most probable pose
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

    stop = time.time()
    print("Time: " + str(stop-start))

print("\n")
plt.show()
