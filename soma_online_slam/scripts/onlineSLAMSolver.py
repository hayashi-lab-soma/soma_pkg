import numpy as np
from math import pi, sqrt
from random import random
from scipy.stats import multivariate_normal
from scipy.stats.mvn import mvnun
from utils.fastslam import motion, correspondence, h_inverse, h, H, Q


# Feature: distinctive object that we can clearly distinguish from others
class Feature:
    def __init__(self, pose, sigma):
        self.pose = np.array([pose[:]]).transpose()
        self.sigma = np.array(sigma)

        return

    def __str__(self, nested=False):
        tmp = ""
        if not nested:
            tmp += "\nMap feature\n"
        tmp += "\nPose:\n" + str(self.pose)
        tmp += "\nCovariance matrix:\n" + str(self.sigma)
        if not nested:
            tmp += "\n"

        return tmp

    def clone(self):
        return Feature(self.pose.transpose()[0], self.sigma)


# Particle: robot pose and featured-map joint hypothesis
class Particle:
    def __init__(self, pose, weight, features=[]):
        self.pose = np.array([pose[:]]).transpose()
        self.weight = weight
        self.features = features[:]

        return

    def __str__(self):
        tmp = "\nParticle for online SLAM\n"
        tmp += "\nPose:\n" + str(self.pose)
        tmp += "\nWeight: " + str(self.weight)
        tmp += "\nFeatures: "
        for i, f in enumerate(self.features):
            tmp += "\n- " + str(i) + ": " + f.__str__(True)
        tmp += "\n"

        return tmp

    def clone(self):
        return Particle(self.pose.transpose()[0], self.weight, self.features)


# Online SLAM solver based on FastSLAM (particles with robot pose and Kalman filters for each feature)
class OnlineSLAMSolver:
    def __init__(self, particles_num=100, initial_pose=[0.0, 0.0, 0.0], motion_model="velocity", motion_noise=[[0.0, 0.01, 0.0], [0.01, 0.0, 0.0], [0.0, 0.01, 0.0]], observation_model="range_bearing", min_visibility=1.0, max_visibility=5.0, observation_noise=[[0.0, 0.0, 0.5], [0.0, 0.0, 0.05]], correspondence_threshold=10**(-5), delete_threshold=10**(-5)):
        # Initial pose
        self.robot_initial_pose = np.array(initial_pose)

        # Motion
        self.motion_model = motion_model
        # v, omega, yaw for velocity / rot1, trans, rot2 for odometry
        self.motion_noise = np.array(motion_noise)

        # Observation
        self.observation_model = observation_model
        self.min_visibility = min_visibility
        self.max_visibility = max_visibility
        self.observation_noise = np.array(observation_noise)
        self.correspondence_threshold = correspondence_threshold
        self.delete_threshold = delete_threshold

        # Particles
        self.particles_num = particles_num
        self.particles = []

        for i in range(particles_num):
            # Pose
            new_pose = initial_pose[:]

            # Weight
            new_weight = 1.0/particles_num

            new_particle = Particle(new_pose, new_weight)
            self.particles.append(new_particle.clone())

        return

    def __str__(self):
        tmp = "\nOnlineSLAMSolver from FASTSLAM\n"
        tmp += "\nRobot initial pose:\n" + str(self.robot_initial_pose)
        tmp += "\nMotion noise:\n" + str(self.motion_noise)
        tmp += "\nMinimum visibility: " + str(self.min_visibility)
        tmp += "\nMaximum visibility: " + str(self.max_visibility)
        tmp += "\nObservation noise:\n" + str(self.observation_noise)
        tmp += "\nNumber of particles: " + str(self.particles_num) + "\n"

        return tmp

    def motion_update(self, command, dt=1):
        print("Motion update: " + str(round(command[0], 2)) + ", " + str(
            round(command[1], 2)) + ", " + str(round(command[2], 2)))

        for p in self.particles:
            p.pose = np.array([
                motion(self.motion_model, p.pose.transpose()[0], command, self.motion_noise, dt)[:]]).transpose()

        return

    def observation_update_particle_weight(self, observation, j):
        # print("\nParticle " + str(j+1))

        # Correction & mapping
        p = self.particles[j]

        features_correspondences = len(observation)*[None]
        features_preferences = []
        features_likelihoods = []
        global_visible_features = []
        handled_observations = []
        unhandled_visible_features = []

        if len(p.features) > 0:
            # print("\nFeatures:")
            # print([i for i in range(len(p.features))])

            correspondences_orders = len(observation)*[0]

            for i in range(len(observation)):
                # print("\nObservation " + str(i+1))

                visible, global_visible_features, visible_features, invisible_features, corresponding_features, corresponding_likelihoods = correspondence(
                    [[feature.pose, feature.sigma] for feature in p.features], p.pose, observation[i].transpose()[0], self.min_visibility, self.max_visibility, self.observation_noise, self.correspondence_threshold, self.delete_threshold)

                handled_observations.append(
                    visible and invisible_features == [])

                if handled_observations[-1]:
                    features_preferences.append(corresponding_features)
                    features_likelihoods.append(corresponding_likelihoods)

                    # print("Corresponding features:")
                    # print(corresponding_features)
                    # print([format(l, '.1E')
                    #    for l in corresponding_likelihoods])

                else:
                    features_correspondences.pop(-1)
                    for f in visible_features:
                        if f not in unhandled_visible_features:
                            unhandled_visible_features.append(f)

            tmp = observation[:]
            observation = []
            for i, k in enumerate(handled_observations):
                if k:
                    observation.append(tmp[i])

            for i in range(len(features_preferences)):
                if features_preferences[i] == []:
                    features_correspondences[i] = None
                else:
                    features_correspondences[i] = features_preferences[i][0]

            conflicts = dict()
            for i, c in enumerate(features_correspondences):
                if c != None and features_correspondences.count(c) > 1:
                    if c not in conflicts:
                        conflicts[c] = [i]
                    else:
                        conflicts[c].append(i)

            while conflicts != {}:
                for c in conflicts:
                    likelihoods = [
                        [features_likelihoods[e][correspondences_orders[e]], e] for e in conflicts[c]]
                    tmp = list(np.sort(likelihoods))
                    tmp.reverse()
                    right_feature = tmp[0][1]

                    for i, e in enumerate(conflicts[c]):
                        if e != right_feature:
                            new_order = correspondences_orders[e] + 1
                            if new_order < len(features_preferences[e]):
                                features_correspondences[e] = features_preferences[e][new_order]
                                correspondences_orders[e] += 1
                            else:
                                features_correspondences[e] = None

                conflicts = dict()
                for i, c in enumerate(features_correspondences):
                    if c != None and features_correspondences.count(c) > 1:
                        if c not in conflicts:
                            conflicts[c] = [i]
                        else:
                            conflicts[c].append(i)

        for i in range(len(observation)):
            new_z = np.array(observation[i][:])
            new_feature_index = len(p.features)

            # Previously unseen feature
            if features_correspondences[i] == None:
                # print("New feature: " + str(new_feature_index))
                new_feature_index += 1

                # if len(p.features) < 10:
                mu = h_inverse(p.pose, new_z)
                H1 = H(p.pose, mu)
                H1_inverse = np.linalg.inv(H1)
                Q1 = Q(new_z)
                sigma = (H1_inverse.dot(Q1)).dot(H1_inverse.transpose())
                p.features.append(Feature(mu.transpose()[0], sigma))

                d = sqrt((p.pose.transpose()[0][0]-mu.transpose()[0][0])
                         ** 2 + (p.pose.transpose()[0][1]-mu.transpose()[0][1])**2)

                assert d < self.max_visibility

            # Already seen feature
            # TODO: Bug
            else:
                feature = p.features[features_correspondences[i]]
                mu, sigma = feature.pose[:], feature.sigma[:]
                f = mu.transpose()[0]
                predicted_z = h(p.pose, f)
                H1 = H(p.pose, mu)
                Q1 = (H1.dot(sigma)).dot(
                    H1.transpose()) + Q(new_z)
                Q1_inverse = np.linalg.inv(Q1)
                K = (sigma.dot(H1.transpose())).dot(Q1_inverse)
                new_mu = mu + K.dot(new_z - predicted_z)
                new_sigma = (np.identity(2) - K.dot(H1)).dot(sigma)
                p.features[features_correspondences[i]] = Feature(
                    new_mu.transpose()[0], new_sigma)

                if abs(new_z[1] - predicted_z[1]) > abs(new_z[1] - predicted_z[1] + 2*pi):
                    predicted_z -= 2*pi
                elif abs(new_z[1] - predicted_z[1]) > abs(new_z[1] - predicted_z[1] - 2*pi):
                    predicted_z[1] += 2*pi

                eigenvalues = np.linalg.eigvals(Q1)
                d_interval, phi_interval = eigenvalues[0] / \
                    10, eigenvalues[1]/10
                weight_update = mvnun(np.array([new_z[0]-d_interval/2.0, new_z[1]-phi_interval/2.0]), np.array(
                    [new_z[0]+d_interval/2.0, new_z[1]+phi_interval/2.0]), np.array([predicted_z[0], predicted_z[1]]), np.array(Q1))[0]

                assert weight_update <= 1, "Probability greater than 1 !"

                d = sqrt((p.pose.transpose()[0][0]-new_mu.transpose()[0][0])**2 + (
                    p.pose.transpose()[0][1]-new_mu.transpose()[0][1])**2)

                # if d >= self.max_visibility:
                # print("\nfeature:")
                # print(features_correspondences[i])
                # print("visibility:")
                # print(self.max_visibility)
                # print("d:")
                # print(d)
                # print("new_z:")
                # print(new_z)
                # print("predicted_z:")
                # print(predicted_z)

                # print("\n")
                # assert False

                # Bias to ensure non-zero weight
                p.weight = max(p.weight*weight_update, 10**(-323))

        old_features = p.features[:]
        features_to_delete = []
        for i in range(len(old_features)):
            if i in global_visible_features and i not in features_correspondences and i not in unhandled_visible_features:
                features_to_delete.append(i)

        # print("\nGlobal visible features: " + str(global_visible_features))
        # print("Correspondences: " + str(features_correspondences))
        # print("Unhandled visible features: " + str(unhandled_visible_features))
        # print("Features to delete:" + str(features_to_delete))
        # print("")

        # Delete unused features
        p.features = []
        for i in range(len(old_features)):
            if i not in features_to_delete:
                p.features.append(old_features[i].clone())

        return

    def observation_update(self, observation):
        print("Observation update: " + str(len(observation)))

        print("- Weights update")

        for i in range(self.particles_num):
            self.observation_update_particle_weight(observation, i)

        print("- Normalization")

        # Normalization
        weights_sum = 0
        for p in self.particles:
            weights_sum += p.weight
        assert weights_sum > 0, "/!\ Sum of weights equals 0 !"
        for p in self.particles:
            p.weight /= weights_sum

        return

    def resampling(self):
        print("- Resampling")

        # Resampling criteria (effective particles number)
        effective_particles_num = 0
        for p in self.particles:
            effective_particles_num += p.weight**2
        effective_particles_num = 1/effective_particles_num

        # Resampling
        indexes = []
        if effective_particles_num < self.particles_num/2:
            # if False:
            print("  Yes")
            cumulated_weights = [0]
            for p in self.particles:
                cumulated_weights.append(cumulated_weights[-1] + p.weight)

            old_particles = self.particles[:]
            self.particles = []
            resampling_sigma = [[0.01, 0.0, 0.0],
                                [0.0, 0.01, 0.0],
                                [0.0, 0.0, 0.001]]

            for i in range(self.particles_num):
                r = random()
                for j in range(len(cumulated_weights)-1):
                    if r >= cumulated_weights[j] and r < cumulated_weights[j+1]:
                        new_pose = multivariate_normal.rvs(
                            old_particles[j].pose.transpose()[0], resampling_sigma)[:]
                        new_weight = 1.0/self.particles_num
                        new_features = old_particles[j].features[:]
                        new_particle = Particle(
                            new_pose, new_weight, new_features)
                        self.particles.append(new_particle.clone())
                        indexes.append(j)

                        break

            return True, indexes

        else:
            print("  No")

            return False, indexes


# TESTS

if __name__ == '__main__':
    F = Feature([78.9, -14.26], [[0.52, 0.0], [0.0, 0.00187]])
    print(F)

    P = Particle(np.array([2.4, -5.6, 0.1254]), 0.0015)
    print(P)

    S = OnlineSLAMSolver()
    print(S)
