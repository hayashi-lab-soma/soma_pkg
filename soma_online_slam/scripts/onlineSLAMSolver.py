import numpy as np
from math import sqrt, pi, exp
from random import random
from scipy.stats import multivariate_normal
from utils.fastslam import delete_features, motion, correspondence, h_inverse, h, H, Q, delete_features


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


class OnlineSLAMSolver:
    def __init__(self, particles_num=100, initial_x=0.0, initial_y=0.0, initial_theta=0.0, v_noise=[0.01, 0.0], omega_noise=[0.0, 0.01], yaw_noise=[0.0, 0.01], visibility=30, d_noise=[0.05, 0.0], phi_noise=[0.0005, 0.0]):
        # Initial pose
        self.robot_initial_pose = np.array(
            [[initial_x], [initial_y], [initial_theta]])

        # Motion
        self.motion_noise = np.array([v_noise, omega_noise, yaw_noise])

        # Observation
        self.visibility = visibility
        self.observation_noise = np.array([d_noise, phi_noise])

        # Particles
        self.particles_num = particles_num
        self.particles = []

        for i in range(particles_num):
            # Pose
            new_pose = [initial_x, initial_y, initial_theta]

            # Weight
            new_weight = 1.0/particles_num

            new_particle = Particle(new_pose, new_weight)
            self.particles.append(new_particle.clone())

        return

    def __str__(self):
        tmp = "\nOnlineSLAMSolver from FASTSLAM\n"
        tmp += "\nRobot initial pose:\n" + str(self.robot_initial_pose)
        tmp += "\nMotion noise:\n" + str(self.motion_noise)
        tmp += "\nVisibility: " + str(self.visibility)
        tmp += "\nObservation noise:\n" + str(self.observation_noise)
        tmp += "\nNumber of particles: " + str(self.particles_num) + "\n"

        return tmp

    def motion_update(self, command, dt):
        print("Motion update")

        for p in self.particles:
            p.pose = np.array([
                motion(p.pose.transpose()[0], command, self.motion_noise, dt)[:]]).transpose()

        return

    def observation_update(self, observation):
        print("Observation update")

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

        # Find most probable pose and features
        # max_weight = 0
        # for i, p in enumerate(self.particles):
            # if p.weight > max_weight:
            # max_weight = p.weight

        print("- Resampling")

        # Resampling criteria (effective particles number)
        effective_particles_num = 0
        for p in self.particles:
            effective_particles_num += p.weight**2
        effective_particles_num = 1/effective_particles_num

        # Resampling
        if effective_particles_num < self.particles_num/2:
            # if False:
            cumulated_weights = [0]
            for p in self.particles:
                cumulated_weights.append(cumulated_weights[-1] + p.weight)

            new_particles = []
            resampling_sigma = [[0.01, 0, 0],
                                [0, 0.01, 0],
                                [0, 0, 0.0001]]
            for i in range(self.particles_num):
                r = random()
                for j in range(len(cumulated_weights)-1):
                    if r >= cumulated_weights[j] and r < cumulated_weights[j+1]:
                        new_pose = multivariate_normal.rvs(
                            self.particles[j].pose.transpose()[0], resampling_sigma)[:]
                        new_weight = 1.0/self.particles_num
                        new_features = self.particles[j].features[:]
                        new_particle = Particle(
                            new_pose, new_weight, new_features)
                        new_particles.append(new_particle.clone())

        return

    def observation_update_particle_weight(self, observation, j):
        # Correction & mapping
        p = self.particles[j]

        features_correspondences = len(observation)*[None]

        if len(p.features) > 0:
            correspondences_orders = len(observation)*[0]
            features_preferences = []
            features_likelihoods = []

            for i in range(len(observation)):

                corresponding_features, corresponding_likelihoods = correspondence(
                    [[feature.pose, feature.sigma] for feature in p.features], p.pose, observation[i].transpose()[0], self.visibility, self.observation_noise, 10**(-5))

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

        for i in range(len(observation)):
            new_z = np.array(observation[i][:])

            # Previously unseen feature
            if features_correspondences[i] == None:
                # if len(p.features) < 10:
                mu = h_inverse(p.pose, new_z)
                H1 = H(p.pose, mu)
                H1_inverse = np.linalg.inv(H1)
                Q1 = Q(new_z)
                sigma = (H1_inverse.dot(Q1)).dot(H1_inverse.transpose())
                p.features.append(Feature(mu.transpose()[0], sigma))

            # Already seen feature
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
                tmp = (
                    (new_z - predicted_z).transpose().dot(Q1_inverse)).dot(new_z - predicted_z)[0][0]
                tmp = max(tmp, -709)
                weight_update = 1 / \
                    sqrt(2*pi*abs(np.linalg.det(Q1))) * exp(-1/2 * tmp)

                # Bias to ensure non-zero weight
                p.weight = max(p.weight*weight_update, 10**(-323))

        new_features = delete_features(
            [[feature.pose, feature.sigma] for feature in p.features], p.pose, observation, self.observation_noise, self.visibility, 10**(-5))[:]
        p.features = []
        for feature in new_features:
            p.features.append(Feature(feature[0].transpose()[0], feature[1]))

        return


# Tests

if __name__ == '__main__':
    F = Feature([78.9, -14.26], [[0.52, 0.0], [0.0, 0.00187]])
    print(F)

    P = Particle(np.array([2.4, -5.6, 0.1254]), 0.0015)
    print(P)

    S = OnlineSLAMSolver()
    print(S)
