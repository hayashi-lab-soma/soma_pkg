#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import os
from datetime import datetime
from geometry_msgs.msg import PoseArray, Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion
from math import sqrt, atan2, pi, cos, sin
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from utils.fastslam import position_error, map_error
from onlineSLAMSolver import OnlineSLAMSolver


DIR = os.path.dirname(__file__)
RESULTS_FILE_NAME = DIR + '/../data/results.txt'
RAW_HISTORY_FILE_NAME = DIR + '/../data/raw_history.txt'


# Online SLAM ROS node for Gazebo simulation
class OnlineSLAMNode:
    def __init__(self, particles_num=10, motion_model="odometry", motion_noise=[[0.01, 0.1, 0.0], [0.1, 0.01, 0.0], [0.01, 0.1, 0.0]], observation_model="range_bearing", min_visibility=1.0, max_visibility=8.0, observation_noise=[[0.0, 0.0, 0.5], [0.0, 0.0, 0.01]], correspondence_threshold=10**(-10), delete_threshold=10**(-10)):
        self.real = []
        self.features = []

        self.solver = OnlineSLAMSolver(
            particles_num=particles_num, motion_model=motion_model, motion_noise=motion_noise, observation_model=observation_model, min_visibility=min_visibility, max_visibility=max_visibility, observation_noise=observation_noise, correspondence_threshold=correspondence_threshold, delete_threshold=delete_threshold)

        self.first = True

        self.most_probable_index = 0

        if motion_model == "velocity":
            self.command = [0.0, 0.0]
            self.command_start = rospy.get_time()

        elif motion_model == "odometry":
            self.command = [0.0, 0.0, 0.0]
            self.old_odom = [0.0, 0.0, 0.0]
            self.new_odom = [0.0, 0.0, 0.0]
            self.last_odom_time = 0

        else:
            assert False, "Invalid motion model: " + motion_model

        self.init_log_results()
        self.init_log_history()

        self.fig, self.ax = plt.subplots()
        self.real_ln, = plt.plot([], [], 'ro', markersize=5)
        self.real_x_data, self.real_y_data, self.real_theta_data = 0, 0, 0
        self.heading_ln, = plt.plot([], [], 'k-', markersize=3)
        self.min_visibility_ln, = plt.plot([], [], 'r--', markersize=1)
        self.max_visibility_ln, = plt.plot([], [], 'r--', markersize=1)
        self.trees_ln, = plt.plot([], [], 'bx', markersize=10)
        self.trees_x_data, self.trees_y_data = [], []
        self.particles_ln, = plt.plot([], [], 'go', markersize=5)
        self.particles_x_data, self.particles_y_data = [], []
        self.most_probable_ln, = plt.plot([], [], 'ko', markersize=5)
        self.features_ln, = plt.plot([], [], 'gx', markersize=10)
        self.features_x_data, self.features_y_data = [], []

        self.motion_mutex = False
        self.observation_mutex = False

        return

    # Logs

    def init_log_results(self):
        fo = open(RESULTS_FILE_NAME, "w")

        fo.write("Date & time: " + str(datetime.now()))

        fo.write("\n\n\nPARAMETERS")

        fo.write("\n\nparticles_num: " + str(self.solver.particles_num))
        fo.write("\n\nmotion_model: " + str(self.solver.motion_model))
        fo.write("\nmotion_noise:\n" + str(self.solver.motion_noise))
        fo.write("\n\nobservation_model: " +
                 str(self.solver.observation_model))
        fo.write("\nmin_visibility: " + str(self.solver.min_visibility))
        fo.write("\nmax_visibility: " + str(self.solver.max_visibility))
        fo.write("\nobservation_noise:\n" + str(self.solver.observation_noise))
        fo.write("\n\ncorrespondence_threshold: " +
                 str(self.solver.correspondence_threshold))
        fo.write("\ndelete_threshold: " + str(self.solver.delete_threshold))

        fo.write("\n\n\nRESULTS\n")

        fo.close()

        return

    def log_results(self, most_probable_particle):
        fo = open(RESULTS_FILE_NAME, "r")

        l = ""
        mem = []
        while l != "RESULTS\n":
            l = fo.readline()
            mem.append(l)

        fo.close()

        fo = open(RESULTS_FILE_NAME, "w")

        for l in mem:
            fo.write(l)

        fo.write("\nReal pose: \n" + str([round(self.real_x_data, 3),
                 round(self.real_y_data, 3), round(self.real_theta_data, 3)]))

        fo.write("\n\nReal map:")
        real_map = []
        for i in range(len(self.trees_x_data)):
            real_map.append([round(self.trees_x_data[i], 3),
                            round(self.trees_y_data[i], 3)])
        real_map.sort()
        for i, e in enumerate(real_map):
            fo.write("\n- " + str(i) + ": " + str(e))

        fo.write("\n\nEstimated pose: \n" + str([round(most_probable_particle.pose[0][0], 3), round(
            most_probable_particle.pose[1][0], 3), round(most_probable_particle.pose[2][0], 3)]))

        position_error = sqrt((self.real_x_data - most_probable_particle.pose[0][0])**2 + (
            self.real_y_data - most_probable_particle.pose[1][0])**2)
        fo.write("\nPosition error: " + str(round(position_error, 3)))

        fo.write("\n\nEstimated map:")
        estimated_map = []
        map_error = []
        for feature in most_probable_particle.features:
            estimated_map.append(
                [round(feature.pose[0][0], 3), round(feature.pose[1][0], 3)])
            min_distance = 100
            nearest_tree = 0
            for i, e in enumerate(real_map):
                new_distance = sqrt(
                    (estimated_map[-1][0] - e[0])**2 + (estimated_map[-1][1] - e[1])**2)
                if new_distance < min_distance:
                    nearest_tree = i
                    min_distance = new_distance
            if min_distance > 1:
                nearest_tree = None
            map_error.append([nearest_tree, round(min_distance, 3)])

        alphabet = ["A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L",
                    "M", "N", "O", "P", "Q", "R", "S", "T", "U", "V", "W", "X", "Y", "Z"]
        for i, e in enumerate(estimated_map):
            if i < 26:
                fo.write("\n- " + alphabet[i] + ": " + str(e) + " -> " +
                         str(map_error[i][0]) + ", " + str(map_error[i][1]))

        orphan_features = 0
        redundant_features = 0
        trees_error = len(real_map)*[0]
        for i, e in enumerate(estimated_map):
            tree = map_error[i][0]
            tree_error = map_error[i][1]
            if tree != None:
                if trees_error[tree] > 0:
                    redundant_features += 1
                    trees_error[tree] = min(trees_error[tree], tree_error)
                else:
                    trees_error[tree] = tree_error
            else:
                orphan_features += 1
        detected_features = len(estimated_map) - \
            redundant_features-orphan_features
        global_map_error = round(sum(trees_error)/float(detected_features), 3)

        fo.write("\nDetected features: " + str(detected_features) + "/" + str(len(real_map)
                                                                              ) + " -> " + str(int(float(detected_features)/float(len(real_map))*100)) + "%")
        fo.write("\nGlobal map error: " + str(global_map_error))
        fo.write("\nRedundant features: " + str(redundant_features))
        fo.write("\nOrphan features: " + str(orphan_features))

        fo.close()

        return

    def init_log_history(self):
        fo = open(RAW_HISTORY_FILE_NAME, "w")

        fo.write("Date & time: " + str(datetime.now()))

        fo.write("\n\n\nPARAMETERS")

        fo.write("\n\nparticles_num: " + str(self.solver.particles_num))
        fo.write("\n\nmotion_model: " + str(self.solver.motion_model))
        fo.write("\nmotion_noise:\n" + str(self.solver.motion_noise))
        fo.write("\n\nobservation_model: " +
                 str(self.solver.observation_model))
        fo.write("\nmin_visibility: " + str(self.solver.min_visibility))
        fo.write("\nmax_visibility: " + str(self.solver.max_visibility))
        fo.write("\nobservation_noise:\n" + str(self.solver.observation_noise))
        fo.write("\n\ncorrespondence_threshold: " +
                 str(self.solver.correspondence_threshold))
        fo.write("\ndelete_threshold: " + str(self.solver.delete_threshold))

        fo.write("\n\n\nHISTORY")

        fo.close()

        return

    def log_history(self, timestamp, resampling, indexes):
        fo = open(RAW_HISTORY_FILE_NAME, "a")

        fo.write("\n\n" + str(timestamp) + " " + str(resampling))
        for i, p in enumerate(self.solver.particles):
            new = [indexes[i], round(position_error(self.real, [p.pose.transpose()[0][0], p.pose.transpose()[0][1]]), 3),
                   round(map_error(self.features, [None, None, None, None, [[f.pose] for f in p.features]]), 3), round(p.weight, 3)]
            fo.write("\n" + str(new))

        fo.close()

        return

    # Display

    def plot_init(self):
        self.ax.set_xlim(-15, 15)
        self.ax.set_ylim(-15, 15)

        return self.real_ln, self.heading_ln, self.min_visibility_ln, self.max_visibility_ln, self.trees_ln, self.particles_ln, self.most_probable_ln, self.features_ln

    def update_plot(self, frame):
        if self.motion_mutex or self.observation_mutex:
            return

        print("DISPLAY\n")

        self.real_ln.set_data(self.real_x_data, self.real_y_data)
        self.trees_ln.set_data(self.trees_x_data, self.trees_y_data)
        self.features_ln.set_data(self.features_x_data, self.features_y_data)
        self.particles_ln.set_data(
            self.particles_x_data, self.particles_y_data)
        if self.particles_x_data != []:
            self.most_probable_ln.set_data(
                self.particles_x_data[self.most_probable_index], self.particles_y_data[self.most_probable_index])
        self.heading_ln.set_data(np.linspace(self.real_x_data, self.real_x_data + cos(self.real_theta_data), 20),
                                 np.linspace(self.real_y_data, self.real_y_data + sin(self.real_theta_data), 20))
        self.min_visibility_ln.set_data([self.real_x_data + self.solver.min_visibility * cos(theta) for theta in np.linspace(-pi, pi, 100)], [
                                        self.real_y_data + self.solver.min_visibility * sin(theta) for theta in np.linspace(-pi, pi, 100)])
        self.max_visibility_ln.set_data([self.real_x_data + self.solver.max_visibility * cos(theta) for theta in np.linspace(-pi, pi, 100)], [
                                        self.real_y_data + self.solver.max_visibility * sin(theta) for theta in np.linspace(-pi, pi, 100)])

        return self.real_ln, self.heading_ln, self.min_visibility_ln, self.max_visibility_ln, self.trees_ln, self.particles_ln, self.most_probable_ln, self.features_ln

    # Update

    def real_update(self, data):
        # FOREST WITH UNEVEN GROUND
        # soma_index = 11
        # trees_start_index = 1
        # trees_stop_index = 11

        # CYLINDER FOREST WITH UNEVEN GROUND
        # soma_index = 18
        # trees_start_index = 1
        # trees_stop_index = 18

        # FLAT FOREST
        # soma_index = 18
        # trees_start_index = 1
        # trees_stop_index = 17

        # FLAT CYLINDER FOREST
        soma_index = 17
        trees_start_index = 1
        trees_stop_index = 17

        self.real_x_data = data.pose[soma_index].position.x
        self.real_y_data = data.pose[soma_index].position.y
        self.real_theta_data = euler_from_quaternion([
            data.pose[soma_index].orientation.x, data.pose[soma_index].orientation.y, data.pose[soma_index].orientation.z, data.pose[soma_index].orientation.w])[2]
        self.real = [self.real_x_data, self.real_y_data, self.real_theta_data]

        if self.first:
            for p in self.solver.particles:
                p.pose[0][0] = self.real_x_data
                p.pose[1][0] = self.real_y_data
                p.pose[2][0] = self.real_theta_data

            self.trees_x_data = [
                data.pose[i].position.x for i in range(trees_start_index, trees_stop_index)]
            self.trees_y_data = [
                data.pose[i].position.y for i in range(trees_start_index, trees_stop_index)]
            self.features = [[self.trees_x_data[i], self.trees_y_data[i]]
                             for i in range(trees_stop_index - trees_start_index)]

            self.first = False

        return

    def motion_update(self, data):
        if self.solver.motion_model == "velocity":
            stop = rospy.get_time()

            new_command = [data.linear.x, data.angular.z]

            if self.command != new_command:
                self.motion_mutex = True
                print("MOTION UPDATE")

                self.solver.motion_update(
                    self.command, stop-self.command_start)

                self.command = [data.linear.x, data.angular.z]
                self.command_start = stop

                self.motion_mutex = False
                print("END\n")

        elif self.solver.motion_model == "odometry":
            self.last_odom_time = data.header.stamp

            new_x = data.pose.pose.position.x
            new_y = data.pose.pose.position.y
            new_theta = euler_from_quaternion([
                data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])[2]

            self.new_odom = [new_x, new_y, new_theta]

            if self.old_odom[0] != 0.0 and self.old_odom[1] != 0.0 and self.old_odom[2] != 0.0:
                trans = sqrt(
                    (new_x-self.old_odom[0])**2 + (new_y-self.old_odom[1])**2)
                rot1 = atan2(
                    new_y-self.old_odom[1], new_x-self.old_odom[0]) - self.old_odom[2]
                if rot1 <= -pi/2:
                    trans *= -1
                    rot1 += pi
                elif rot1 > pi/2:
                    trans *= -1
                    rot1 -= pi
                rot2 = new_theta - self.old_odom[2] - rot1

                self.command = [rot1, trans, rot2]

            else:
                self.old_odom = self.new_odom[:]

        return

    def observation_update(self, data):
        if self.last_odom_time == 0:
            return

        if self.solver.motion_model == "velocity":
            stop = rospy.get_time()

        if self.observation_mutex or self.motion_mutex:
            print("Already running...\n")
            return

        self.observation_mutex = True
        print("SOLVER LOOP")
        update_time = rospy.get_time()
        print("Time: " + str(update_time) + " s")
        print("Motion delay: " + str(round((update_time - self.last_odom_time.secs -
              self.last_odom_time.nsecs * 10**(-9)) * 10**3, 3)) + " ms")
        print("Observation delay: " + str(round((update_time - data.header.stamp.secs -
              data.header.stamp.nsecs * 10**(-9)) * 10**3, 3)) + " ms")

        observation = []
        for pose in data.poses:
            if pose.position.x != 0 and pose.position.y != 0:

                # Polar coordinates
                d = sqrt(pose.position.x**2 + pose.position.y**2)
                phi = atan2(pose.position.y, pose.position.x)

                # Keep only observations within lidar visibility scope
                if d > self.solver.min_visibility and d < self.solver.max_visibility:
                    observation.append(np.array([[d], [phi]]))

        if self.solver.motion_model == "velocity":
            self.solver.motion_update(self.command, stop-self.command_start)

        elif self.solver.motion_model == "odometry":
            self.solver.motion_update(self.command)

            self.old_odom = self.new_odom[:]
            self.command = [0.0, 0.0, 0.0]

        self.solver.observation_update(observation)
        # Log history (update)
        self.log_history(round(update_time, 3), "UPDATE", [
                         i for i in range(self.solver.particles_num)])
        resampling_time = rospy.get_time()
        resampling, indexes = self.solver.resampling()
        # Log history (resampling)
        if resampling:
            self.log_history(round(resampling_time, 3), "RESAMPLING", indexes)

        self.particles_x_data, self.particles_y_data = [], []
        for p in self.solver.particles:
            self.particles_x_data.append(p.pose[0][0])
            self.particles_y_data.append(p.pose[1][0])

        highest_weight = 0
        for i, p in enumerate(self.solver.particles):
            if p.weight > highest_weight:
                highest_weight = p.weight
                self.most_probable_index = i

        most_probable_particle = self.solver.particles[self.most_probable_index].clone(
        )

        self.features_x_data, self.features_y_data = [], []
        if highest_weight > 0:
            for feature in most_probable_particle.features:
                self.features_x_data.append(feature.pose[0][0])
                self.features_y_data.append(feature.pose[1][0])

        if self.solver.motion_model == "velocity":
            self.command_start = stop

        self.observation_mutex = False
        print("END\n")

        self.update_plot(None)

        # Log results
        self.log_results(most_probable_particle)

        return


if __name__ == '__main__':
    rospy.init_node('online_slam', anonymous=True)

    motion_model = "odometry"
    node = OnlineSLAMNode(motion_model=motion_model)

    rospy.Subscriber('/gazebo/model_states', ModelStates,
                     callback=node.real_update, queue_size=1)

    if motion_model == "velocity":
        rospy.Subscriber('/ackermann_steering_controller/cmd_vel',
                         Twist, callback=node.motion_update, queue_size=1)
    elif motion_model == "odometry":
        rospy.Subscriber('/ackermann_steering_controller/odom',
                         Odometry, callback=node.motion_update, queue_size=1)
    else:
        assert False, "Invalid motion model: " + motion_model

    # With centers of clusters
    # rospy.Subscriber('/tree_pose_array', PoseArray,
        #  callback=node.observation_update, queue_size=1)
    # With circle recognition
    rospy.Subscriber('/trees_centers', PoseArray,
                     callback=node.observation_update, queue_size=1)

    ani = FuncAnimation(node.fig, node.update_plot, init_func=node.plot_init)
    plt.show(block=True)

    rospy.spin()
