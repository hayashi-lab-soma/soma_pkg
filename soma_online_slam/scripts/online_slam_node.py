#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
from geometry_msgs.msg import PoseArray, Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion
from math import sqrt, atan2, pi, cos, sin
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from onlineSLAMSolver import OnlineSLAMSolver


# Online SLAM ROS node for Gazebo simulation
class OnlineSLAMNode:
    def __init__(self, particles_num=50, motion_model="odometry", motion_noise=[[0.01, 0.1, 0.0], [0.1, 0.01, 0.0], [0.01, 0.1, 0.0]], observation_model="range_bearing", min_visibility=1.0, max_visibility=10.0, observation_noise=[[0.0, 0.0, 0.1], [0.0, 0.0, 0.1]], correspondence_threshold=10**(-5), delete_threshold=10**(-5)):
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

        else:
            assert False, "Invalid motion model: " + motion_model

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

    def real_update(self, data):
        # FOREST WITH UNEVEN GROUND
        # soma_index = 11
        # trees_start_index = 1
        # trees_stop_index = 11

        # FLAT FOREST
        soma_index = 18
        trees_start_index = 1
        trees_stop_index = 17

        self.real_x_data = data.pose[soma_index].position.x
        self.real_y_data = data.pose[soma_index].position.y
        self.real_theta_data = euler_from_quaternion([
            data.pose[soma_index].orientation.x, data.pose[soma_index].orientation.y, data.pose[soma_index].orientation.z, data.pose[soma_index].orientation.w])[2]

        if self.first:
            for p in self.solver.particles:
                p.pose[0][0] = self.real_x_data
                p.pose[1][0] = self.real_y_data
                p.pose[2][0] = self.real_theta_data

                self.trees_x_data = [
                    data.pose[i].position.x for i in range(trees_start_index, trees_stop_index)]
                self.trees_y_data = [
                    data.pose[i].position.y for i in range(trees_start_index, trees_stop_index)]

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
        if self.solver.motion_model == "velocity":
            stop = rospy.get_time()

        if self.observation_mutex or self.motion_mutex:
            print("Already running...\n")
            return

        self.observation_mutex = True
        print("SOLVER LOOP")

        observation = []
        for pose in data.poses:
            # Polar coordinates
            d = sqrt(pose.position.x**2 + pose.position.y**2)
            phi = atan2(pose.position.y, pose.position.x)

            if d > self.solver.min_visibility and d < self.solver.max_visibility:
                observation.append(np.array([[d], [phi]]))

        if self.solver.motion_model == "velocity":
            self.solver.motion_update(self.command, stop-self.command_start)

        elif self.solver.motion_model == "odometry":
            self.solver.motion_update(self.command)

            self.old_odom = self.new_odom[:]

        self.solver.observation_update(observation)

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

    rospy.Subscriber('/tree_pose_array', PoseArray,
                     callback=node.observation_update, queue_size=1)

    ani = FuncAnimation(node.fig, node.update_plot, init_func=node.plot_init)
    plt.show(block=True)

    rospy.spin()
