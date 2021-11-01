#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
from math import pi
from geometry_msgs.msg import PoseArray, Twist
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion
from math import sqrt, atan2
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from onlineSLAMSolver import OnlineSLAMSolver


class OnlineSLAMNode:
    def __init__(self):
        self.solver = OnlineSLAMSolver(particles_num=10)
        self.first = True

        self.command = [0, 0]
        self.command_start = rospy.get_time()
        self.observation = False

        self.fig, self.ax = plt.subplots()
        self.real_ln, = plt.plot([], [], 'go', markersize=5)
        self.particles_ln, = plt.plot([], [], 'ro', markersize=5)
        self.features_ln, = plt.plot([], [], 'bo', markersize=10)
        self.real_x_data, self.real_y_data, self.real_theta_data = 0, 0, 0
        self.particles_x_data, self.particles_y_data = [], []
        self.features_x_data, self.features_y_data = [], []

        self.motion_mutex = False
        self.observation_mutex = False

        return

    def plot_init(self):
        self.ax.set_xlim(-20, 20)
        self.ax.set_ylim(-20, 20)

        return self.real_ln, self.particles_ln, self.features_ln

    def update_plot(self, frame):
        if self.motion_mutex or self.observation_mutex:
            return

        print("DISPLAY\n")

        self.features_ln.set_data(self.features_x_data, self.features_y_data)
        self.particles_ln.set_data(
            self.particles_x_data, self.particles_y_data)
        self.real_ln.set_data(self.real_x_data, self.real_y_data)

        return self.real_ln, self.particles_ln, self.features_ln

    def real_update(self, data):
        self.real_x_data = data.pose[1].position.x
        self.real_y_data = data.pose[1].position.y
        self.real_theta_data = euler_from_quaternion([
            data.pose[1].orientation.x, data.pose[1].orientation.y, data.pose[1].orientation.z, data.pose[1].orientation.w])[2]

        return

    def motion_update(self, data):
        stop = rospy.get_time()

        new_command = [data.linear.x, data.angular.z]

        if self.command != new_command:
            self.motion_mutex = True
            print("MOTION UPDATE\n")

            if not self.observation:
                self.solver.motion_update(
                    self.command, stop-self.command_start)

            self.command = [data.linear.x, data.angular.z]
            self.command_start = stop
            self.observation = False

            self.motion_mutex = False

        return

    def observation_update(self, data):
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

            observation.append(np.array([[d], [phi]]))

        if self.first:
            for p in self.solver.particles:
                p.pose[0][0] = self.real_x_data
                p.pose[1][0] = self.real_y_data
                p.pose[2][0] = self.real_theta_data

            self.first = False

        self.solver.motion_update(self.command, stop-self.command_start)
        self.solver.observation_update(observation)

        self.particles_x_data, self.particles_y_data = [], []
        for p in self.solver.particles:
            self.particles_x_data.append(p.pose[0][0])
            self.particles_y_data.append(p.pose[1][0])

        highest_weight = 0
        for p in self.solver.particles:
            if p.weight > highest_weight:
                highest_weight = p.weight
        most_probable_particle = p.clone()

        self.features_x_data, self.features_y_data = [], []
        if highest_weight > 0:
            for feature in most_probable_particle.features:
                self.features_x_data.append(feature.pose[0][0])
                self.features_y_data.append(feature.pose[1][0])

        self.command_start = stop
        self.observation = False

        self.observation_mutex = False

        print("END\n")

        self.update_plot(None)

        return


if __name__ == '__main__':
    rospy.init_node('online_slam', anonymous=True)

    node = OnlineSLAMNode()

    rospy.Subscriber('/tree_pose_array', PoseArray,
                     callback=node.observation_update, queue_size=1)
    rospy.Subscriber('/ackermann_steering_controller/cmd_vel',
                     Twist, callback=node.motion_update, queue_size=1)
    rospy.Subscriber('/gazebo/model_states', ModelStates,
                     callback=node.real_update, queue_size=1)

    ani = FuncAnimation(node.fig, node.update_plot, init_func=node.plot_init)
    plt.show(block=True)

    rospy.spin()
