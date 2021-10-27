#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
from geometry_msgs.msg import PoseArray
from math import sqrt, atan2
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from onlineSLAMSolver import OnlineSLAMSolver


class OnlineSLAMNode:
    def __init__(self):
        self.solver = OnlineSLAMSolver(particles_num=10)

        self.fig, self.ax = plt.subplots()
        self.particles_ln, = plt.plot([], [], 'ro', markersize=5)
        self.features_ln, = plt.plot([], [], 'bo', markersize=10)
        self.particles_x_data, self.particles_y_data = [], []
        self.features_x_data, self.features_y_data = [], []

        self.mutex = False

        return

    def plot_init(self):
        self.ax.set_xlim(-20, 20)
        self.ax.set_ylim(-20, 20)

        return self.particles_ln, self.features_ln

    def update_plot(self, frame):
        if self.mutex:
            return

        print("DISPLAY\n")

        self.particles_ln.set_data(
            self.particles_x_data, self.particles_y_data)
        self.features_ln.set_data(self.features_x_data, self.features_y_data)

        return self.particles_ln, self.features_ln

    def observation_update(self, data):
        if self.mutex:
            print("Already running...\n")
            return

        self.mutex = True
        print("SOLVER LOOP")

        observation = []
        for pose in data.poses:
            # Polar coordinates
            d = sqrt(pose.position.x**2 + pose.position.y**2)
            phi = atan2(pose.position.y, pose.position.x)

            observation.append(np.array([[d], [phi]]))

        self.solver.motion_update([0.1, 0.001])
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

        self.mutex = False

        print("END\n")

        return


if __name__ == '__main__':
    rospy.init_node('online_slam', anonymous=True)

    node = OnlineSLAMNode()

    rospy.Subscriber('/tree_pose_array', PoseArray,
                     callback=node.observation_update, queue_size=1)

    ani = FuncAnimation(node.fig, node.update_plot, init_func=node.plot_init)
    plt.show(block=True)

    rospy.spin()
