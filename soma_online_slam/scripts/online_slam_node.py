#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float64MultiArray
from math import sqrt, atan2
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class Visualiser:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.ln0, = plt.plot([0], [0], 'ro', markersize=20)
        self.ln1, = plt.plot([], [], 'bo', markersize=10)
        self.x_data, self.y_data = [], []

    def plot_init(self):
        self.ax.set_xlim(-20, 20)
        self.ax.set_ylim(-20, 20)
        return self.ln1

    def observation_update(self, data):
        observation = []
        for pose in data.poses:
            # Polar coordinates
            r = sqrt(pose.position.x**2 + pose.position.y**2)
            theta = atan2(pose.position.y, pose.position.x)

            observation += [r, theta]

        # Publish observation
        # o = Float64MultiArray()
        # o.data = observation
        # observation_pub.publish(o)

        # Display
        l = len(data.poses)
        x, y = [], []
        for i in range(l):
            x.append(data.poses[i].position.x)
            y.append(data.poses[i].position.y)

        self.x_data = x[:]
        self.y_data = y[:]

    def update_plot(self, frame):
        self.ln1.set_data(self.x_data, self.y_data)
        return self.ln1


if __name__ == '__main__':
    rospy.init_node('online_slam', anonymous=True)

    vis = Visualiser()

    rospy.Subscriber('/tree_pose_array', PoseArray,
                     callback=vis.observation_update)
    observation_pub = rospy.Publisher(
        '/observation', Float64MultiArray, queue_size=1)

    ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init)
    plt.show(block=True)

    rospy.spin()
