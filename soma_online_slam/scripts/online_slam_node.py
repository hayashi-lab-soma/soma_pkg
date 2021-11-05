#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
from geometry_msgs.msg import PoseArray, Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion
from math import sqrt, atan2, pi
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from onlineSLAMSolver import OnlineSLAMSolver


# Online SLAM ROS node for Gazebo simulation
class OnlineSLAMNode:
    def __init__(self, motion_model="odometry", motion_noise=[[0.007, 0.0, 0.0], [0.0, 0.01, 0.0], [0.0, 0.0, 0.007]], observation_model="range_bearing"):
        self.solver = OnlineSLAMSolver(
            particles_num=10, motion_model=motion_model, motion_noise=motion_noise, observation_model=observation_model)

        self.first = True

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
        self.trees_ln, = plt.plot([], [], 'bx', markersize=10)
        self.trees_x_data, self.trees_y_data = [], []
        self.particles_ln, = plt.plot([], [], 'go', markersize=5)
        self.particles_x_data, self.particles_y_data = [], []
        self.features_ln, = plt.plot([], [], 'gx', markersize=10)
        self.features_x_data, self.features_y_data = [], []

        self.motion_mutex = False
        self.observation_mutex = False

        return

    def plot_init(self):
        self.ax.set_xlim(-15, 15)
        self.ax.set_ylim(-15, 15)

        return self.real_ln, self.trees_ln, self.particles_ln, self.features_ln

    def update_plot(self, frame):
        if self.motion_mutex or self.observation_mutex:
            return

        print("DISPLAY\n")

        self.real_ln.set_data(self.real_x_data, self.real_y_data)
        self.trees_ln.set_data(self.trees_x_data, self.trees_y_data)
        self.features_ln.set_data(self.features_x_data, self.features_y_data)
        self.particles_ln.set_data(
            self.particles_x_data, self.particles_y_data)

        return self.real_ln, self.trees_ln, self.particles_ln, self.features_ln

    def real_update(self, data):
        self.real_x_data = data.pose[1].position.x
        self.real_y_data = data.pose[1].position.y
        self.real_theta_data = euler_from_quaternion([
            data.pose[1].orientation.x, data.pose[1].orientation.y, data.pose[1].orientation.z, data.pose[1].orientation.w])[2]

        if self.first:
            for p in self.solver.particles:
                p.pose[0][0] = self.real_x_data
                p.pose[1][0] = self.real_y_data
                p.pose[2][0] = self.real_theta_data

                self.trees_x_data = [
                    data.pose[i].position.x for i in range(2, 19)]
                self.trees_y_data = [
                    data.pose[i].position.y for i in range(2, 19)]

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
        for p in self.solver.particles:
            if p.weight > highest_weight:
                highest_weight = p.weight
        most_probable_particle = p.clone()

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
