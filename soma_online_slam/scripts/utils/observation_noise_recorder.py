#!/usr/bin/env python
import rospy
import os
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseArray
from tf.transformations import euler_from_quaternion
from math import atan2, pi, sqrt, cos, sin
import message_filters


DIR = os.path.dirname(__file__)
OBSERVATION_FILE_NAME = DIR + '/../../data/observation.txt'


class ObservationNoiseRecorder:
    def __init__(self):
        self.real_pose = [0, 0, 0]
        self.real_trees = []

        self.first = True

        return

    def callback(self, real, lidar):
        soma_index = 17
        trees_start_index = 1
        trees_stop_index = 17

        real_x = real.pose[soma_index].position.x
        real_y = real.pose[soma_index].position.y
        real_theta = euler_from_quaternion([
            real.pose[soma_index].orientation.x, real.pose[soma_index].orientation.y, real.pose[soma_index].orientation.z, real.pose[soma_index].orientation.w])[2]
        self.real_pose = [real_x, real_y, real_theta]

        if self.first:
            self.real_trees = [[real.pose[i].position.x, real.pose[i].position.y]
                               for i in range(trees_stop_index - trees_start_index)]

            self.first = False

        for pose in lidar.poses:
            if pose.position.x != 0 and pose.position.y != 0:
                d = sqrt(pose.position.x**2 + pose.position.y**2)
                phi = atan2(pose.position.y, pose.position.x)
                x = d*cos(phi+self.real_pose[2]) + self.real_pose[0]
                y = d*sin(phi+self.real_pose[2]) + self.real_pose[1]
                nearest_tree = 0
                min_error = 100000
                for i, tree in enumerate(self.real_trees):
                    error = (x-tree[0])**2 + (y-tree[1])**2
                    if error < min_error:
                        nearest_tree = i
                        min_error = error
                real_d = sqrt((self.real_trees[nearest_tree][0]-self.real_pose[0])**2 + (
                    self.real_trees[nearest_tree][1]-self.real_pose[1])**2)
                real_phi = atan2(self.real_trees[nearest_tree][1]-self.real_pose[1],
                                 self.real_trees[nearest_tree][0]-self.real_pose[0]) - self.real_pose[2]
                d_error = d-real_d
                phi_error = phi-real_phi
                if phi_error > pi:
                    phi_error -= 2*pi
                elif phi_error <= -pi:
                    phi_error += 2*pi
                x_error = x-self.real_trees[nearest_tree][0]
                y_error = y-self.real_trees[nearest_tree][1]

            tmp = str(real_d) + " " + str(real_phi) + " "
            tmp += str(d_error) + " " + \
                str(phi_error) + " "
            tmp += str(x_error) + " " + \
                str(y_error) + "\n"

            fo = open(OBSERVATION_FILE_NAME, "a")
            fo.write(tmp)
            fo.close()

        return


if __name__ == '__main__':
    rospy.init_node('motion_noise_estimation', anonymous=True)

    node = ObservationNoiseRecorder()

    real_sub = message_filters.Subscriber(
        '/gazebo/model_states', ModelStates)
    lidar_sub = message_filters.Subscriber(
        '/trees_centers', PoseArray)

    sync = message_filters.ApproximateTimeSynchronizer(
        [real_sub, lidar_sub], 10, 0.1, allow_headerless=True)
    sync.registerCallback(node.callback)

    rospy.spin()
