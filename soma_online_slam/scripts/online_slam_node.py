#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float64MultiArray
from math import sqrt, atan2
import matplotlib.pyplot as plt


def observation_update(data):
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
    # l = len(data.poses)
    # for i in range(l):
    #     plt.plot(data.poses[i].position.x,
    #              data.poses[i].position.y, "bo", markersize=10)
    # plt.show()


if __name__ == '__main__':
    rospy.init_node('online_slam', anonymous=True)
    rospy.Subscriber('/tree_pose_array', PoseArray,
                     callback=observation_update)
    observation_pub = rospy.Publisher(
        '/observation', Float64MultiArray, queue_size=1)

    rospy.spin()
