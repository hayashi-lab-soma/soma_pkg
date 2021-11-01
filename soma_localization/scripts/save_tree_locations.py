#!/usr/bin/python2

import rospy
import numpy as np
import os
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from math import sqrt, atan2

DIR = os.path.dirname(__file__)
SAVE_FILE_NAME = DIR+'/../data/Mirais-tree_locations.txt'


def cb(data):
    stamp = data.header.stamp
    print('Time:', stamp)

    trees = []
    for i, pose in enumerate(data.poses):
        # print(pose.position.x, pose.position.y, pose.position.z)
        tmp = [
            int(i),
            round(pose.position.x, 3),
            round(pose.position.y, 3),
            round(pose.position.z, 3)
        ]
        # Polar coordinates
        r = round(sqrt(tmp[1]**2 + tmp[2]**2), 3)
        theta = round(atan2(tmp[2], tmp[1]), 3)
        tmp += [r, theta]
        trees.append(tmp)

    trees = np.array(trees)
    print(trees)
    print('tree num:', len(trees))

    np.savetxt(SAVE_FILE_NAME, trees,
               fmt=['%d',
                    '%.3f', '%.3f', '%.3f',
                    '%.3f', '%.3f'])
    print('save tree locations')


if __name__ == '__main__':
    rospy.init_node('TreeLocationServerError', anonymous=True)
    rospy.Subscriber('/tree_pose_array', PoseArray, callback=cb)
    print(DIR)
    rospy.spin()
