#!/usr/bin/python2

import rospy
import numpy as np
import os
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray

DIR = os.path.dirname(__file__)
SAVE_FILE_NAME = DIR+'/../data/Mirais-tree_locations.txt'


def cb(data):
  stamp = data.header.stamp
  print('Time:', stamp)

  trees = []
  for pose in data.poses:
    # print(pose.position.x, pose.position.y, pose.position.z)
    tmp = [
        round(pose.position.x, 3),
        round(pose.position.y, 3),
        round(pose.position.z, 3)
    ]
    trees.append(tmp)

  trees = np.array(trees)
  print(trees)
  print('tree num:', len(trees))

  np.savetxt(SAVE_FILE_NAME, trees, fmt="%.3f")
  print('save tree locations')


if __name__ == '__main__':
  rospy.init_node('TreeLocationServerError', anonymous=True)
  rospy.Subscriber('/tree_pose_array', PoseArray, callback=cb)
  print(DIR)
  rospy.spin()
