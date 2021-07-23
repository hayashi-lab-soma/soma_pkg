#!/usr/bin/env python

import math
import numpy as np
import os
import time
#
import rospy
from tf import transformations
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovariance
from nav_msgs.msg import Odometry

DIR = os.path.dirname(__file__)
CONTROL_INPUT = 'data/noboru-log-ut.txt'

WHEEL_BASE = 1.04  # (m)
BASE_LINK_ID = 'base_link'
ODOM_LINK_ID = 'wodom'

ELAPSSED_TIME = 0
GEOM_POSE = Pose()


def convert_twist_msgs(ut):
  # ut = [timestamp,steering_angle,velocity_speed]
  msgs = TwistStamped()

  stamp_sec = int(ut[0])
  stamp_nsec = int((float(ut[0])-float(stamp_sec))*(10**9))
  msgs.header.stamp.set(stamp_sec, stamp_nsec)
  msgs.header.frame_id = BASE_LINK_ID
  msgs.twist.linear.x = ut[2]  # wheel speed
  msgs.twist.angular.z = ut[2]*math.tan(ut[1])/WHEEL_BASE  # angular velocity

  return msgs


def convert_odom_msgs(twist):
  global ELAPSSED_TIME
  global GEOM_POSE

  msgs = Odometry()  # nav_msgs

  msgs.header.stamp = twist.header.stamp  # copy timestamp
  msgs.header.frame_id = ODOM_LINK_ID
  msgs.child_frame_id = BASE_LINK_ID

  msgs.twist.twist = twist_msgs.twist  # copy twist

  # calculate wheel odometry
  quat = [GEOM_POSE.orientation.x, GEOM_POSE.orientation.y, GEOM_POSE.orientation.z, GEOM_POSE.orientation.w]
  (roll, pitch, yaw) = transformations.euler_from_quaternion(quat)

  GEOM_POSE.position.x = ELAPSSED_TIME * twist_msgs.twist.linear.x*math.cos(yaw) + GEOM_POSE.position.x
  GEOM_POSE.position.y = ELAPSSED_TIME * twist_msgs.twist.linear.x*math.sin(yaw) + GEOM_POSE.position.y
  msgs.pose.pose.position = GEOM_POSE.position

  new_yaw = ELAPSSED_TIME * twist_msgs.twist.angular.z + yaw
  _quat = transformations.quaternion_from_euler(
      roll,
      pitch,
      new_yaw)

  msgs.pose.pose.orientation = Quaternion(_quat[0], _quat[1], _quat[2], _quat[3])
  GEOM_POSE.orientation = Quaternion(_quat[0], _quat[1], _quat[2], _quat[3])

  return msgs


if __name__ == '__main__':
  global ELAPSSED_TIME
  global GEOM_POSE

  print('Noboru log publisher')

  # data load
  ut = np.loadtxt(DIR+'/../'+CONTROL_INPUT)

  # bringup ros node
  rospy.init_node('noboru_log_publisher', anonymous=True)
  twist_pub = rospy.Publisher('/soma/vel', TwistStamped, queue_size=10)
  odom_pub = rospy.Publisher('/soma/wodom', Odometry, queue_size=10)
  rate = rospy.Rate(10)  # Hz

  n = 0
  max_n = len(ut)
  ELAPSSED_TIME = 0
  GEOM_POSE = Pose()
  while not rospy.is_shutdown():
    if n >= max_n:
      break

    print(ut[n])
    print(GEOM_POSE.position.x, GEOM_POSE.position.y)

    ELAPSSED_TIME = ut[n][0] - ut[n-1][0]
    print(ELAPSSED_TIME)

    # make twist message, publish
    twist_msgs = convert_twist_msgs(ut[n])
    twist_pub.publish(twist_msgs)

    # make odometry message, publish
    odom_msgs = convert_odom_msgs(twist_msgs)
    odom_pub.publish(odom_msgs)

    # rate.sleep()
    rospy.Duration()
    n = n+1
