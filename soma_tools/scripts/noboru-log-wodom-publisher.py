#!/usr/bin/python

#
# /soma/ut in noboru log to TwistStamped message
#

import math
import numpy as np
import os
import rospy
from tf import transformations
from tf import TransformBroadcaster
from std_msgs.msg import *
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovariance
from nav_msgs.msg import Odometry


WHEEL_BASE = 1.04  # (m)
BASE_LINK_ID = 'base_link'
ODOM_LINK_ID = 'wodom'

ODOM_MSG = None

TWIST_PUB = None
ODOM_PUB = None
LAST_STAMP = None  # sec

ODOM_TF_BROADCASTER = None


def convert_twist_msgs(ut):
  #
  # ut: [steering_angle(rad),wheel_speed(m/s)]
  #
  msgs = Twist()

  # stamp_sec = int(ut[0])
  # stamp_nsec = int((float(ut[0])-float(stamp_sec))*(10**9))
  # msgs.header.stamp.set(stamp_sec, stamp_nsec)
  # msgs.header.frame_id = BASE_LINK_ID
  msgs.linear.x = ut[1]  # wheel speed
  msgs.angular.z = ut[1]*math.tan(ut[0])/WHEEL_BASE  # angular velocity

  return msgs


def ut_callback(msg):
  global LAST_STAMP
  global ODOM_MSG
  global TWIST_PUB

  _msg = TwistStamped()
  _msg.header.stamp = rospy.Time.now()
  _msg.header.frame_id = BASE_LINK_ID

  if LAST_STAMP == None:
    elapsed_time = 0.0
  else:
    elapsed_time = (_msg.header.stamp - LAST_STAMP).to_sec()

  _msg.twist = convert_twist_msgs(msg.data)
  TWIST_PUB.publish(_msg)  # twist feedback publish
  #
  #
  #
  ODOM_MSG.header.stamp = rospy.Time.now()
  ODOM_MSG.header.frame_id = ODOM_LINK_ID
  ODOM_MSG.child_frame_id = BASE_LINK_ID

  ODOM_MSG.twist.twist = _msg.twist  # copy twist

  q = [ODOM_MSG.pose.pose.orientation.x,
       ODOM_MSG.pose.pose.orientation.y,
       ODOM_MSG.pose.pose.orientation.z,
       ODOM_MSG.pose.pose.orientation.w]
  (roll, pitch, yaw) = transformations.euler_from_quaternion(q)

  last_position = ODOM_MSG.pose.pose.position
  new_position = Point()
  new_position.x = last_position.x + elapsed_time * _msg.twist.linear.x*math.cos(yaw)
  new_position.y = last_position.y + elapsed_time * _msg.twist.linear.x*math.sin(yaw)
  new_yaw = yaw + elapsed_time * _msg.twist.angular.z

  new_q = transformations.quaternion_from_euler(
      roll,
      pitch,
      new_yaw)

  ODOM_MSG.pose.pose.position = new_position
  ODOM_MSG.pose.pose.orientation = Quaternion(new_q[0], new_q[1], new_q[2], new_q[3],)

  ODOM_MSG.twist.covariance = [0.01, 0, 0, 0, 0, 0,
                               0, 0.01, 0, 0, 0, 0,
                               0, 0, 0.01, 0, 0, 0,
                               0, 0, 0, math.radians(0.01), 0, 0,
                               0, 0, 0, 0, math.radians(0.01), 0,
                               0, 0, 0, 0, 0, math.radians(0.01), ]
  ODOM_MSG.pose.covariance = [0.5, 0, 0, 0, 0, 0,
                              0, 0.1, 0, 0, 0, 0,
                              0, 0, 0, 0, 0, 0,
                              0, 0, 0, math.radians(0.1), 0, 0,
                              0, 0, 0, 0, math.radians(0.1), 0,
                              0, 0, 0, 0, 0, math.radians(0.1), ]  # 36 elements

  ODOM_PUB.publish(ODOM_MSG)  # wheel odometry publish
  #
  #
  #
  # make wodom tf
  # odom_tf = TransformStamped()
  # odom_tf.header.stamp = ODOM_MSG.header.stamp
  # odom_tf.header.frame_id = ODOM_LINK_ID
  # odom_tf.child_frame_id = BASE_LINK_ID
  # odom_tf.transform.translation.x = ODOM_MSG.pose.pose.position.x
  # odom_tf.transform.translation.y = ODOM_MSG.pose.pose.position.y
  # odom_tf.transform.translation.z = ODOM_MSG.pose.pose.position.z
  # odom_tf.transform.rotation = new_q
  ODOM_TF_BROADCASTER.sendTransform((ODOM_MSG.pose.pose.position.x,
                                     ODOM_MSG.pose.pose.position.y,
                                     ODOM_MSG.pose.pose.position.z),
                                    new_q,
                                    rospy.Time.now(),
                                    BASE_LINK_ID,
                                    ODOM_LINK_ID)
  #
  #
  #
  LAST_STAMP = _msg.header.stamp  # last timestamp


if __name__ == '__main__':
  global ODOM_MSG
  global TWIST_PUB
  global ODOM_TF_BROADCASTER

  rospy.init_node('twist_publisher', anonymous=True)
  ut_sub = rospy.Subscriber('/soma/ut', Float32MultiArray, callback=ut_callback, queue_size=10)

  ODOM_MSG = Odometry()
  TWIST_PUB = rospy.Publisher('/soma/vel', TwistStamped, queue_size=10)
  ODOM_PUB = rospy.Publisher('/soma/wodom', Odometry, queue_size=10)
  ODOM_TF_BROADCASTER = TransformBroadcaster()

  rospy.spin()
