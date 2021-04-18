#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler
from tf import TransformBroadcaster
from maxon_epos_msgs.msg import MotorState
from maxon_epos_msgs.msg import MotorStates
from math import sin, cos, tan

# global parameters
WHEEL_BASE = None
DURATION = None

# global variables
wheel_vel = 0.0
steer_phi = 0.0
x = y = theta = 0.0


def callback_wheel_vel(data):
  global wheel_vel
  wheel_vel = data.data


def callback_steering_state(data):
  global steer_phi
  steer_phi = data.position  # steering angle [rad]


def timer_callback(event):
  global x, y, theta

  rospy.loginfo('Timer callback: {}'.format(rospy.Time.now().to_time()))
  rospy.loginfo('(v, phi)=({:.3f}, {:.3f})'.format(wheel_vel, steer_phi))

  # calculate wheel odometry
  dt = DURATION
  # _x = wheel_vel*cos(theta)*dt
  # _y = wheel_vel*sin(theta)*dt
  # _theta = wheel_vel*tan(steer_phi)/WHEEL_BASE*dt
  _x = wheel_vel*cos(theta)
  _y = wheel_vel*sin(theta)
  _theta = wheel_vel*tan(steer_phi)/WHEEL_BASE

  # x = x + _x
  # y = y + _y
  theta = theta + _theta*dt

  rospy.loginfo('(x, y, th)=({:.3f}, {:.3f}, {:.3f})'.format(x, y, theta))

  # publish odometry message
  odom = Odometry()  # nav_msgs.msg.Odometry
  odom.header.stamp = rospy.Time.now()
  odom.header.frame_id = ODOM_FRAME_ID
  odom.child_frame_id = BASE_FRAME_ID
  # set pose
  # odom.pose.pose.position.x = x
  # odom.pose.pose.position.y = y
  # odom.pose.pose.position.z = 0.0  # 2D
  # q = quaternion_from_euler(0.0, 0.0, theta)
  # odom.pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
  odom.twist.twist.linear.x = _x
  odom.twist.twist.linear.y = _y
  q = quaternion_from_euler(0.0, 0.0, _theta)
  odom.twist.twist.angular = Quaternion(q[0], q[1], q[2], q[3])
  # set coveriance
  odom.pose.covariance[0] = 2.0 #sigma_xx
  odom.pose.covariance[7] = 2.0 #sigma_yy
  odom.pose.covariance[14] = 0.0 #sigma_zz
  odom.pose.covariance[21] = 0.001 #sigma_roll,roll
  odom.pose.covariance[28] = 0.001 #sigma_pitch,pitch
  odom.pose.covariance[35] = 0.001 #sigma_yaw,yaw
  odom_pub.publish(odom)

  # broadcast wodom -> base_link
  transform = TransformStamped()
  transform.header.stamp = rospy.Time.now()
  transform.header.frame_id = ODOM_FRAME_ID
  transform.child_frame_id = BASE_FRAME_ID
  transform.transform.translation = odom.pose.pose.position
  transform.transform.rotation = odom.pose.pose.orientation
  # tf_broadcaster.sendTransformMessage(transform)

if __name__ == '__main__':
  global BASE_FRAME_ID
  global ODOM_FRAME_ID
  global WHEEL_BASE

  rospy.init_node('wheel_odom_node', anonymous=True)

  # arguments
  BASE_FRAME_ID = rospy.get_param('~base_frame_id', 'base_link')
  ODOM_FRAME_ID = rospy.get_param('~odom_frame_id', 'wodom')
  WHEEL_BASE = rospy.get_param('~wheel_base', 1.04)
  DURATION = rospy.get_param('~publish_duration', 0.1)

  # subscriber
  rospy.Subscriber('/wheel_vel', Float32, callback=callback_wheel_vel)
  rospy.Subscriber('/maxon/steering/state',
                   MotorState, callback_steering_state)

  # publishers
  odom_pub = rospy.Publisher('/soma/wheel_odom', Odometry, queue_size=5)
  tf_broadcaster = TransformBroadcaster() # tf1 ver.

  rospy.Timer(rospy.Duration(DURATION), timer_callback)
  rospy.spin()
