#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Float32
import geometry_msgs
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from tf.transformations import quaternion_from_euler
from tf import TransformBroadcaster
import tf2_ros
import tf_conversions
from maxon_epos_msgs.msg import MotorState
from maxon_epos_msgs.msg import MotorStates
from math import sin, cos, tan
import copy

WHEEL_BASE = 1.04
TIMER_T = 0.25

# frame id
base_link = 'soma_link'
frame_id = 'wodom'

#global variable
v = 0.0
phi = 0.0
X_t = [0.0, 0.0, 0.0]  # odometry x,y,theta estimated


def callback_wheel_vel(data):
    # rospy.loginfo('wheel vel:'+str(data.data))
    global v
    v = data.data  # velocity [m/s]


def callback_steering_state(data):
    global phi
    phi = data.position  # steering angle [rad]


def timer_callback(event):
    global v
    global phi
    _v = copy.copy(v)
    _phi = copy.copy(phi)
    rospy.loginfo('(v,phi)=({:.2f}, {:.2f})'.format(_v, _phi))

    # Dead Recogning
    dt = TIMER_T
    if abs(_v) != 0.0 and abs(_phi) != 0.0:  # turning
        # front wheel odometry?
        # omega = v*sin(phi)/WHEEL_BASE
        # X_t[0] = X_t[0] - v/omega*sin(X_t[2]) + v/omega*sin(X_t[2] + omega*dt)
        # X_t[1] = X_t[1] + v/omega*cos(X_t[2]) - v/omega*cos(X_t[2] + omega*dt)
        # X_t[2] = X_t[2] + omega*dt
        # rear wheel odometry?
        X_t[0] = X_t[0] + _v*cos(X_t[2])*dt
        X_t[1] = X_t[1] + _v*sin(X_t[2])*dt
        X_t[2] = X_t[2] + _v*tan(_phi)/WHEEL_BASE*dt
    elif abs(_v) != 0.0 and abs(_phi) == 0.0:  # move straight
        X_t[0] = X_t[0] + _v*dt*cos(X_t[2])
        X_t[1] = X_t[1] + _v*dt*sin(X_t[2])
        X_t[2] = X_t[2]
    else:
        # rospy.logwarn('warn {}, {}'.format(_v, _phi))
        X_t[0] = X_t[0]
        X_t[1] = X_t[1]
        X_t[2] = X_t[2]

    rospy.loginfo('(x,y,th)=({:.2f},{:.2f},{:.2f})'.format(
        X_t[0], X_t[1], X_t[2]))

    # publish
    odom = Odometry() #nav_msgs.msg.Odometry
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = frame_id
    odom.child_frame_id = base_link
    # set pose xyz
    odom.pose.pose.position.x = X_t[0]
    odom.pose.pose.position.y = X_t[1]
    odom.pose.pose.position.z = 0.0 #for 2D
    # set pose quaternion as yaw,pitch,roll
    q = quaternion_from_euler(0.0, 0.0, X_t[2])
    odom.pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
    # let's publish
    odom_pub.publish(odom)

    # publish tf odom_dr -> soma_link
    transform = geometry_msgs.msg.TransformStamped()
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = frame_id
    transform.child_frame_id = base_link

    transform.transform.translation.x = X_t[0]
    transform.transform.translation.y = X_t[1]
    transform.transform.translation.z = 0.0
    q2 = tf_conversions.transformations.quaternion_from_euler(0, 0, X_t[2])
    transform.transform.rotation.x = q2[0]
    transform.transform.rotation.y = q2[1]
    transform.transform.rotation.z = q2[2]
    transform.transform.rotation.w = q2[3]
    odom_broadcaster.sendTransform(transform)

    # pos = (X_t[0], X_t[1], 0.0)
    # quat = (q[0], q[1], q[2], q[3])
    # odom_broadcaster.sendTransform(pos,
    #                                quat,
    #                                rospy.Time.now(),
    #                                base_link,
    #                                frame_id)


if __name__ == '__main__':
    rospy.loginfo('Run atv wheel odometry node')
    rospy.init_node('atv_wheel_odometry', anonymous=True)

    # subscriber
    rospy.Subscriber('/soma/wheel_vel', Float32, callback=callback_wheel_vel)
    rospy.Subscriber('/maxon_bringup/steering/get_state',
                     MotorState, callback_steering_state)

    # publishers
    odom_pub = rospy.Publisher('/soma/wheel_odom', Odometry, queue_size=3)
    # odom_broadcaster = TransformBroadcaster()
    odom_broadcaster = tf2_ros.TransformBroadcaster()

    rospy.Timer(rospy.Duration(TIMER_T), timer_callback)
    rospy.spin()
