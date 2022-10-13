#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler
from tf import TransformBroadcaster
from maxon_epos_msgs.msg import MotorState
from maxon_epos_msgs.msg import MotorStates
from math import sin, cos, tan, sqrt

def callback_wheel_vel(data):
    vel= TwistStamped()
    vel.header.stamp = rospy.Time.now()
    vel.twist.linear.x=data.data
    vel_pub.publish(vel)

def callback_steering_state(data):
    steer= MotorState()
    steer.header.stamp = rospy.Time.now()
    steer.position=data.position
    steer_pub.publish(steer)

if __name__ == '__main__':

    rospy.init_node('add_time_stamp', anonymous=True)

    # subscriber
    rospy.Subscriber('/soma/wheel_vel', Float32, callback=callback_wheel_vel)
    rospy.Subscriber('/maxon_bringup/steering/get_state',
                     MotorState, callback_steering_state)

    # publishers
    vel_pub = rospy.Publisher('/soma/wheel_vel_new', TwistStamped, queue_size=1)
    steer_pub = rospy.Publisher('/soma/steering_angle', MotorState, queue_size=1)
    rospy.spin()