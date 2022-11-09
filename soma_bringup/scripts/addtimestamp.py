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
# import iir_filter
from scipy import signal

y_old=0
y_old_2=0

def callback_wheel_vel(data):
    vel= TwistStamped()
    vel.header.stamp = rospy.Time.now()
    # f0 = 48.0
    # f1 = 52.0
    # f2 = 400
    # fs = 1000.0
    # sos1 = signal.butter(4, f2/fs*2, output='sos')
    # iir1 = iir_filter.IIR_filter(sos1)
    # vel.twist.linear.x= iir1.filter(data.data)

    # vel.twist.linear.x=data.data
    vel_pub.publish(vel)
    vel1= TwistStamped()
    vel1=vel
    alpha = 0.5
    beta= 0.2
    global y_old
    global y_old_2
    y = beta* y_old_2 + (alpha) * y_old + (1 - alpha -beta) * data.data
    y_old = y
    y_old_2 = y_old
    
    vel1.twist.linear.x= y
    vel1_pub.publish(vel1)

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
    vel1_pub = rospy.Publisher('/soma/wheel_vel_filtered2', TwistStamped, queue_size=1)
    steer_pub = rospy.Publisher('/soma/steering_angle', MotorState, queue_size=1)
    rospy.spin()
