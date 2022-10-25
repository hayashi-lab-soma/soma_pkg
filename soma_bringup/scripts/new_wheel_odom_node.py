#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler
from tf import TransformBroadcaster
import message_filters
from maxon_epos_msgs.msg import MotorState
from math import sin, cos, tan, sqrt


# global parameters
WHEEL_BASE = 1.04
AXIS_LENGTH = 0.70  # Distance between the 2 rear wheels

# global variables
wheel_vel = 0.0
steer_phi = 0.0
x = y = theta = 0.0
old_time = None
old_time_float = None


def odom_callback(wheel_vel_data, steering_data):
    global wheel_vel, steer_phi, x, y, theta, old_time, old_time_float

    if old_time == None:
        old_time = wheel_vel_data.header.stamp
        old_time_float = wheel_vel_data.header.stamp.to_sec()
        return

    new_time = wheel_vel_data.header.stamp
    new_time_float = wheel_vel_data.header.stamp.to_sec()

    rospy.loginfo('New Time: {}'.format(new_time))
    rospy.loginfo('New Time in float: {}'.format(new_time_float))
    rospy.loginfo('Old Time: {}'.format(old_time))
    rospy.loginfo('Old Time in float: {}'.format(old_time_float))
    rospy.loginfo('(v, phi)=({:.3f}, {:.3f})'.format(wheel_vel, steer_phi))

    # calculate wheel odometry
    dt = new_time_float-old_time_float
    old_time = new_time
    old_time_float = new_time_float
    rospy.loginfo('(dt)=({:.3f})'.format(dt))
    old_x = x
    old_y = y
    old_theta = theta

    wheel_vel = wheel_vel_data.twist.linear.x
    steer_phi = steering_data.position

    # Straight motion
    if abs(steer_phi) < 1e-3:
        x += wheel_vel*dt*cos(theta)
        y += wheel_vel*dt*sin(theta)
    # Circular motion
    else:
        # Turning left
        if wheel_vel*steer_phi > 0:
            theta += abs(wheel_vel)*dt / \
                sqrt((WHEEL_BASE/abs(tan(steer_phi)) +
                     AXIS_LENGTH/2)**2+WHEEL_BASE**2)
        # Turning right
        else:
            theta -= abs(wheel_vel)*dt / \
                sqrt((WHEEL_BASE/abs(tan(steer_phi)) -
                     AXIS_LENGTH/2)**2+WHEEL_BASE**2)
        x += WHEEL_BASE/tan(steer_phi)*(sin(theta)-sin(old_theta))
        y -= WHEEL_BASE/tan(steer_phi)*(cos(theta)-cos(old_theta))

    rospy.loginfo('(x, y, theta)=({:.3f}, {:.3f}, {:.3f})'.format(x, y, theta))

    # publish odometry message
    odom = Odometry()  # nav_msgs.msg.Odometry
    odom.header.stamp = new_time
    odom.header.frame_id = ODOM_FRAME_ID
    odom.child_frame_id = BASE_FRAME_ID
    # set pose
    odom.pose.pose.position.x = x
    odom.pose.pose.position.y = y
    # To check theta value directly (need to be removed later)
    odom.pose.pose.position.z = theta
    q = quaternion_from_euler(0.0, 0.0, theta)
    odom.pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

    odom.twist.twist.linear.x = (x-old_x)/dt
    odom.twist.twist.linear.y = (y-old_y)/dt
    odom.twist.twist.linear.z = 0.0
    odom.twist.twist.angular.x = 0.0
    odom.twist.twist.angular.y = 0.0
    odom.twist.twist.angular.z = (theta-old_theta)/dt

    odom_pub.publish(odom)

    # broadcast wodom -> base_link
    transform = TransformStamped()
    transform.header.stamp = new_time
    transform.header.frame_id = ODOM_FRAME_ID
    transform.child_frame_id = BASE_FRAME_ID
    transform.transform.translation = odom.pose.pose.position
    transform.transform.rotation = odom.pose.pose.orientation
    tf_broadcaster.sendTransformMessage(transform)

    return


if __name__ == '__main__':
    global BASE_FRAME_ID
    global ODOM_FRAME_ID

    rospy.init_node('wheel_odom_node', anonymous=True)

    # arguments
    BASE_FRAME_ID = rospy.get_param('~base_frame_id', 'base_link')
    ODOM_FRAME_ID = rospy.get_param('~odom_frame_id', 'wodom')
    # publishers
    odom_pub = rospy.Publisher('/soma/wheel_odom', Odometry, queue_size=5)
    tf_broadcaster = TransformBroadcaster()  # tf1 ver.
    # subscribers (synchronized)
    wheel_vel_sub = message_filters.Subscriber('/wheel_vel', TwistStamped)
    steering_sub = message_filters.Subscriber(
        '/maxon/steering/state', MotorState)

    sync = message_filters.ApproximateTimeSynchronizer(
        [wheel_vel_sub, steering_sub], queue_size=1, slop=0.5, allow_headerless=True)
    sync.registerCallback(odom_callback)

    rospy.spin()
