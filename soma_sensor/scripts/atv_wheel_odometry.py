#!/usr/bin/python2
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
from tf import TransformBroadcaster
from maxon_epos_msgs.msg import MotorStates

WHEEL_BASE = 1.04
TIMER_T = 0.2

# frame id
frame_id = 'wodom'
base_link = 'soma_link'

#global variable
v = 0.0
phi = 0.0
X_t = [0.0, 0.0, 0.0]  # odometry x,y,theta estimated


def callback_wheel_vel(data):
    rospy.loginfo('wheel vel:'+str(data.data))
    v = data.data  # velocity [m/s]


def callback_motor_states(data):
    phi = data.states[0].position


def timer_callback(event):
    rospy.loginfo('timer callback')
    rospy.loginfo('(v,phi)=({:.2f},{:.2f})'.format(v, phi))

    # Dead Recogning
    dt = TIMER_T
    if v != 0.0 and phi != 0.0: #turning
        omega = v*sin(phi)/WHEEL_BASE
        X_t[0] = X_t[0] - v/omega*sin(X_t[2]) + v/omega*sin(X_t[2] + omega*dt)
        X_t[1] = X_t[1] + v/omega*cos(X_t[2]) - v/omega*cos(X_t[2] + omega*dt)
        X_t[2] = X_t[2] + omega*dt
    elif v != 0.0 and phi == 0.0: #move straight
        X_t[0] = X_t[0] + v*dt*cos(X_t[2])
        X_t[1] = X_t[1] + v*dt*sin(X_t[2])
        X_t[2] = X_t[2]
    else:
        pass

    rospy.loginfo('(x,y,th)=({:.2f},{:.2f},{:.2f})'.format(
        X_t[0], X_t[1], X_t[2]))

    #publish /soma/odom_dr
    odom = Odometry()
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = frame_id
    odom.child_frame_id = base_link
    odom.pose.pose.position.x = X_t[0]
    odom.pose.pose.position.y = X_t[1]
    odom.pose.pose.position.z = 0.0
    q = quaternion_from_euler(0.0, 0.0, X_t[2])
    odom.pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
    odom_pub.publish(odom)

    # publish tf odom_dr -> soma_link
    pos = (X_t[0], X_t[1], 0.0)
    quat = (q[0], q[1], q[2], q[3])
    odom_broadcaster.sendTransform(pos,
                                   quat,
                                   rospy.Time.now(),
                                   base_link,
                                   frame_id)


if __name__ == '__main__':
    rospy.loginfo('Run atv wheel odometry node')
    rospy.init_node('atv_wheel_odometry', anonymous=True)
    rospy.Subscriber('/soma/wheel_vel', Float32, callback=callback_wheel_vel)
    rospy.Subscriber('/get_all_states', MotorStates, callback_motor_states)

    # publishers
    odom_pub = rospy.Publisher('/soma/odom_dr', Odometry, queue_size=3)
    odom_broadcaster = TransformBroadcaster()

    rospy.Timer(rospy.Duration(TIMER_T), timer_callback)
    rospy.spin()
