#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler
from tf import TransformBroadcaster


initial_x = None
initial_y = None


def odom_callback(data):
    global initial_x, initial_y

    new_time = data.header.stamp

    # publish odometry message
    odom = Odometry()  # nav_msgs.msg.Odometry
    odom.header.stamp = new_time
    odom.header.frame_id = ODOM_FRAME_ID
    odom.child_frame_id = BASE_FRAME_ID
    # set pose
    if initial_x == None and initial_y == None:
        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        initial_x = data.pose.pose.position.x
        initial_y = data.pose.pose.position.y
    else:
        tmp_x = data.pose.pose.position.x - initial_x
        tmp_y = data.pose.pose.position.y - initial_y
        odom.pose.pose.position.x = -tmp_y
        odom.pose.pose.position.y = tmp_x

    odom.pose.pose.position.z = 0.0
    q = quaternion_from_euler(0.0, 0.0, 0.0)
    odom.pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

    odom.twist.twist.linear.x = 0.0
    odom.twist.twist.linear.y = 0.0
    odom.twist.twist.linear.z = 0.0
    odom.twist.twist.angular.x = 0.0
    odom.twist.twist.angular.y = 0.0
    odom.twist.twist.angular.z = 0.0

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
    rospy.init_node('gps_relative_odom', anonymous=True)

    # arguments
    BASE_FRAME_ID = rospy.get_param('~base_frame_id', 'gps')
    ODOM_FRAME_ID = rospy.get_param('~odom_frame_id', 'wodom')
    # publishers
    odom_pub = rospy.Publisher('/gps_relative_odom', Odometry, queue_size=5)
    tf_broadcaster = TransformBroadcaster()
    # subscriber
    odom_sub = rospy.Subscriber('/gps_odom',
                                Odometry, callback=odom_callback, queue_size=1)

    rospy.spin()
