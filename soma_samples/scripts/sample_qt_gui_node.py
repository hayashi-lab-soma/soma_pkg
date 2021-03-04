#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def main():
    rospy.init_node('sample_talker',anonymous=True)
    pub=rospy.Publisher('chatter',String,queue_size=10)
    rate=rospy.Rate(10)
    while not rospy.is_shutdown():
        msg = "hello world %s" % rospy.get_time()
        pub.publish(msg)
        rate.sleep()

if __name__=='__main__':
    main()