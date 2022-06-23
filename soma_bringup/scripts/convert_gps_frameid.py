#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Quaternion
import tf

class ConverterGpsFrameid():
  def __init__(self):
    self.subscriber = rospy.Subscriber('/fix', NavSatFix, self.callback, queue_size=10)
    self.message = NavSatFix()
    self.pub = rospy.Publisher("/fix/gps", NavSatFix, queue_size=10)

  def callback(self, data):
    self.pub_msg= data
    self.pub_msg.header.frame_id = "gps"
    self.pub_msg.header.stamp = data.header.stamp
    self.pub.publish(self.pub_msg)

if __name__ == '__main__':
  rospy.init_node('ConverterGpsFrameid', anonymous=True)

  converter = ConverterGpsFrameid()

  rospy.spin()