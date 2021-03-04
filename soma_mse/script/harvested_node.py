#!/usr/bin/env python
import roslib
import rospy
import math
import tf
import geometry_msgs

roslib.load_manifest('octomap_server')
import octomap_msgs.srv

SRV_NAME='/octomap_server_nodelet/clear_bbx'
SRV_INTERFACE=octomap_msgs.srv.BoundingBoxQuery

if __name__ == '__main__':
  rospy.init_node('harvested_node')
  listener = tf.TransformListener()

  while True:
    try:
      listener.waitForTransform('/map','/soma_link',rospy.Time(),rospy.Duration(1.0))
      tf_ = listener.lookupTransform('/map','/soma_link',rospy.Time(0))
      break
    except (tf.LookupException):
      print('not found tf')

  print('Got tf')

  min_pt = geometry_msgs.msg.PointStamped()
  min_pt.header.frame_id = 'soma_link'
  min_pt.point.x = 0.5
  min_pt.point.y = -1.0
  min_pt.point.z = -10.0

  max_pt = geometry_msgs.msg.PointStamped()
  max_pt.header.frame_id = 'soma_link'
  max_pt.point.x = 3.5
  max_pt.point.y = 1.0
  max_pt.point.z = 10.0

  _min_pt = listener.transformPoint("/map", min_pt)
  _max_pt = listener.transformPoint("/map", max_pt)


  min = geometry_msgs.msg.Point()
  max = geometry_msgs.msg.Point()

  min.x = _min_pt.point.x
  min.y = _min_pt.point.y
  min.z = _min_pt.point.z

  max.x = _max_pt.point.x
  max.y = _max_pt.point.y
  max.z = _max_pt.point.z

  service = rospy.ServiceProxy(SRV_NAME,SRV_INTERFACE)
  service(min, max)
