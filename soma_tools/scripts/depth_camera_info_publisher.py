#!/usr/bin/python
import math
import numpy as np
import os
import time
import rospy

from sensor_msgs.msg import CameraInfo

if __name__ == "__main__":
  rospy.init_node('camera_info_publisher', anonymous=True)
  f_cam_info_pub = rospy.Publisher('camera_F/depth/camera_info', CameraInfo, queue_size=10)
  b_cam_info_pub = rospy.Publisher('camera_B/depth/camera_info', CameraInfo, queue_size=10)

  f_cam_info_msg = CameraInfo()
  f_cam_info_msg.header.stamp = rospy.Time.now()
  f_cam_info_msg.header.frame_id = 'camera_F_depth_optical_frame'

  f_cam_info_msg.height = 480
  f_cam_info_msg.width = 640
  f_cam_info_msg.distortion_model = 'plumb_bob'
  f_cam_info_msg.D = [0., 0., 0., 0., 0.]
  f_cam_info_msg.K = [382.595764,
                      0.0,
                      320.434540,
                      0.0,
                      382.595764,
                      242.255585,
                      0.0,
                      0.0,
                      1.0]
  f_cam_info_msg.R = [1.0,
                      0.0,
                      0.0,
                      0.0,
                      1.0,
                      0.0,
                      0.0,
                      0.0,
                      1.0]
  f_cam_info_msg.P = [
      382.595764,
      0.0,
      320.434540,
      0.0,
      0.0,
      382.595764,
      242.255585,
      0.0,
      0.0,
      0.0,
      1.0,
      0.0
  ]
  f_cam_info_msg.binning_x = 0
  f_cam_info_msg.binning_y = 0
  f_cam_info_msg.roi.x_offset = 0
  f_cam_info_msg.roi.y_offset = 0
  f_cam_info_msg.roi.height = 0
  f_cam_info_msg.roi.width = 0
  f_cam_info_msg.roi.do_rectify = False

  rate = rospy.Rate(10)
  while not rospy.is_shutdown():
    f_cam_info_pub.publish(f_cam_info_msg)
    # b_cam_info_pub.publish(f_cam_info_msg)
    rate.sleep()
    # rospy.spin()
