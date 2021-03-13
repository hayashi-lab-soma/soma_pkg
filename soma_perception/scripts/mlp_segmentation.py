#!/usr/bin/env python
import os
import rospy
import roslib
import rosbag
import rosparam
import ros_numpy
import numpy as np
import sensor_msgs.msg as sensor_msgs
from sensor_msgs.msg import PointCloud2

import tensorflow as tf
from tensorflow.keras import layers
from tensorflow.keras import models

INPUT_WEIGHT_DIR = '/home/soma/development/slope_classification6/weight/10/0.5/20210205_032021/20210205_032021.h5'
SIZE_OF_CLOUDS = 4096
BASE_LINK = 'soma_link'


class Publishsers():
  def __init__(self):
    self.publisher = rospy.Publisher('/ground', PointCloud2, queue_size=1)
    # self.slope_pub = rospy.Publisher('/slope', PointCloud2, queue_size=1)
    self.message = PointCloud2()

  def make_msg(self, pc_ary):        
    def _ary_to_pc2(points, frame_id):
      cloud = PointCloud2()
      ros_dtype = sensor_msgs.PointField.FLOAT32
      dtype = np.float32
      itemsize = np.dtype(dtype).itemsize
      data = points.astype(dtype).tobytes()
      fields = [sensor_msgs.PointField(
          name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
          for i, n in enumerate('xyz')]
      cloud.header.frame_id = frame_id
      cloud.header.stamp = rospy.Time.now()
      cloud.height = 1
      cloud.width = points.shape[0]
      cloud.is_dense = False
      cloud.is_bigendian = False
      cloud.fields = fields
      cloud .point_step = (itemsize*3)
      cloud.row_step=(itemsize*3*points.shape[0])
      cloud.data = data
      return cloud
    
    self.message = _ary_to_pc2(pc_ary, BASE_LINK)
    
  def send_msg(self):
    self.publisher.publish(self.message)


class Subscribe_publishers():
  def __init__(self, pub):
    self.subscriber = rospy.Subscriber('/cloud_normal', PointCloud2, self.callback, queue_size=1)
    self.message = PointCloud2()
    self.pub = pub

  def callback(self, data):
    def _convert_to_npary(data):
      print('start callback')
      pc = ros_numpy.numpify(data)
      print('pc.shape', pc.shape)
      print(pc)
      test_clouds = np.zeros((pc.shape[0],6))
      test_clouds[:,0]=pc['x']
      test_clouds[:,1]=pc['y']
      test_clouds[:,2]=pc['z']
      test_clouds[:,3]=pc['normal_x']   
      test_clouds[:,4]=pc['normal_y']
      test_clouds[:,5]=pc['normal_z']
      return test_clouds
    
    def _predict(test_clouds):
      ## reshape test_clouds ##
      x_test = test_clouds[:, 2:]
      x_test = x_test.reshape(-1, 1, 4)
      print('x_train.shape:', x_test.shape)

      ## predict and sort ##
      raw_predictions = model.predict(x_test)
      predictions = np.round(raw_predictions)
      print('predictions', predictions.shape)

      N = x_test.shape[0]
      predictions = predictions.reshape(N,1)
      print('test', test_clouds.shape, 'predictions', predictions.shape)
      predicted_ground = np.empty((0, 3))
      predicted_slope = np.empty((0, 3))
      tmp = np.empty((0,7))
      
      test_clouds = np.hstack((test_clouds, predictions))
      print('concatenate', test_clouds.shape)
      tmp = test_clouds[test_clouds[:,6]==0]
      predicted_ground = tmp[:, 0:3]
      tmp = test_clouds[test_clouds[:,6]==1]
      predicted_slope = tmp[:, 0:3]
      print('ground', predicted_ground.shape, 'slope', predicted_slope.shape)
      return predicted_ground, predicted_slope
    
    model = models.load_model(INPUT_WEIGHT_DIR)
    print(model.summary())

    test_clouds = _convert_to_npary(data)
    print('test_clouds.shape', test_clouds.shape)
    print(test_clouds)
    predicted_ground, predicted_slope = _predict(test_clouds)

    self.pub.make_msg(predicted_ground)
    self.pub.send_msg()

def main():
  rospy.init_node('mlp_segmentation', anonymous=True)

  pub = Publishsers()
  sub = Subscribe_publishers(pub)

  rospy.spin()

if __name__ == '__main__':
   main()