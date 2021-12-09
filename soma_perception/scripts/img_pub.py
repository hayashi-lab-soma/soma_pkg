#!/usr/bin/env python3
from __future__ import absolute_import
from __future__ import division
import argparse
from functools import partial
from config import cfg, assert_and_infer_cfg
import logging
import math
import os
import sys
import time
# ROS libs
import rospy
import ros_numpy
from sensor_msgs.msg import Image as Image1
from sensor_msgs.msg import CompressedImage
#from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
import torch
import numpy as np
from datasets import cityscapes, rellis
import yaml
from utils.misc import AverageMeter, prep_experiment, evaluate_eval, fast_hist
from utils.f_boundary import eval_mask_boundary
import datasets
import loss
import network
import optimizer
from tqdm import tqdm
from PIL import Image as Image2
import transforms.joint_transforms as joint_transforms
import transforms.transforms as extended_transforms
import torchvision.transforms as standard_transforms

def main():
    rospy.init_node('pub_seg',anonymous=True)

    pub = rospy.Publisher('/Rellis/GSCNN_seg/raw_data', Image1, queue_size=10) #raw segmentation output
    im1 = cv2.imread('/home/hayashi/catkin_ws/src/soma_pkg/soma_perception/prediction/color/seg./seg.png', cv2.IMREAD_COLOR)
    #im = ros_numpy.numpify(im)
    #im1 = Image2.open('/home/hayashi/catkin_ws/src/soma_pkg/soma_perception/prediction/color/seg./seg.png').convert('RGB')
    #im = ros_numpy.numpify(im1)
    imgmsg = ros_numpy.msgify(Image1, im1, encoding= 'bgr8')
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        pub.publish(imgmsg)
        rate.sleep()
    #pub.publish(imgmsg)
    rospy.spin()

if __name__ == '__main__':



    try:
        main()
        
    except KeyboardInterrupt:
        print("Shutting Down Node!!!")
        sys.exit(0)