#!/usr/bin/env python

import rosbag
import numpy as np


bag_file_name = '2020-06-02-all-topics.bag'
bag_file_path = '/home/hayashi/Documents/tominaga/' + bag_file_name
print('src file:{}'.format(bag_file_path))

bag = rosbag.Bag(bag_file_path)
print('message_count:{}'.format(bag.get_message_count()))
print('start:{} / end:{} / total:{}'.format(bag.get_start_time(), bag.get_end_time()))