#!/usr/bin/env python3

import os
# import pathlib
import sys
import numpy as np
import datetime
import time
from sklearn import preprocessing

# tensorflow,keras by functional API
import tensorflow as tf
import tensorflow.keras
from tensorflow.keras import models
from tensorflow.keras import layers


if __name__=='__main__':
  #current file directry
  # DIR = os.path.abspath(__file__)
  MODEL_NAME='/home/hayashi/catkin_ws/src/soma_pkg/soma_tools/models/model-x_train-2-n6-e2000.h5'
  # print(os.path.dirname(__file__))
  model = tf.keras.models.load_model(MODEL_NAME,compile=False)

