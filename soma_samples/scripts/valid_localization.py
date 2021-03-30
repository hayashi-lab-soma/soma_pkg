#!/usr/bin/python

import os
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

TREE_LOCATION_PATH = '/home/hayashi/catkin_ws/src/soma_pkg/soma_tools/data/TreeLocations_Mirais.txt'
TRAIN_X_DATASET_NAME = '/home/hayashi/catkin_ws/src/soma_pkg/soma_tools/data/x_train-test.txt'
TRAIN_Y_DATASET_NAME = '/home/hayashi/catkin_ws/src/soma_pkg/soma_tools/data/y_train-test.txt'
MODEL_NAME='/home/hayashi/catkin_ws/src/soma_pkg/soma_tools/models/model-test.h5'

#load model
model = tf.keras.models.load_model(MODEL_NAME)
model.summary()