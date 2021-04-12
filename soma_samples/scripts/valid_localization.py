#!/usr/bin/python3

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
Q_DATASET_NAME = '/home/hayashi/catkin_ws/src/soma_pkg/soma_tools/data/Q-set-test.txt'
TRAIN_X_DATASET_NAME = '/home/hayashi/catkin_ws/src/soma_pkg/soma_tools/data/x_train-1.txt'
TRAIN_Y_DATASET_NAME = '/home/hayashi/catkin_ws/src/soma_pkg/soma_tools/data/y_train-1.txt'
MODEL_NAME = '/home/hayashi/catkin_ws/src/soma_pkg/soma_tools/models/model-x_train-1-n12-e1.h5'

print(tf.__version__)

# load model
model = tf.keras.models.load_model(MODEL_NAME)
model.summary()

q_set = np.loadtxt(Q_DATASET_NAME)
_x_valid = np.loadtxt(TRAIN_X_DATASET_NAME)
x_valid = _x_valid[:, 15:27]
x_valid = preprocessing.minmax_scale(x_valid)
y_valid = np.loadtxt(TRAIN_Y_DATASET_NAME)

print('Q:', q_set.shape)
print('x:', x_valid.shape, type(x_valid[:1]))
print('y:', y_valid.shape, type(y_valid))

eval = model.evaluate(x_valid[:], y_valid[:], verbose=0)
print(eval)

predictions = model.predict(x_valid)
print(predictions.shape)
print(predictions)