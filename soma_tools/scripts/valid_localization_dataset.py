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


#current file directry
DIR=os.path.dirname(__file__)
MODEL_NAME = DIR+'/../'+'models/model-x_train-1-n12-e1.h5'

print(MODEL_NAME)