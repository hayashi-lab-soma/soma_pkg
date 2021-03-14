import os
import sys
import numpy as np

# tensorflow,keras by functional API
import tensorflow as tf
import tensorflow.keras
from tensorflow.keras import models
from tensorflow.keras import layers

# Definitions
TREE_LOCATION_PATH = '/home/hayashi/catkin_ws/src/soma_pkg/soma_tools/data/TreeLocations_Mirais.txt'
TRAIN_DATASET = '/home/hayashi/catkin_ws/src/soma_pkg/soma_tools/data/train_dataset.txt'

EPOCH = 10


def make_train_model(num_inputs, num_trees):
    # input layer
    inputs = tf.keras.Input(shape=(num_inputs,))

    # hidden layer
    dense_alpha = layers.Dense(64, activation='relu')(inputs)
    dense_beta = layers.Dense(64, activation='relu')(inputs)
    dense_gamma = layers.Dense(64, activation='relu')(inputs)

    # output layer
    output_alpha = layers.Dense(num_trees, activation='softmax')(dense_alpha)
    output_beta = layers.Dense(num_trees, activation='softmax')(dense_beta)
    output_gamma = layers.Dense(num_trees, activation='softmax')(dense_gamma)

    outputs = layers.Concatenate(axis=1)(
        [output_alpha, output_beta, output_gamma])

    model = tf.keras.Model(inputs=inputs,
                           outputs=outputs,)

    model.compile(loss=tf.keras.losses.SparseCategoricalCrossentropy(),
                  optimizer=tf.keras.optimizers.RMSprop(),)

    model.summary()

    tf.keras.utils.plot_model(model,
                              show_shapes=True,
                              to_file='/home/hayashi/catkin_ws/src/soma_pkg/soma_tools/data/model.png')

    return model


if __name__ == '__main__':
    print(tensorflow.__file__)
    tree_locations = np.loadtxt(TREE_LOCATION_PATH, comments='#')
    num_trees = len(tree_locations)
    print("Number of trees:", num_trees)
    tree_id_arr = tree_locations[:, :1]
    tree_id_arr = tree_id_arr.flatten()
    print(tree_id_arr)

    train_dataset = np.loadtxt(TRAIN_DATASET, comments='#')
    print("Number of train dataset row:", len(train_dataset))
    # print(train_dataset)

    # shape to organized train dataset
    train_dataset = train_dataset[:, 15:]
    # print(train_dataset.shape)
    # print(train_dataset)

    # make supervised vector
    y_train_alpha = []
    y_train_beta = []
    y_train_gamma = []
    # print('supervised vector shape:', y_train.shape)
    for row in train_dataset:
        tree_ids = row[:3]
        alpha = int(tree_ids[0])
        beta = int(tree_ids[1])
        gamma = int(tree_ids[2])

        _y_train_alpha = np.zeros(num_trees)
        _y_train_beta = np.zeros(num_trees)
        _y_train_gamma = np.zeros(num_trees)

        _y_train_alpha[np.where(tree_id_arr == alpha)] = 1.0
        _y_train_beta[np.where(tree_id_arr == beta)] = 1.0
        _y_train_gamma[np.where(tree_id_arr == gamma)] = 1.0

        y_train_alpha.append(_y_train_alpha)
        y_train_beta.append(_y_train_beta)
        y_train_gamma.append(_y_train_gamma)

    y_train_alpha = np.array(y_train_alpha)
    y_train_beta = np.array(y_train_beta)
    y_train_gamma = np.array(y_train_gamma)

    y_train = np.hstack((y_train_alpha, y_train_beta))
    y_train = np.hstack((y_train, y_train_gamma))
    print('y_train shape:', y_train.shape)

    # make x vector
    x_train = train_dataset[:, 3:]
    print('x_train shape: ', x_train.shape)

    model = make_train_model(num_inputs=x_train.shape[1],
                             num_trees=num_trees)

    model.fit(x=x_train, y=y_train, epochs=EPOCH,)
