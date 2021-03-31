#!/usr/bin/env python3

import numpy as np
import itertools
import math
import scipy.spatial as ss
from scipy.spatial import distance

# input
Q_SET_PATH = '/home/hayashi/catkin_ws/src/soma_pkg/soma_tools/data/Q-set3.txt'
TREE_LOCATION_PATH = '/home/hayashi/catkin_ws/src/soma_pkg/soma_tools/data/TreeLocations_Mirais.txt'
# output
TRAIN_X_DATASET_NAME = '/home/hayashi/catkin_ws/src/soma_pkg/soma_tools/data/x_train3.txt'
TRAIN_Y_DATASET_NAME = '/home/hayashi/catkin_ws/src/soma_pkg/soma_tools/data/y_train3.txt'


def search_neighbor_trees(M: np.array,
                          q: np.array,):
    kdTree = ss.KDTree(M[:, 1:], leafsize=4)
    dists, indices = kdTree.query((q[0], q[1]), eps=0.01, k=3)

    # unique IDs
    IDs = []
    for i in indices:
        IDs.append(M[i][0])

    # global coord.
    global_coods = []
    for i in indices:
        global_coods.append(M[i][1])
        global_coods.append(M[i][2])

    # local coord.
    local_coords = []
    Tr = np.array([
        [math.cos(thi), -math.sin(thi), 0],
        [math.sin(thi), math.cos(thi), 0],
        [0, 0, 1]]
    )  # rotational matrix

    Tm = np.array(
        [[1, 0, xi],
         [0, 1, yi],
         [0, 0, 1]]
    )  # transform matrix

    for i in indices:
        _pos = np.array([[M[i][1]],
                         [M[i][2]],
                         [1]])
        pos = Tm @ Tr @ _pos
        local_coords.append(pos[0][0])
        local_coords.append(pos[1][0])

    return IDs, global_coods, local_coords


def to_polar(pt: np.array,
             ps=np.array([0.0, 0.0])):
    r = distance.euclidean(pt, ps)
    phi = math.atan2(pt[1]-ps[1], pt[0]-ps[0])

    return r, phi


if __name__ == '__main__':
    # load c-space samples
    Q = np.loadtxt(Q_SET_PATH, delimiter='\t')
    print('total C-Space samples: {}'.format(len(Q)))

    # load tree locations
    tree_locations = np.loadtxt(TREE_LOCATION_PATH, comments='#')
    print('tree spread range =>')
    print(' x min:{},  x max:{}'.format(
        np.min(tree_locations[:, 1]),
        np.max(tree_locations[:, 1])))
    print(' y min:{},  y max:{}'.format(
        np.min(tree_locations[:, 2]),
        np.max(tree_locations[:, 2])))
    print('Number of trees:{}'.format(len(tree_locations)))
    IDs_array = tree_locations[:, 0:1]
    IDs_array = np.ravel(IDs_array)

    # Sort trees by neighborhood ordering
    # top 4 trees
    kdTree = ss.KDTree(tree_locations[:, 1:], leafsize=4)
    dataset = []

    # iteration and make dataset
    for (xi, yi, thi) in Q:
        tmp = [xi, yi, thi]  # append sample state vector

        IDs, global_coords, local_coords = search_neighbor_trees(
            tree_locations, np.array([xi, yi, thi]))

        print(np.array([xi, yi, thi]))
        # print('->', IDs)
        # print('->', global_coords)
        # print('->', local_coords)

        # conver to polar coordinate
        r_alpha, phi_alpha = to_polar(
            pt=np.array([local_coords[0], local_coords[1]]))
        r_beta, phi_beta = to_polar(
            pt=np.array([local_coords[2], local_coords[3]]))
        r_gamma, phi_gamma = to_polar(
            pt=np.array([local_coords[4], local_coords[5]]))

        polar_coords = [r_alpha, phi_alpha,
                        r_beta, phi_beta,
                        r_gamma, phi_gamma]

        # print('->', polar_coords)

        d_alpha_beta, psi_alpha_beta = to_polar(
            pt=np.array([local_coords[2], local_coords[3]]),
            ps=np.array([local_coords[0], local_coords[1]]))
        d_beta_gamma, psi_beta_gamma = to_polar(
            pt=np.array([local_coords[4], local_coords[5]]),
            ps=np.array([local_coords[2], local_coords[3]]))
        d_gamma_alpha, psi_gamma_alpha = to_polar(
            pt=np.array([local_coords[0], local_coords[1]]),
            ps=np.array([local_coords[4], local_coords[5]]))

        relative_polar_coords = [d_alpha_beta, psi_alpha_beta,
                                 d_beta_gamma, psi_beta_gamma,
                                 d_gamma_alpha, psi_gamma_alpha]

        # print('->', relative_polar_coords)

        col = np.concatenate([IDs,
                              global_coords,
                              local_coords,
                              polar_coords,
                              relative_polar_coords])
        print(' ->', col)
        dataset.append(col)
        print('No. col:', len(dataset))

    dataset = np.array(dataset)
    print('shape:', dataset.shape)
    np.savetxt(TRAIN_X_DATASET_NAME, dataset, fmt='%.3f', delimiter='\t')

    # make y_train
    rho_N = []
    for col in dataset:
        rho_array = np.zeros((3, len(tree_locations)))
        # print(rho_array.shape)
        for no, tree_id in enumerate(col[0:3]):
            # print(no,tree_id)
            # print(np.where(IDs_array==tree_id)[0])
            rho_array[no][np.where(IDs_array == int(tree_id))[0]] = 1.0

        rho_N.append(np.ravel(rho_array))

    rho_N = np.array(rho_N)
    # print(rho_N.shape)

    np.savetxt(TRAIN_Y_DATASET_NAME,
               rho_N,
               fmt='%.1f',
               delimiter='\t')
