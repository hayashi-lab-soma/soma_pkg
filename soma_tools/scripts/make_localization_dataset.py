import numpy as np
import itertools
import math
import scipy.spatial as ss

# Definitions
TREE_LOCATION_PATH = './soma_tools/data/TreeLocations_Mirais.txt'
TRAIN_DATASET = './soma_tools/data/train_dataset.txt'

X_RANGE = [0.0, 30.0]
Y_RANGE = [0.0, 30.0]
TH_RANGE = [-math.pi, math.pi]

DX = 1.0  # (m)
DY = 1.0  # (m)
DTH = 1  # (deg)


def uniform_sampling(x_range: [], y_range: [], th_range: [], nx, ny, nth):
    # print(nx, ny, nth)
    q_set = []

    x_set = np.arange(x_range[0], x_range[1], step=nx)
    y_set = np.arange(y_range[0], y_range[1], step=ny)
    th_set = np.arange(th_range[0], th_range[1], step=nth)

    q_set = list(itertools.product(x_set, y_set, th_set))
    return q_set


if __name__ == '__main__':
    Q = uniform_sampling(X_RANGE,
                         Y_RANGE,
                         TH_RANGE,
                         DX, DY, math.radians(DTH))
    print('total qspace sample: {}'.format(len(Q)))

    tree_locations = np.loadtxt(TREE_LOCATION_PATH, comments='#')
    print('tree spread range =>')
    print('x min:{},  x max:{}'.format(
        np.min(tree_locations[:, 1]),
        np.max(tree_locations[:, 1])))
    print('y min:{},  y max:{}'.format(
        np.min(tree_locations[:, 2]),
        np.max(tree_locations[:, 2])))

    # Sort trees by neighborhood ordering
    kdTree = ss.KDTree(tree_locations[:, :2], leafsize=4)
    dataset = []

    # iteration and make dataset
    for (x, y, th) in Q:
        dists, indices = kdTree.query((x, y), eps=0.01, k=3)
        tmp = [x, y, th]  # sampled state vector

        for i in indices:
            # Global coord.
            tmp.append(tree_locations[i][0])
            tmp.append(tree_locations[i][1])

        mat = np.array([
            [math.cos(th), -math.sin(th)],
            [math.sin(th), math.cos(th)]]
            )

        for i in indices:
            # local coord.
            _pos = np.array([[tree_locations[i][0]-x],
                             [tree_locations[i][1]-y]]
                             )
            pos = np.dot(mat,_pos)
            tmp.append(pos[0][0])
            tmp.append(pos[1][0])

        dataset.append(tmp)

    np.savetxt(TRAIN_DATASET, dataset, fmt='%.3f')
