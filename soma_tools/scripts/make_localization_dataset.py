import numpy as np
import itertools
import math
import scipy.spatial as ss
from scipy.spatial import distance

# Definitions
TREE_LOCATION_PATH = '/home/hayashi/catkin_ws/src/soma_pkg/soma_tools/data/TreeLocations_Mirais.txt'
TRAIN_DATASET = '/home/hayashi/catkin_ws/src/soma_pkg/soma_tools/data/train_dataset.txt'


X_RANGE = [0.0, 30.0]
Y_RANGE = [0.0, 30.0]
TH_RANGE = [-math.pi, math.pi]

DX = 10.0  # (m)
DY = 10.0  # (m)
DTH = 90  # (deg)


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
    kdTree = ss.KDTree(tree_locations[:, 1:], leafsize=4)
    dataset = []

    # iteration and make dataset
    for (x, y, th) in Q:
        dists, indices = kdTree.query((x, y), eps=0.01, k=3)
        tmp = [x, y, th]  # sampled state vector

        # print(indices)
        for i in indices:
            # Global coord.
            tmp.append(tree_locations[i][1])
            tmp.append(tree_locations[i][2])

        mat = np.array([
            [math.cos(th), -math.sin(th)],
            [math.sin(th), math.cos(th)]]
        )

        for i in indices:
            # local coord.
            _pos = np.array([[tree_locations[i][1]-x],
                             [tree_locations[i][2]-y]]
                            )
            pos = np.dot(mat, _pos)
            tmp.append(pos[0][0])
            tmp.append(pos[1][0])

        # tree IDs
        for i in indices:
            tmp.append(tree_locations[i][0])

        dataset.append(tmp)

    # polar coordinates
    local_coords = np.array(dataset)[:, 12:]
    tmp = []
    for row in local_coords:
        _tmp = []

        p_alpha = row[0:2]
        p_beta = row[2:4]
        p_gamma = row[4:6]

        r_alpha = distance.euclidean(p_alpha, [0, 0])
        r_beta = distance.euclidean(p_beta, [0, 0])
        r_gamma = distance.euclidean(p_gamma, [0, 0])

        phi_alpha = math.atan2(p_alpha[1], p_alpha[0])
        phi_beta = math.atan2(p_beta[1], p_beta[0])
        phi_gamma = math.atan2(p_gamma[1], p_gamma[0])

        d_alpha_beta = distance.euclidean(p_alpha, p_beta)
        d_beta_gamma = distance.euclidean(p_beta, p_gamma)
        d_gamma_alpha = distance.euclidean(p_gamma, p_alpha)

        psi_alpha_beta = math.atan2((p_beta[1]-p_alpha[1]),
                                    (p_beta[0]-p_alpha[0]))
        psi_beta_gamma = math.atan2((p_gamma[1]-p_beta[1]),
                                    (p_gamma[0]-p_beta[0]))
        psi_gamma_alpha = math.atan2((p_alpha[1]-p_gamma[1]),
                                     (p_alpha[0]-p_gamma[0]))

        _tmp.append(r_alpha)
        _tmp.append(r_beta)
        _tmp.append(r_gamma)
        _tmp.append(phi_alpha)
        _tmp.append(phi_beta)
        _tmp.append(phi_gamma)
        _tmp.append(d_alpha_beta)
        _tmp.append(d_beta_gamma)
        _tmp.append(d_gamma_alpha)
        _tmp.append(psi_alpha_beta)
        _tmp.append(psi_beta_gamma)
        _tmp.append(psi_gamma_alpha)

        tmp.append(_tmp)

    dataset = np.array(dataset)
    tmp = np.array(tmp)
    print(dataset.shape, tmp.shape)
    dataset = np.hstack((dataset, tmp))
    print(dataset.shape)

    np.savetxt(TRAIN_DATASET, dataset, fmt='%.3f')
