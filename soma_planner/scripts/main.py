import numpy as np
import math
import time
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import networkx as nx
import sys

from Environment import Environment
from PathPlanner import PathPlanner
import DubinsPath

# Rudias of tree
R_RANGE = [0.3, 0.3]
# Work space range
X_RANGE = [-3, 25]
Y_RANGE = [-3, 35]
OFFSET = 0.0

# Q-Space parameter
NH = 30
LOCAL_PATH_PLANNER = 'DubinsPath'
CURVATURE = 0.43
q_init = [0.0, 0.0, 0.0]

#
INPUT_TREE_LOCATIONS = '/home/hayashi/catkin_ws/src/soma_pkg/soma_mapping/data/tree_locations.txt'

#
OUTPUT_GEN_DIR = '/home/hayashi/catkin_ws/src/soma_pkg/soma_planner/gen/'


def draw_treemap(trees, ax):
    for x, y, r in trees:
        c = patches.Circle(xy=(x, y), radius=r, color='g')
        ax.add_patch(c)


def save_images():
    fig = plt.figure(num=2, figsize=(5, 5), dpi=150)
    ax = plt.axes()
    ax.set_xlim(X_RANGE[0], X_RANGE[1])
    ax.set_ylim(Y_RANGE[0], Y_RANGE[1])
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    draw_treemap(env.Trees, ax)
    nx.draw_networkx_edges(planner.G,
                           pos=planner.pos,
                           edge_color='k')
    # plt.show()
    fig.savefig(OUTPUT_GEN_DIR+'G.png')

    fig = plt.figure(num=3, figsize=(5, 5), dpi=150)
    ax = plt.axes()
    ax.set_xlim(X_RANGE[0], X_RANGE[1])
    ax.set_ylim(Y_RANGE[0], Y_RANGE[1])
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    draw_treemap(env.Trees, ax)
    ax.plot([pi[0] for pi in way_points],
            [pi[1] for pi in way_points],
            'bs-')
    fig.savefig(OUTPUT_GEN_DIR+'way_points.png')


if __name__ == '__main__':
    print('Graph based global trajectory planner')

    env = Environment(input_file=INPUT_TREE_LOCATIONS,
                      x_range=X_RANGE+[OFFSET, -OFFSET],
                      y_range=Y_RANGE+[OFFSET, -OFFSET],
                      r_range=R_RANGE)
    env.show()
    env.view()
    env.save_png(OUTPUT_GEN_DIR+'tree_map.png')
    # exit(1)

    # Path planning
    st = time.time()

    planner = PathPlanner(env.Trees[:, :2])
    way_points = planner.planning()

    et = time.time() - st
    print('Elapsed time:', et)

    planner.save_graphs(OUTPUT_GEN_DIR)

    # make images
    save_images()

    print('Path optimization')

    Q = []
    for k in range(len(way_points)):
        x_k = way_points[k][0]
        y_k = way_points[k][1]

        for h in range(NH):
            ang = h/(NH-1)*360
            ang = ang/180.0*math.pi
            ang_k = math.atan2(math.sin(ang), math.cos(ang))
            ang_k = round(ang_k, 2)

            q_kh = [x_k, y_k, ang_k]  # sampled pose
            Q.append(q_kh)

    print('|Q|=', len(Q))

    print('Make pose graph ==========')
    GQ = nx.DiGraph()

    # add node
    for k in range(len(way_points)):
        for h in range(NH):
            GQ.add_node(h + k*NH)
    print(' Num node: ', GQ.number_of_nodes())

    # add weighted edges
    for k in range(len(way_points)-1):
        for h in range(NH):
            for i in range(NH):
                #print(h + k*(nh), i + (k+1)*(nh))
                # print(Q[h+k*nh],Q[i+(k+1)*nh])
                qi = Q[h + k*NH]
                qj = Q[i + (k+1)*NH]

                # Dubins Path Algorithm
                if LOCAL_PATH_PLANNER == 'DubinsPath':
                    px, py, pyaw, mode, length = DubinsPath.dubins_path_planning(
                        qi[0], qi[1], qi[2],
                        qj[0], qj[1], qj[2],
                        c=CURVATURE)
                    # check collision
                    # if checkCollision(env,px,py,1.4):
                    #     continue

                # set edge weight
                GQ.add_edge(h+k*NH, i+(k+1)*NH, weight=length)

    print(' Gq |V|=', GQ.number_of_nodes(), ' |E|=', GQ.number_of_edges())

    # add initial q and node
    GQ.add_node(GQ.number_of_nodes())
    for h in range(NH):
        # Dubins Path Algorithm
        if LOCAL_PATH_PLANNER == 'DubinsPath':
            px, py, pyaw, mode, length = DubinsPath.dubins_path_planning(
                q_init[0], q_init[1], q_init[2],
                Q[h][0], Q[h][1], Q[h][2],
                c=CURVATURE)
            # check collision
            # if checkCollision(env,px,py,1.4):
            # continue

        GQ.add_edge(GQ.number_of_nodes()-1,
                    h,
                    weight=length)

    # add dummy last node
    GQ.add_node(GQ.number_of_nodes())
    for h in range(NH):
        #print(h + (len(way_points) - 1)*nh)
        GQ.add_edge(h + (len(way_points)-1)*NH,
                    GQ.number_of_nodes()-1,
                    weight=0.0)

    print('C-Space pose graph |V|:', GQ.number_of_nodes())

    # find shortest path
    try:
        path = nx.shortest_path(GQ,
                                source=GQ.number_of_nodes()-2,
                                target=GQ.number_of_nodes()-1,
                                weight='weight')
    except:
        print('cannot find shortest path')
        sys.exit(-1)

    print('found shortest path')
    print('->', path)
    path = list(path)
    path.pop(-1)

    traj_node2node = []
    traj = []
    yaws = []
    total_length = 0.0

    for n, i in enumerate(path):
        if n == len(path)-1:
            break

        if i == GQ.number_of_nodes()-2:
            qi = q_init
        else:
            qi = Q[i]

        qj = Q[path[n+1]]

        if LOCAL_PATH_PLANNER == 'DubinsPath':
            px, py, pyaw, mode, length = DubinsPath.dubins_path_planning(
                qi[0], qi[1], qi[2],
                qj[0], qj[1], qj[2],
                c=CURVATURE)

        tmp_list = [(xi, yi) for xi, yi in zip(px, py)]
        traj_node2node.append(tmp_list)
        traj = traj + tmp_list
        yaws = yaws + pyaw  # pyaw 1-D array
        # yaws.append(pyaw)
        total_length += length

    # print('Total time:',round(T1+T2+T3,2),'[sec]')
    et = time.time() - st
    print('Elapsed time:', et)
    print('Total length:', round(total_length, 2), '[m]')

    fig = plt.figure(num=4, figsize=(5, 5), dpi=150)
    ax = plt.axes()
    ax.set_xlim(X_RANGE[0], X_RANGE[1])
    ax.set_ylim(Y_RANGE[0], Y_RANGE[1])
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    draw_treemap(env.Trees, ax)

    # ax.plot([pi[0] for pi in way_points],
    #         [pi[1] for pi in way_points],
    #         'bs')

    ax.plot([xi for xi, yi in traj],
            [yi for xi, yi in traj],
            'r--')

    # ax_x.plot([qi[0] for qi in traj],'.')
    # ax_y.plot([qi[1] for qi in traj],'.')
    # ax_th.plot([th for th in yaws],'.')

    fig.savefig(OUTPUT_GEN_DIR+'trajectory.png')

    np.savetxt(OUTPUT_GEN_DIR+'yaws.txt', np.array(yaws), fmt='%.3f')
