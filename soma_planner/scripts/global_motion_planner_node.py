#!/usr/bin/python

import rospy
import numpy as np
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Point

from PathPlanner import PathPlanner
import DubinsPath

import math
import networkx as nx

# Q-Space parameter
NH = 3
LOCAL_PATH_PLANNER = 'DubinsPath'
CURVATURE = 0.43
q_init = [0.0, 0.0, 0.0]

def call_back(data):
    print('num input trees:', len(data.poses))

    # to 2d point array
    tree_positions = []
    for pose in data.poses:
        # print(pose.position.x,)
        tmp = [pose.position.x,
               pose.position.y]
        tree_positions.append(tmp)

    tree_positions = np.array(tree_positions)
    print(tree_positions)

    planner = PathPlanner(tree_positions)
    way_points = planner.planning()

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
    
    rospy.loginfo('creator motion plan!!')


if __name__ == '__main__':
    rospy.init_node('global_motion_planner', anonymous=True)
    rospy.loginfo('start')
    rospy.Subscriber('tree_poses', PoseArray, call_back)
    rospy.spin()
