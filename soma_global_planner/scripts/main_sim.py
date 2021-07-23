import numpy as np
import math
import time
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import networkx as nx
import sys
import os

from Environment import Environment
from PathPlanner import PathPlanner
import DubinsPath

#
INPUT_TREE_MAP = '/home/hayashi/catkin_ws/src/soma_pkg/soma_global_planner/treepositions/simple-4.txt'
# INPUT_TREE_MAP = '/home/hayashi/catkin_ws/src/soma_pkg/soma_global_planner/treepositions/simple-4-2.txt'

# Work space range
X_RANGE = [0.0, 12.0]
Y_RANGE = [0.0, 10]
OFFSET = 0.0
# Rudias of tree
R_RANGE = [0.3, 0.3]

LOCAL_PATH_PLANNER = 'DubinsPath'
# CURVATURE = 0.42
CURVATURE = 2.0
q_init = [0.0, 0.0, 0.0]

if __name__ == '__main__':
  print('Graph based global trajectory planner')

  env = Environment(input_file=INPUT_TREE_MAP,
                    x_range=X_RANGE+[OFFSET, -OFFSET],
                    y_range=Y_RANGE+[OFFSET, -OFFSET],
                    r_range=R_RANGE)

  fig = plt.figure(num=1, figsize=(5, 5), dpi=100)
  plt.rcParams["font.family"] = "Times New Roman"
  plt.rcParams["xtick.direction"] = 'in'
  plt.rcParams["ytick.direction"] = 'in'
  ax = plt.axes()
  ax.set_xlim(X_RANGE[0], X_RANGE[1])
  ax.set_ylim(Y_RANGE[0], Y_RANGE[1])
  ax.set_aspect('equal')
  ax.set_xlabel('x (m)')
  ax.set_ylabel('y (m)')
  ax.grid(False)

  planner = PathPlanner(env.Trees[:, 1:3])
  way_points = np.array(planner.planning())
  print(way_points)

  fig = plt.figure(num=1, figsize=(5, 5), dpi=100)
  plt.rcParams["font.family"] = "Times New Roman"
  plt.rcParams["xtick.direction"] = 'in'
  plt.rcParams["ytick.direction"] = 'in'
  ax = plt.axes()
  ax.set_xlim(X_RANGE[0], X_RANGE[1])
  ax.set_ylim(Y_RANGE[0], Y_RANGE[1])
  ax.set_aspect('equal')
  ax.set_xlabel('x (m)')
  ax.set_ylabel('y (m)')
  ax.grid(False)

  # nx.draw_networkx_nodes(planner.G,
  #                        pos=planner.pos,
  #                        ax=ax,
  #                        node_color='g')
  # nx.draw_networkx_edges(planner.G,
  #                        pos=planner.pos,
  #                        ax=ax,
  #                        edge_color='k')
  # fig.savefig(fname='/home/hayashi/catkin_ws/src/soma_pkg/soma_global_planner/gen/G.png',
  # dpi=300)
  # plt.show()

  ax.set_aspect('equal')
  ax.set_xlim(X_RANGE[0], X_RANGE[1])
  ax.set_ylim(Y_RANGE[0], Y_RANGE[1])
  ax.grid(False)
  ax.scatter(planner.pos[:, :1],
             planner.pos[:, 1:2],
             marker='x',
             s=10.0,
             c='g')
  ax.scatter(
      way_points[:, 0],
      way_points[:, 1:2],
      marker='s',
      s=50,
      c='r'
  )
  plt.plot(way_points[:, 0],
           way_points[:, 1:2],
           c='r')
  plt.savefig(fname='/home/hayashi/catkin_ws/src/soma_pkg/soma_global_planner/gen/waypoints.png')
  # plt.show()

  Q = []
  NH = 30
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

  # make graph
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
        # print(h + k*(nh), i + (k+1)*(nh))
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
    # print(h + (len(way_points) - 1)*nh)
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
  # print('->', path)
  path = list(path)
  path.pop(-1)

  X = []
  Y = []
  TH = []
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
          c=CURVATURE,
          step_size=0.3)
      # X = X + px
      # Y = Y + py
      # TH = TH + pyaw
      # print(px.shape)
      X.append(px)
      Y.append(py)
      TH.append(pyaw)
      print(len(X), len(Y), len(TH))

  # make image
  plt.plot()
  ax.set_aspect('equal')
  ax.set_xlim(X_RANGE[0], X_RANGE[1])
  ax.set_ylim(Y_RANGE[0], Y_RANGE[1])
  ax.grid(False)
  ax.scatter(planner.pos[:, :1],
             planner.pos[:, 1:2],
             marker='x',
             s=10.0,
             c='g')
  ax.scatter(
      way_points[:, 0],
      way_points[:, 1:2],
      marker='s',
      s=50,
      c='r',
      lw=0.1
  )
  plt.plot(way_points[:, 0],
           way_points[:, 1:2],
           c='r',
           lw=0.1)

  TRAJ = zip(X, Y)
  for idx, (x, y) in enumerate(TRAJ):
    print(idx)
    for idx2, (xi, yi) in enumerate(zip(x, y)):
      ax.scatter(xi, yi,
                 marker='o',
                 s=3,
                 c='k',
                 zorder=100)

      im = '/home/hayashi/catkin_ws/src/soma_pkg/soma_global_planner/gen/waypoints_' + str(idx) + '-' + str(idx2) + '.png'
      # plt.savefig(fname=im)
      # plt.show()
      # plt.plot()
