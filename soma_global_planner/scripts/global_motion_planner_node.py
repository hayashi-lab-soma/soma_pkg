#!/usr/bin/python

import rospy
from tf.transformations import *
import numpy as np
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray  # for waypoints visualization

from PathPlanner import PathPlanner
import DubinsPath

import itertools
import math
import networkx as nx
from scipy.spatial import KDTree

# SOMA parameters
WHEEL_BASE = 1.04  # [m]
# MIN_CURVATURE = 0.43  # minimum curvature
MIN_CURVATURE = 0.406

# Q-Space parameter
NH = 30
LOCAL_PATH_PLANNER = 'DubinsPath'
DUBINS_STEP = 0.5
q_init = [1.0, 1.0, 0.0]

# ros publisher
pub_waypoints = None  # publisher
pub_path = None

# way points marker arguments
WAY_POINTS_SIZE = 0.2   # (m)
WAY_POINTS_COLOR = (1.0, 0.0, 0.0, 1.0)  # (r, g, b, a)
# way lines arguments
WAY_LINES_WIDTH = 0.05
WAY_LINES_COLOR = (1.0, 0.0, 0.0, 1.0)  # (r, g, b, a)


def make_marker_array_msg(_waypoints):
  """
  Make MarkerArray message for visualization in rviz

  Parameters
  ----------

  Returns
  -------

  """

  marker_arry = MarkerArray()

  stamp = rospy.Time.now()
  frame_id = 'map'

  waypoints = Marker()
  waypoints.header.stamp = stamp
  waypoints.header.frame_id = frame_id
  waypoints.type = Marker.POINTS
  waypoints.action = Marker.ADD
  waypoints.scale.x = WAY_POINTS_SIZE
  waypoints.scale.y = WAY_POINTS_SIZE
  waypoints.scale.z = WAY_POINTS_SIZE
  waypoints.color.r = WAY_POINTS_COLOR[0]
  waypoints.color.g = WAY_POINTS_COLOR[1]
  waypoints.color.b = WAY_POINTS_COLOR[2]
  waypoints.color.a = WAY_POINTS_COLOR[3]
  waypoints.ns = 'waypoints'

  for (x, y) in _waypoints:
    p = Point()
    p.x = x
    p.y = y
    p.z = 0.0
    waypoints.points.append(p)

  waylines = Marker()
  waylines.header.stamp = stamp
  waylines.header.frame_id = frame_id
  waylines.type = Marker.LINE_STRIP
  waylines.action = Marker.ADD
  waylines.scale.x = WAY_LINES_WIDTH
  waylines.scale.y = WAY_LINES_WIDTH
  waylines.scale.z = WAY_LINES_WIDTH
  waylines.color.r = WAY_LINES_COLOR[0]
  waylines.color.g = WAY_LINES_COLOR[1]
  waylines.color.b = WAY_LINES_COLOR[2]
  waylines.color.a = WAY_LINES_COLOR[3]
  waylines.ns = 'waylines'

  for (x, y) in _waypoints:
    p = Point()
    p.x = x
    p.y = y
    p.z = 0.0
    waylines.points.append(p)

  marker_arry.markers.append(waypoints)
  marker_arry.markers.append(waylines)
  pub_waypoints.publish(marker_arry)


def configuration_sampling(pts, nh):
  """
  Sampling nh-th Theta from C-space with (x,y)

  Parameters
  ----------
  pts: numpy.array 2-D
  nh: int

  Returns
  -------
  Q: list

  """

  Q = []
  for i, p in enumerate(pts):
    x_i = p[0]
    y_i = p[1]

    for h in range(nh):  # theta axis
      ang = h/(NH-1)*360.0  # degree
      ang = round(math.radians(ang), 2)
      Q.append([x_i, y_i, ang])

  Q = np.array(Q)
  return Q


def call_back(data):

  print('num input trees:', len(data.poses))
  tree_positions = [[pose.position.x, pose.position.y]
                    for pose in data.poses]
  tree_positions = np.array(tree_positions)
  kd_tree = KDTree(tree_positions, leafsize=3)

  # create planner and waypoints
  planner = PathPlanner(tree_positions)
  way_points = planner.planning()
  print('way points ({}) : {}'.format(type(way_points), len(way_points)))

  Q = configuration_sampling(way_points, NH)
  print('|Q|=', len(Q))

  print('Make pose graph ==========')
  set_kh_tup = list(itertools.product(
      range(len(way_points)), range(NH)))  # create touple (k,h)
  GQ = nx.DiGraph()

  # add nodes
  for k, h in set_kh_tup:
    GQ.add_node(h + k*NH)
  # add edges
  for k, h in set_kh_tup:
    if k >= len(way_points)-1:
      break
    for i in range(NH):
      qi = Q[h + k*NH]        # current way points
      qj = Q[i + (k+1)*NH]    # next way points

      if LOCAL_PATH_PLANNER == 'DubinsPath':  # Dubins Path Algorithm
        px, py, pyaw, mode, length = DubinsPath.dubins_path_planning(
            qi[0], qi[1], qi[2],  # source pose
            qj[0], qj[1], qj[2],  # target pose
            c=MIN_CURVATURE,
            step_size=DUBINS_STEP)

      # minimum distance to obstacle
      min_d = float('inf')
      for p in zip(px,py):
        d,i = kd_tree.query(p, k=1)
        if d < min_d:
          min_d = d

      # set edge weight
      GQ.add_edge(h+k*NH,
                  i+(k+1)*NH,
                  weight=min_d)

  # print(' Num edges: ', GQ.number_of_edges())

  # add weighted edges
  # for k in range(len(way_points)-1):
  #     for h in range(NH):
  #         for i in range(NH):
  #             qi = Q[h + k*NH]
  #             qj = Q[i + (k+1)*NH]

  #             # Dubins Path Algorithm
  #             if LOCAL_PATH_PLANNER == 'DubinsPath':
  #                 px, py, pyaw, mode, length = DubinsPath.dubins_path_planning(
  #                     qi[0], qi[1], qi[2],
  #                     qj[0], qj[1], qj[2],
  #                     c=CURVATURE)
  #                 # check collision
  #                 # if checkCollision(env,px,py,1.4):
  #                 #     continue

  #             # set edge weight
  #             GQ.add_edge(h+k*NH, i+(k+1)*NH, weight=length)

  print(' Gq : |V|=', GQ.number_of_nodes(), ' |E|=', GQ.number_of_edges())

  # add initial q as node at last of list
  GQ.add_node(GQ.number_of_nodes())
  for h in range(NH):
    # Dubins Path Algorithm
    if LOCAL_PATH_PLANNER == 'DubinsPath':
      px, py, pyaw, mode, length = DubinsPath.dubins_path_planning(
          q_init[0], q_init[1], q_init[2],
          Q[h][0], Q[h][1], Q[h][2],
          c=MIN_CURVATURE,
          step_size=DUBINS_STEP)
      # check collision
      # if checkCollision(env,px,py,1.4):
      # continue
    GQ.add_edge(GQ.number_of_nodes()-1,
                h,
                weight=length)

  GQ.add_node(GQ.number_of_nodes())
  for h in range(NH):
    # print(h + (len(way_points) - 1)*nh)
    GQ.add_edge(h + (len(way_points)-1)*NH,
                GQ.number_of_nodes()-1,
                weight=0.0)

  try:
    path = nx.shortest_path(GQ,
                            source=GQ.number_of_nodes()-2,
                            target=GQ.number_of_nodes()-1,
                            weight='weight')
  except:
    print('cannot find shortest path')
    return

  print('found shortest path')
  print('->', path)
  path = list(path)
  path.pop(-1)

  # add initial q and node
  # GQ.add_node(GQ.number_of_nodes())
  # for h in range(NH):
  #     # Dubins Path Algorithm
  #     if LOCAL_PATH_PLANNER == 'DubinsPath':
  #         px, py, pyaw, mode, length = DubinsPath.dubins_path_planning(
  #             q_init[0], q_init[1], q_init[2],
  #             Q[h][0], Q[h][1], Q[h][2],
  #             c=CURVATURE)
  #         # check collision
  #         # if checkCollision(env,px,py,1.4):
  #         # continue

  #     GQ.add_edge(GQ.number_of_nodes()-1,
  #                 h,
  #                 weight=length)

  # add dummy last node
  # GQ.add_node(GQ.number_of_nodes())
  # for h in range(NH):
  #     # print(h + (len(way_points) - 1)*nh)
  #     GQ.add_edge(h + (len(way_points)-1)*NH,
  #                 GQ.number_of_nodes()-1,
  #                 weight=0.0)

  # print('C-Space pose graph |V|:', GQ.number_of_nodes())

  # find shortest path
  # try:
  #     path = nx.shortest_path(GQ,
  #                             source=GQ.number_of_nodes()-2,
  #                             target=GQ.number_of_nodes()-1,
  #                             weight='weight')
  # except:
  #     print('cannot find shortest path')
  #     sys.exit(-1)

  # print('found shortest path')
  # print('->', path)
  # path = list(path)
  # path.pop(-1)

  traj_node2node = []
  traj = []
  yaws = []
  total_length = 0.0

  path_msg = Path()
  path_msg.header.stamp = rospy.Time.now()
  path_msg.header.frame_id = 'map'

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
          c=MIN_CURVATURE,
          step_size=DUBINS_STEP)

    # tmp_list = [(xi, yi) for xi, yi in zip(px, py)]
    # traj_node2node.append(tmp_list)
    # traj = traj + tmp_list
    # yaws = yaws + pyaw  # pyaw 1-D array
    # yaws.append(pyaw)
    total_length += length

    for (xi, yi, thi) in zip(px, py, pyaw):
      _pose = PoseStamped()
      _pose.header.stamp = path_msg.header.stamp
      _pose.header.frame_id = path_msg.header.frame_id
      _pose.pose.position.x = xi
      _pose.pose.position.y = yi
      _pose.pose.position.z = 0.0
      q = quaternion_from_euler(0, 0, thi)
      _pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
      path_msg.poses.append(_pose)

  print(len(path_msg.poses))

  rospy.loginfo('creator motion plan!!')
  make_marker_array_msg(way_points)
  pub_path.publish(path_msg)

if __name__ == '__main__':
  rospy.init_node('global_motion_planner', anonymous=True)
  rospy.loginfo('start')
  pub_waypoints = rospy.Publisher(
      '/soma/waypoints', MarkerArray, queue_size=3)
  pub_path = rospy.Publisher('/soma/global_trajectory', Path, queue_size=3)
  rospy.Subscriber('tree_poses', PoseArray, call_back)

  rospy.

  rospy.spin()
