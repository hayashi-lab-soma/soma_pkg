#!/usr/bin/python

import numpy as np
import math
import itertools
import sys
import os
import matplotlib.pyplot as plt
import networkx as nx
from scipy.spatial import Delaunay
from scipy.spatial import distance
import pandas as pd


class PathPlanner:
  def __init__(self, landmark_pos=[]):
    self.pos = landmark_pos
    # print('landmarks ==>')
    # print(self.pos)

    # self.fig_G = plt.figure(num=1, figsize=(3, 3),)
    # self.ax_G = plt.subplot(1, 1, 1)
    # self.ax_G.set_xlim(0.0, 10.0)
    # self.ax_G.set_ylim(0.0, 10.0)

    self.G = nx.Graph()

    # self.fig_Ge = plt.figure(num=2, figsize=(3, 3),)
    # self.ax_Ge = plt.subplot(1, 1, 1)
    # self.ax_Ge.set_xlim(0.0, 10.0)
    # self.ax_Ge.set_ylim(0.0, 10.0)

  def planning(self,):
    """
    description

    Parameters
    ----------

    Returns
    -------

    """

    # print('--------------------------------------------------')
    # print('(i) Make initial graph')
    self.G = self.delaunay_network()
    print('|V|=', self.G.number_of_nodes())
    print('|E|=', self.G.number_of_edges())
    # draw initial graph G
    # nx.draw_networkx(self.G, pos=self.pos, ax=self.ax_G)
    """
    """
    # print('--------------------------------------------------')
    # print('(ii) Modify to be Eulerian')
    self.Ge = nx.MultiGraph()
    self.Ge, odd_nodes, M = self.chinese_postmap_problem(self.G)
    print('|Ve|=', self.Ge.number_of_nodes())
    print('|Ee|=', self.Ge.number_of_edges())
    # draw odd nodes on G
    # nx.draw_networkx_nodes(self.G,
    #                        pos=self.pos,
    #                        nodelist=odd_nodes,
    #                        node_color='r',
    #                        ax=self.ax_G,)
    # draw Eulerian graph Ge
    # nx.draw_networkx_edges(self.G,
    #                        pos=self.pos,
    #                        edgelist=M,
    #                        edge_color='r',
    #                        ax=self.ax_G,)

    if not nx.is_eulerian(self.Ge):
      print('Not Eulerian')
      print(self.Ge.degree())
      # sys.exit(-1)
      return None

    eularian_circuit = list(nx.eulerian_circuit(self.Ge, 0))
    # print('Eularian circuit ==>')
    # print(eularian_circuit)
    """
    """
    # print('--------------------------------------------------')
    # print('(iii) Compute Hamiltonian circuit (way points)')
    way_points = [self.h(e[0], e[1]) for e in eularian_circuit]
    way_points = [[round(p[0], 2), round(p[1], 2)] for p in way_points]
    way_points.append(way_points[0])
    # print(way_points)

    shift_way_points = way_points[1:] + way_points[:1]
    L = 0.0
    for i, j in zip(way_points, shift_way_points):
      # print(i,j)
      L = L + distance.euclidean(i, j)
    # print(L)
    return way_points

  def delaunay_network(self,):
    # print('Delaunay Network')
    triangles = Delaunay(list(self.pos))
    # print('Num triangles=', len(triangles.simplices))
    G = nx.Graph()
    for t in triangles.simplices:
      if distance.euclidean((self.pos[t[0]]), (self.pos[t[1]])) > 2.0:
        G.add_edge(t[0], t[1])
      if distance.euclidean((self.pos[t[1]]), (self.pos[t[2]])) > 2.0:
        G.add_edge(t[1], t[2])
      if distance.euclidean((self.pos[t[2]]), (self.pos[t[0]])) > 2.0:
        G.add_edge(t[2], t[0])
    return G

  def chinese_postmap_problem(self, Gin):
    # print(type(Gin.degree()))
    odd_nodes = [v for (v, d) in Gin.degree() if d % 2 == 1]
    # odd_nodes = [v for (v, d) in Gin.degree().items() if d % 2 == 1]
    # print('Odd nodes ==>')
    # print(odd_nodes)
    odd_node_pairs = list(itertools.combinations(odd_nodes, 2))
    # print('Odd nodes pairs ==>')
    print(odd_node_pairs)
    cost_list = {}
    for pair in odd_node_pairs:
      cost_list[pair] = nx.dijkstra_path_length(Gin,
                                                pair[0],
                                                pair[1],
                                                weight='weight')
    # print('Cost list ==>')
    print(cost_list)

    Gc = nx.Graph()
    for k, v in cost_list.items():
      Gc.add_edge(k[0], k[1], **{'distance': v, 'weight': v})

    odd_matching_dupes = nx.max_weight_matching(Gc)
    M = list(pd.unique([tuple(sorted([k, v]))
                        for k, v in odd_matching_dupes.items()]))
    # print('Matching M ==>')
    # print(M)

    Geular = nx.MultiGraph()
    Geular.add_nodes_from(Gin.nodes(), pos=self.pos)
    Geular.add_edges_from(Gin.edges())

    # print('Add paths ==>')
    for (m1, m2) in M:
      path = nx.dijkstra_path(Gin,
                              m1,
                              m2,
                              weight='weight')
      print(path)
      nx.add_path(Geular, path)

    return Geular, odd_nodes, M,

  def h(self, vs, vt):
    p = [(self.pos[vs][0]+self.pos[vt][0])/2.0,
         (self.pos[vs][1]+self.pos[vt][1])/2.0]
    return p

  def save_graphs(self, file_path='./'):
    # save node list
    nodes = list(self.G.nodes())
    nodes.sort()
    nodelist = []
    for v in nodes:
      x = self.pos[v][0]
      y = self.pos[v][1]
      nodelist.append([v, x, y])
    np.savetxt(file_path+'G-nodelist.txt',
               np.array(nodelist), fmt='%.3f', delimiter='\t')

    # save graph as edge list of G
    with open(file_path+'G-edgelist.txt', 'w') as f:
      for (vs, vt) in self.G.edges():
        xs = self.pos[vs][0]
        ys = self.pos[vs][1]
        xt = self.pos[vt][0]
        yt = self.pos[vt][1]
        f.write('{}\t{}\n{}\t{}\n\n'.format(xs, ys, xt, yt))

    with open(file_path+'Ge-edgelist.txt', 'w') as f:
      for (vs, vt) in self.Ge.edges():
        xs = self.pos[vs][0]
        ys = self.pos[vs][1]
        xt = self.pos[vt][0]
        yt = self.pos[vt][1]
        f.write('{}\t{}\n{}\t{}\n\n'.format(xs, ys, xt, yt))

  def view(self,):
    plt.show()
