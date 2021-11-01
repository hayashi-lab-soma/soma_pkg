import numpy as np
import math
import random as rnd
from scipy.spatial import KDTree
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import cv2
import os


class Environment:
    def __init__(self,
                 input_file='',
                 TreeN=10,
                 x_range=[0.0, 10.0],
                 y_range=[0.0, 10.0],
                 r_range=[-math.pi, math.pi]):

        self.x_range = x_range  # random forest x range(m) [min,max]
        self.y_range = y_range  # random forest y range(m) [min,max]
        self.r_range = r_range  # range of tree radius(rad) [min, max]

        if input_file == '':
            print('Create environment by random')
            self.TreeN = TreeN  # number of trees
            self.Trees = []  # set of tree data [x,y,r]

            # random sampling
            while len(self.Trees) < self.TreeN:
                xi = rnd.uniform(self.x_range[0], self.x_range[1])
                yi = rnd.uniform(self.y_range[0], self.y_range[1])
                ri = rnd.uniform(self.r_range[0], self.r_range[1])

                if len(self.Trees) >= 1:
                    kdTree = KDTree([[ti[0], ti[1]] for ti in self.Trees])
                    # neighborhood tree from query
                    d, j = kdTree.query([xi, yi])
                    if d <= 2.5:
                        continue

                self.Trees.append([xi, yi, ri])

            self.Trees = np.array(self.Trees)
        else:
            print('Create environment from file:', input_file)
            self.Trees = np.loadtxt(input_file)
            self.TreeN = len(self.Trees)
            self.isFromFile = True

        self.fig = plt.figure(num=1,figsize=(5,5), dpi=150)
        self.ax = plt.axes()
        self.ax.set_xlim(self.x_range[0],self.x_range[1])
        self.ax.set_ylim(self.y_range[0],self.y_range[1])
        self.ax.set_aspect('equal')
        self.ax.set_xlabel('x (m)')
        self.ax.set_ylabel('y (m)')

    def show(self,):
      print('Trees ==>')
      print(self.Trees)
    
    def view(self,):
      for x,y,r in self.Trees:
        c = patches.Circle(xy=(x,y),radius=r,fc='g',ec='g')
        self.ax.add_patch(c)
      plt.show()

    def save_png(self,file_path='./'):
      print('Save tree image :', os.path.abspath(file_path))
      self.fig.savefig(file_path)

    def save_positions(self,file_path='./'):
      print('Save tree positions :', os.path.abspath(file_path))
      np.savetxt(file_path, self.Trees, fmt='%.3f',delimiter='\t')

if __name__ == '__main__':
    env = Environment(TreeN=4,r_range=[0.5,0.5])
    env.show()
    env.view()
    # env.save_positions()
