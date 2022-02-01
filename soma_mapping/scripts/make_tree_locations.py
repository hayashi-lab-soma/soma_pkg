import numpy as np
from tkinter import filedialog
import glob
import os
import open3d as o3d

ROOT_DIR = '/mnt/d/bagfiles/hdl_graph_slam_exp/map_organize'
TREE_LOCATIONS='/home/hayashi/catkin_ws/src/soma_pkg/soma_mapping/data/tree_locations.txt'

if __name__ == '__main__':
    print('Make tree locations file')

    dir_path = filedialog.askdirectory(initialdir=ROOT_DIR)
    print('select dir:', dir_path)

    files=glob.glob(dir_path+'/*.txt')
    print(len(files))

    center_points = []
    for file in files: #absolute paths
      cloud = o3d.io.read_point_cloud(file,format='xyz')
      # print(cloud)
      # print(cloud.get_center())
      tmp = [cloud.get_center()[0], cloud.get_center()[1], 0.3]
      center_points.append(tmp)
    
    center_points = np.array(center_points)
    print(center_points)
    np.savetxt(TREE_LOCATIONS,center_points,fmt='%.3f',delimiter='\t')



