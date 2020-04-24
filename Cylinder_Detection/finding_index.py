# -*- coding: utf-8 -*-
"""
Created on Apr 20 11:19:00 2020

@author: Jack Hale

This script is used to determine the closest point in a point cloud to 
the coordinates you specify, and outputs the index of that closest point.
"""

import open3d as o3d
import numpy as np

from parameters import parameters

[voxel_size, radius, max_nn, 
     cylinder, inlier, num_rounds, 
     delta, local_zone] = parameters()

path = 'object3d.ply'

pcd = o3d.io.read_point_cloud(path)

# Downsample point cloud
downpcd = pcd.voxel_down_sample(voxel_size)
# Calculate surface normals
downpcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius, max_nn))
        
down_pcd = np.asarray(downpcd.points) # Create numpy array for downsampled point cloud
down_pcd = down_pcd.T # 3XN array
        
normals_pcd = np.asarray(downpcd.normals) # Create numpy array of the surface normals
# arrays must be transposed so that they are 3xN
normals_pcd = normals_pcd.T # 3XN array

first = [0.566255,0.0140978,0.133984]
# second = [0.538777,0.0531362,0.211776]

offset = 0.01
index =0
# print(np.shape(down_pcd))

switch = down_pcd.T

for i in np.arange(len(down_pcd.T)):
    if switch[i,0] < (first[0]+offset) and switch[i,0] > (first[0]-offset):
        if switch[i,1] < (first[1]+offset) and switch[i,1] > (first[1]-offset):
            if switch[i,2] < (first[2]+offset) and switch[i,2] > (first[2]-offset):
                index = i
        
        
print(index)