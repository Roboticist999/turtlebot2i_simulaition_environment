# -*- coding: utf-8 -*-
"""
Created on Apr 16 09:44:20 2020

@author: Jack Hale

This is the main function used for RANSAC object detection

Documentation on the functions written for this purpose can be seen in their
respective scripts

To change the default parameters for this RANSAC algorithm, reference the 
parameters.py script

# Function Tips #

read_pcd.py:
    We can specify voxel size, radius, and nearest neighbor parameters for surface
        normal calculation and downsampling size. If none are passed parameters from parameters.py
        are used
    There are 3 modes for this function. Their differences can be seen in the read_pcd.py script

ransac.py:
    We can specify cylinder radius, number of inliers allow, number of rounds to run
        the algorithm, the allowable noise about the cylinder radius, the distance a point
        can be from the sampled point based off the cylinder radius, and the size of the zone around
        the sampled point the will have it's points checked
    There are two modes. Static radius and dynamic. Static reads it's radius from the parameters file
        or that specified when the function is called. Dyanmic takes the cylinder radius to be from the
        center of the theoretical axis between the two sampled points to the first sampled point projected
        on an orthogonal plane
    
"""

"""
#######################
# Importing Functions #
#######################
"""

from read_pcd import read_pcd
from ransac import ransac
# from visualize_pcd import visualize_pcd
# from max_distance import max_distance  # This function can be called to determine the maximum distance between points in a point cloud array


from mpl_toolkits import mplot3d # Needed for 3d projection on scatter plots
import numpy as np
import matplotlib.pyplot as plt



"""
#######################
# Objection Detection #
#######################
"""

""" Defines the point cloud we are analysing """
path = 'object3d.ply'

""" Obtaining down sampled point cloud """
[down_pcd, normals_pcd] = read_pcd(path, mode=0)

""" Running RANSAC """
[cylinder, first, second, center, radius, x_plane, norm_axis] = ransac(down_pcd, normals_pcd, static_radius=0)


""" Finding farthest distances between points """
# distance = max_distance(down_pcd)
# print(distance)


"""
#################
# Visualization #
#################
"""

# If the RANSAC algorithm passed a 1D all zero array no cylinder was found
if radius == 0:
    print('No cylinder found')
else:
    
    fig = plt.figure
    ax = plt.axes(projection='3d')
    """
    This code plots 6 different things:
        Found cylinder in red
        Down sampled point cloud in yellow
        The first and second sampled points in green
        The local points in blue projected onto an orthogonal axis
        The calculated center point on the orthogonal plane in a larger blue
    """        
    line = np.arange(-0.5,0.5,0.05)
    line_x = center[0]+norm_axis.T[0]*line
    line_y = center[1]+norm_axis.T[1]*line
    line_z = center[2]+norm_axis.T[2]*line
    
    plt.title(radius)
    ax.scatter3D(cylinder[0,:],cylinder[1,:],cylinder[2,:], color='r', s=10)
    ax.scatter3D(down_pcd[0,:],down_pcd[1,:],down_pcd[2,:], color='y', s=0.5)
    ax.scatter3D(first[0], first[1], first[2], color='g',s=30)
    ax.scatter3D(second[0], second[1], second[2], color='g', s=30)
    ax.scatter3D(center[0], center[1], center[2], color='b', s=30)
    ax.scatter3D(x_plane[0,:],x_plane[1,:],x_plane[2,:], color='b', s=5)
    ax.scatter3D(line_x, line_y , line_z, color = 'g')




"""
################
# Sample Codes #
################
"""

""" 
These code blocks serve as examples for the modular functionality of our other funcitons 
"""



""" Runs the ransac function until a suitable cylinder is found """
# i = 0
# while i == 0:
#     [cylinder, first, second, center, radius, x_plane, norm_axis] = ransac(down_pcd, normals_pcd, static_radius=0)
#     if radius == 0:
#         print('No cylinder found')
#     else:
#         i = 1
#         # visualize_pcd(cylinder)
            
#         fig = plt.figure
#         ax = plt.axes(projection='3d')
#         """
#         This code plots 6 different things:
#             Found cylinder in red
#             Down sampled point cloud in yellow
#             The first and second sampled points in green
#             The local points in blue projected onto an orthogonal axis
#             The calculated center point on the orthogonal plane in a larger blue
#         """        
#         line = np.arange(-1,1,0.05)
#         line_x = center[0]+norm_axis.T[0]*line
#         line_y = center[1]+norm_axis.T[1]*line
#         line_z = center[2]+norm_axis.T[2]*line
        
#         plt.title(radius)
#         ax.scatter3D(cylinder[0,:],cylinder[1,:],cylinder[2,:], color='r', s=10)
#         ax.scatter3D(down_pcd[0,:],down_pcd[1,:],down_pcd[2,:], color='y', s=0.5)
#         ax.scatter3D(first[0], first[1], first[2], color='g',s=30)
#         ax.scatter3D(second[0], second[1], second[2], color='g', s=30)
#         ax.scatter3D(center[0], center[1], center[2], color='b', s=30)
#         ax.scatter3D(x_plane[0,:],x_plane[1,:],x_plane[2,:], color='b', s=5)
#         ax.scatter3D(line_x, line_y , line_z, color = 'g')
    
    
    
""" Runs through all possible cylinder diameters for our sample point cloud """
# count = 0
# for i in [0.05,0.06,0.7,0.08,0.09,0.1]:
#     [cylinder, first, second, center,radius,x_plane, norm_axis] = ransac(down_pcd, normals_pcd, static_radius=1, cylinder_radius=i)
#     if radius == 0:
#         print('No cylinder found')
#     else:
#         # visualize_pcd(cylinder)
#         plt.figure(count)
#         ax = plt.axes(projection='3d')
#         """
#         This code plots 6 different things:
#             Found cylinder in red
#             Down sampled point cloud in yellow
#             The first and second sampled points in green
#             The local points in blue projected onto an orthogonal axis
#             The calculated center point on the orthogonal plane in a larger blue
#         """        
#         line = np.arange(-1,1,0.05)
#         line_x = center[0]+norm_axis.T[0]*line
#         line_y = center[1]+norm_axis.T[1]*line
#         line_z = center[2]+norm_axis.T[2]*line
        
#         plt.title(radius)
#         ax.scatter3D(cylinder[0,:],cylinder[1,:],cylinder[2,:], color='r', s=10)
#         ax.scatter3D(down_pcd[0,:],down_pcd[1,:],down_pcd[2,:], color='y', s=0.5)
#         ax.scatter3D(first[0], first[1], first[2], color='g',s=30)
#         ax.scatter3D(second[0], second[1], second[2], color='g', s=30)
#         ax.scatter3D(center[0], center[1], center[2], color='b', s=30)
#         ax.scatter3D(x_plane[0,:],x_plane[1,:],x_plane[2,:], color='b', s=5)
#         ax.scatter3D(line_x, line_y , line_z, color = 'g')
#         count+=1
#         # break