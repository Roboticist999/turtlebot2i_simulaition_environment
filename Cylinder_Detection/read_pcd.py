# -*- coding: utf-8 -*-
"""
Created on Apr 16 17:38:49 2020

@author: Jack Hale


This function reads a point cloud file and has the potion of 3 outputs
0 - downsamples point cloud and outputs arrays for the points and surface normals
1 - Visualizes downsampled point cloud and surface normals (press n to see surface normals)
2 - Visualizes full point cloud and returns it as an array of points

When calling this function, one can specify the voxel size for down sampling,
the radius to search for nearby points for surface normal calculation, 
and the max number of nearest neighbors considered for surface normal calculation.

If no values are specified the function defaults to the values in parameters.py

"""
import open3d as o3d
import numpy as np

from parameters import parameters

"""
############################
# Reading point cloud data #
############################
"""


def read_pcd(path, mode=0, voxel_size=0, radius=0, max_nn=0):
    
    """ Importanting parameters """
    [voxel_size_temp, radius_temp, max_nn_temp, 
     cylinder_radius, inlier, num_rounds, 
     delta, local_zone] = parameters()
    
    """
    If point cloud down sampling points are defined when the function is called
    then we use those values, otherwise the defaults from the parameters function
    are used
    """
    
    if voxel_size == 0:
        voxel_size = voxel_size_temp
    if radius == 0:
        radius = radius_temp
    if max_nn == 0:
        max_nn = max_nn_temp
    
    
    """ Creates a point cloud object for ply file """
    pcd = o3d.io.read_point_cloud(path)
    
    # Downsamples point cloud and outputs arrays for the points and surface normals
    if mode == 0:
        """ Downsample point cloud and calculate the surface normals """
        
        # Downsample point cloud
        downpcd = pcd.voxel_down_sample(voxel_size)
        # Calculate surface normals
        downpcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius, max_nn))
        
        down_pcd = np.asarray(downpcd.points) # Create numpy array for downsampled point cloud
        down_pcd = down_pcd.T # 3XN array
        
        normals_pcd = np.asarray(downpcd.normals) # Create numpy array of the surface normals
        # arrays must be transposed so that they are 3xN
        normals_pcd = normals_pcd.T # 3XN array
    
        # print(downpcd.normals)
        return [down_pcd, normals_pcd]
    
    # Visualizes downsampled point cloud and surface normals (press n to see surface normals)
    if mode == 1:
        # Downsample point cloud
        downpcd = pcd.voxel_down_sample(voxel_size)
        # Calculate surface normals
        downpcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius, max_nn))
        down_pcd = np.asarray(downpcd.points) # Create numpy array for downsampled point cloud
        down_pcd = down_pcd.T # 3XN array
        
        o3d.visualization.draw_geometries_with_vertex_selection([downpcd]) # Down sampled point cloud
        return [down_pcd, 1]
        
    # Visualizes full point cloud and returns it as an array of points
    if mode == 2:
        """ Visualizes either full point cloud """
        o3d.visualization.draw_geometries_with_vertex_selection([pcd]) # Full point cloud
        full_pcd = np.asarray(pcd.points) # Create numpy array for full point cloud
        
        return [full_pcd, 1]
    
    
    