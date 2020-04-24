# -*- coding: utf-8 -*-
"""
Created on Mon Apr 20 18:47:20 2020

@author: Jack

This function takes an array of points, converts them into a point cloud, and
visualizes it
"""

import open3d as o3d

def visualize_pcd(pcd):
    # Makes sure that the orientation of our array can be converted into a point cloud
    if len(pcd) == 3:
        pcd = pcd.T # Arrays must be Nx3
        
    # Visualizes resultant point cloud
    pcd_test = o3d.geometry.PointCloud()
    pcd_test.points = o3d.utility.Vector3dVector(pcd)
    o3d.visualization.PointColorOption
    o3d.visualization.draw_geometries_with_vertex_selection([pcd_test])
    