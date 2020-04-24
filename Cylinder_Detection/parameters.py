# -*- coding: utf-8 -*-
"""
Created on Apr 16 17:38:40 2020

@author: Jack Hale

This function specifies the parameters of our RANSAC algorithm

When calling read_pcd.py the variables:
    voxel_size, radius, and max_nn can be specified and take priority over those
    set in this file
    
When calling ransa.py the variables:
    cylinder_radius, inlier, num_rounds, delta, and local_zone can be specified and
    take priority over those set in this file
    
"""


"""
#####################
# RANSAC Parameters #
#####################
"""

def parameters():
    
    """ Parameters to change depending on point cloud size """
    voxel_size = 0.015 # Size of voxel boxes used for down sampling
    radius = 0.1 # Radius to search for surface normals
    max_nn= 30 # Number of nearest neighbors to consider for surface normal calculations
    
    """ Parameters for cylinder detection """
    cylinder_radius = 0.08 # Commented if the cylinder radius is defined from the sampled point to center axis
    inlier = 175 # inlier threshold
    num_rounds = 100 # Number of rounds tested
    delta = 0.01 # Noise allowed about cylinder radius
    local_zone = 0.14 # How far the second randomly sampled point can be from the first
    
    return [voxel_size, radius, max_nn, 
            cylinder_radius, inlier, num_rounds,
            delta, local_zone]