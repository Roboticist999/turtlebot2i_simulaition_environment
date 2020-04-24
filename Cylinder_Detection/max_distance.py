# -*- coding: utf-8 -*-
"""
Created on Mon Apr 20 18:07:41 2020

@author: Jack

This script is used to determine the largest distance bewtween two points in a point cloud.
As input it takes a numpy array.

"""

import numpy as np

def max_distance(down_pcd):
    
    """ Used to test which two points are the farthest apart in the point cloud 
     this allows us to get a sense for the scale of the point cloud """
    print(down_pcd)
    test = np.zeros([1,len(down_pcd.T)])
    for i in np.arange(0,len(down_pcd.T)):
        test[0,i] = np.linalg.norm(down_pcd[:,i])
    max = np.max(test)
    min = np.min(test)
    distance = max-min
    return distance